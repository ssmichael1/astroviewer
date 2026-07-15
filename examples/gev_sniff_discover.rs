//! Find a GigE camera that won't answer ordinary discovery, by sniffing its
//! DISCOVERY_ACK off the datalink (BPF) — below the EDR/VPN socket filter that
//! silently drops the inbound reply. We still *send* the limited-broadcast
//! discovery from a normal UDP socket (outbound is fine); we just listen for the
//! answer with libpcap instead of recv()'ing it on the socket.
//!
//!   cargo run --example gev_sniff_discover --features gev -- --iface en8 --host-ip 192.168.0.1
//!
//! Prints each camera's MAC, current IP, and model — the MAC is what you feed to
//! `gev_force_ip --mac <addr> --ip 192.168.0.2` to move it onto your subnet.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use tokio::net::UdpSocket;
use viva_gige::gvcp;
use viva_gige::nic::Iface;

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    let args = parse_args();
    let _ = tracing_subscriber::fmt().with_max_level(tracing::Level::INFO).try_init();

    let host_ip: Ipv4Addr = args.get("host-ip").map_or(Ok(Ipv4Addr::new(192, 168, 0, 1)), |s| s.parse())?;
    let iface = match args.get("iface") {
        Some(name) => Iface::from_system(name)?,
        None => Iface::from_ipv4(host_ip)?,
    };
    let iface_ip = iface.ipv4().ok_or_else(|| anyhow::anyhow!("interface {} has no IPv4", iface.name()))?;
    let secs: u64 = args.get("wait").map_or(Ok(5), |s| s.parse())?;
    println!("Sniffing GVCP discovery acks on {} (ip {}) for {secs}s…", iface.name(), iface_ip);

    // Open the datalink tap first, so we don't miss a fast reply.
    let mut cap = pcap::Capture::from_device(iface.name())?
        .immediate_mode(true)
        .snaplen(2048)
        .timeout(200)
        .open()?;
    // Discovery acks come from the camera's GVCP port 3956 back to our ephemeral
    // port; broadcast cmd also rides 3956. Grab both directions on that port.
    cap.filter("udp and port 3956", true)?;

    // Fire the limited-broadcast discovery from a normal socket (outbound works).
    let sock = UdpSocket::bind(SocketAddr::new(IpAddr::V4(iface_ip), 0)).await?;
    sock.set_broadcast(true)?;
    let mut pkt = [0u8; 8];
    pkt[0] = 0x42; // GVCP cmd key
    pkt[1] = 0x01 | 0x10; // ACK_REQUIRED | BROADCAST
    pkt[2..4].copy_from_slice(&0x0002u16.to_be_bytes()); // DISCOVERY_CMD
    pkt[6..8].copy_from_slice(&1u16.to_be_bytes()); // request id
    sock.send_to(&pkt, SocketAddr::new(IpAddr::V4(Ipv4Addr::BROADCAST), gvcp::GVCP_PORT)).await?;
    println!("sent limited-broadcast discovery → 255.255.255.255:3956");

    let deadline = Instant::now() + Duration::from_secs(secs);
    let mut seen: Vec<[u8; 6]> = Vec::new();
    // Resend periodically in case the first is missed.
    let mut next_send = Instant::now() + Duration::from_secs(1);
    while Instant::now() < deadline {
        if Instant::now() >= next_send {
            let _ = sock.send_to(&pkt, SocketAddr::new(IpAddr::V4(Ipv4Addr::BROADCAST), gvcp::GVCP_PORT)).await;
            next_send += Duration::from_secs(1);
        }
        match cap.next_packet() {
            Ok(p) => {
                // The Ethernet source MAC of the reply frame is the camera's real
                // NIC MAC — authoritative, unlike payload-offset guessing.
                if p.data.len() < 14 { continue; }
                let mut src_mac = [0u8; 6];
                src_mac.copy_from_slice(&p.data[6..12]);
                if let Some(payload) = udp_payload(p.data) {
                    if let Some((ip, mfr, model)) = parse_discovery_ack(payload) {
                        if !seen.contains(&src_mac) {
                            seen.push(src_mac);
                            println!("  ✓ camera: mac {} (eth src)  reported-ip {ip}  [{mfr} {model}]", fmt_mac(src_mac));
                            print!("    payload[0..48]:");
                            for b in &payload[..48.min(payload.len())] { print!(" {b:02x}"); }
                            println!();
                        }
                    }
                }
            }
            Err(pcap::Error::TimeoutExpired) => {}
            Err(e) => return Err(e.into()),
        }
    }

    if seen.is_empty() {
        println!("\nNo discovery ack seen on the wire either. The camera isn't answering at all —");
        println!("check link/power, or it may be mid-boot. (If it answered, we'd see it here even");
        println!("when the VPN drops it at the socket.)");
    } else {
        let mac = seen[0];
        println!("\nTo move it to 192.168.0.2 persistently:");
        println!("  cargo run --example gev_force_ip --features gev -- --mac {} --ip 192.168.0.2", fmt_mac(mac));
        println!("then (host already on 192.168.0.x) re-run with --persist to make it stick.");
    }
    Ok(())
}

/// Extract the UDP payload from a captured Ethernet (DLT_EN10MB) frame.
fn udp_payload(frame: &[u8]) -> Option<&[u8]> {
    if frame.len() < 14 { return None; }
    let mut l3 = 14;
    let mut ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    if ethertype == 0x8100 {
        if frame.len() < 18 { return None; }
        ethertype = u16::from_be_bytes([frame[16], frame[17]]);
        l3 = 18;
    }
    if ethertype != 0x0800 || frame.len() < l3 + 20 { return None; }
    let ihl = (frame[l3] & 0x0f) as usize * 4;
    if ihl < 20 || frame[l3 + 9] != 17 { return None; }
    let payload = l3 + ihl + 8;
    if frame.len() < payload { return None; }
    Some(&frame[payload..])
}

/// Parse a GVCP DISCOVERY_ACK payload (after the 8-byte GVCP header).
fn parse_discovery_ack(buf: &[u8]) -> Option<(Ipv4Addr, String, String)> {
    if buf.len() < 8 + 40 { return None; }
    if u16::from_be_bytes([buf[2], buf[3]]) != 0x0003 { return None; }
    let p = &buf[8..];
    let ip = Ipv4Addr::new(p[36], p[37], p[38], p[39]);
    let mfr = fixed_string(&p[72..(72 + 32).min(p.len())]);
    let model = if p.len() >= 136 { fixed_string(&p[104..136]) } else { String::new() };
    Some((ip, mfr, model))
}

fn fixed_string(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

fn fmt_mac(mac: [u8; 6]) -> String {
    mac.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

fn parse_args() -> std::collections::HashMap<String, String> {
    let mut map = std::collections::HashMap::new();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        if let Some(key) = args[i].strip_prefix("--") {
            if i + 1 < args.len() && !args[i + 1].starts_with("--") {
                map.insert(key.to_string(), args[i + 1].clone());
                i += 2;
            } else {
                map.insert(key.to_string(), "true".to_string());
                i += 1;
            }
        } else { i += 1; }
    }
    map
}
