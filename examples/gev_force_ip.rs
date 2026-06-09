//! GigE Vision force-IP test tool.
//!
//! When a camera powers up with an IP on a different subnet than your NIC, you
//! can't open it — and ordinary discovery (sent to the *subnet-directed*
//! broadcast, e.g. 192.168.0.255) won't even find it, because the camera's IP
//! stack ignores broadcasts outside its own subnet. So this tool discovers via
//! the **limited broadcast 255.255.255.255**, which every GigE camera on the
//! link accepts regardless of its current IP, learns the camera's MAC, and then
//! issues a FORCEIP command to move it onto your adapter's subnet.
//!
//! Defaults are tuned for a Belkin USB-Ethernet adapter at 192.168.0.1:
//!   - host NIC ....... 192.168.0.1   (override: --host-ip)
//!   - camera target .. 192.168.0.10  (override: --ip)
//!   - subnet mask .... 255.255.255.0 (override: --subnet)
//!   - gateway ........ 0.0.0.0       (override: --gateway)
//!
//! Run:
//!   cargo run --example gev_force_ip --features gev -- --discover-only
//!   cargo run --example gev_force_ip --features gev
//!   cargo run --example gev_force_ip --features gev -- --ip 192.168.0.20
//!   cargo run --example gev_force_ip --features gev -- --mac 00:11:1c:aa:bb:cc
//!
//! The assigned IP is temporary (does not survive a camera power-cycle). For a
//! permanent address use the camera's persistent-IP registers.

use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use tokio::net::UdpSocket;
use viva_gige::gvcp;
use viva_gige::nic::Iface;

/// A camera found via limited-broadcast discovery.
#[derive(Clone)]
struct Found {
    mac: [u8; 6],
    ip: Ipv4Addr,
    manufacturer: String,
    model: String,
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    let args = parse_args();

    let level = if args.contains_key("verbose") { tracing::Level::TRACE } else { tracing::Level::INFO };
    let _ = tracing_subscriber::fmt().with_max_level(level).with_target(true).try_init();

    let host_ip: Ipv4Addr = args.get("host-ip").map_or(Ok(Ipv4Addr::new(192, 168, 0, 1)), |s| s.parse())?;
    let target_ip: Ipv4Addr = args.get("ip").map_or(Ok(Ipv4Addr::new(192, 168, 0, 10)), |s| s.parse())?;
    let subnet: Ipv4Addr = args.get("subnet").map_or(Ok(Ipv4Addr::new(255, 255, 255, 0)), |s| s.parse())?;
    let gateway: Ipv4Addr = args.get("gateway").map_or(Ok(Ipv4Addr::UNSPECIFIED), |s| s.parse())?;
    let want_mac = args.get("mac").map(|s| parse_mac(s)).transpose()?;

    // Resolve the interface — by name if given, else by the host NIC IP.
    let iface = match args.get("iface") {
        Some(name) => Iface::from_system(name)?,
        None => Iface::from_ipv4(host_ip).map_err(|e| {
            anyhow::anyhow!("no interface with IP {host_ip} ({e}). Is the adapter set to {host_ip}? Pass --iface <name> or --host-ip <addr>.")
        })?,
    };
    let iface_ip = iface.ipv4().ok_or_else(|| anyhow::anyhow!("interface {} has no IPv4 address", iface.name()))?;
    println!("Using interface {} (ip {})", iface.name(), iface_ip);

    // Limited-broadcast discovery — reaches a camera on any subnet. A DHCP-first
    // camera (e.g. FLIR/Point Grey Blackfly) on a network with no DHCP server can
    // take up to ~60 s to fall back to a link-local 169.254.x.x address before it
    // answers, so optionally keep retrying for `--wait` seconds.
    let wait_secs: u64 = args.get("wait").map_or(Ok(0), |s| s.parse())?;
    println!("Discovering via 255.255.255.255 on {}…", iface.name());
    let found = discover_until(iface_ip, wait_secs).await?;
    if found.is_empty() {
        println!(
            "no cameras answered. If this is a DHCP-first camera with no DHCP server here, give it \
             time to fall back to a 169.254.x.x link-local address — re-run with `--wait 90` to poll \
             for up to 90 s. Otherwise check the link light, that {host_ip} is assigned to the \
             adapter, and that the camera has power. Re-run with --verbose for the raw trace."
        );
        return Ok(());
    }
    for d in &found {
        println!("  • {} @ {} (mac {})", describe(d), d.ip, fmt_mac(d.mac));
    }

    // Pick the target camera.
    let cam = match want_mac {
        Some(mac) => found.iter().find(|d| d.mac == mac).cloned().ok_or_else(|| {
            anyhow::anyhow!("no discovered camera with MAC {}", fmt_mac(mac))
        })?,
        None => found[0].clone(),
    };

    if args.contains_key("discover-only") {
        println!("\n--discover-only: not changing any IP. Re-run without it to force.");
        return Ok(());
    }

    println!(
        "\nForcing {} (mac {}) from {} → {}  mask {}  gw {}",
        describe(&cam), fmt_mac(cam.mac), cam.ip, target_ip, subnet, gateway
    );
    gvcp::force_ip(cam.mac, target_ip, subnet, gateway, Some(&iface)).await?;
    println!("FORCEIP acknowledged.");

    // Verify by re-discovering and checking the new address.
    tokio::time::sleep(Duration::from_millis(1500)).await;
    println!("\nRe-discovering to verify…");
    let after = discover_limited(iface_ip, Duration::from_millis(1500)).await?;
    match after.iter().find(|d| d.mac == cam.mac) {
        Some(d) if d.ip == target_ip => {
            println!("✓ camera now at {} — you can open it in the viewer.", d.ip);
        }
        Some(d) => println!("camera reports {} (expected {target_ip}); it may still be settling.", d.ip),
        None => println!("camera did not answer the verification scan; try the viewer's Refresh GigE."),
    }
    Ok(())
}

/// Run limited-broadcast discovery, retrying every ~2 s until a camera answers
/// or `wait_secs` elapses (0 = a single 1.5 s scan).
async fn discover_until(iface_ip: Ipv4Addr, wait_secs: u64) -> anyhow::Result<Vec<Found>> {
    let deadline = Instant::now() + Duration::from_secs(wait_secs);
    loop {
        let found = discover_limited(iface_ip, Duration::from_millis(1500)).await?;
        if !found.is_empty() || Instant::now() >= deadline {
            return Ok(found);
        }
        println!("  …no answer yet, retrying (waiting for camera, ~{}s left)", deadline.saturating_duration_since(Instant::now()).as_secs());
        tokio::time::sleep(Duration::from_millis(500)).await;
    }
}

/// Discover GigE cameras via the limited broadcast 255.255.255.255, which a
/// camera accepts regardless of which subnet its current IP is on. Hand-rolls
/// the GVCP discovery packet and parses the ACK (GigE Vision table 7-4).
async fn discover_limited(iface_ip: Ipv4Addr, timeout: Duration) -> anyhow::Result<Vec<Found>> {
    const GVCP_CMD_KEY: u8 = 0x42;
    const FLAG_ACK_REQUIRED: u8 = 0x01;
    const FLAG_BROADCAST: u8 = 0x10;
    let port = gvcp::GVCP_PORT;

    let socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(iface_ip), 0)).await?;
    socket.set_broadcast(true)?;

    // 8-byte GVCP DISCOVERY_CMD (0x0002), no payload, request id 1.
    let request_id: u16 = 1;
    let mut pkt = [0u8; 8];
    pkt[0] = GVCP_CMD_KEY;
    pkt[1] = FLAG_ACK_REQUIRED | FLAG_BROADCAST;
    pkt[2..4].copy_from_slice(&0x0002u16.to_be_bytes()); // DISCOVERY_CMD
    pkt[4..6].copy_from_slice(&0u16.to_be_bytes()); // length
    pkt[6..8].copy_from_slice(&request_id.to_be_bytes());

    socket
        .send_to(&pkt, SocketAddr::new(IpAddr::V4(Ipv4Addr::BROADCAST), port))
        .await?;
    tracing::info!(dest = "255.255.255.255", %port, "sent limited-broadcast discovery");

    let mut out: Vec<Found> = Vec::new();
    let mut buf = vec![0u8; 2048];
    let deadline = Instant::now() + timeout;
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        match tokio::time::timeout(remaining, socket.recv_from(&mut buf)).await {
            Ok(Ok((n, src))) => {
                tracing::debug!(%src, bytes = n, "discovery response");
                if let Some(f) = parse_discovery_ack(&buf[..n]) {
                    if !out.iter().any(|e| e.mac == f.mac) {
                        out.push(f);
                    }
                }
            }
            Ok(Err(e)) => return Err(e.into()),
            Err(_) => break, // timed out
        }
    }
    Ok(out)
}

/// Parse a GVCP DISCOVERY_ACK (8-byte header + payload). Returns None if the
/// packet isn't a well-formed discovery ack. Field offsets per GigE Vision 7-4.
fn parse_discovery_ack(buf: &[u8]) -> Option<Found> {
    if buf.len() < 8 + 40 {
        return None;
    }
    let command = u16::from_be_bytes([buf[2], buf[3]]);
    if command != 0x0003 {
        return None; // not DISCOVERY_ACK
    }
    let p = &buf[8..];
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&p[12..18]);
    let ip = Ipv4Addr::new(p[36], p[37], p[38], p[39]);
    let manufacturer = fixed_string(&p[72..(72 + 32).min(p.len())]);
    let model = if p.len() >= 104 + 32 { fixed_string(&p[104..136]) } else { String::new() };
    Some(Found { mac, ip, manufacturer, model })
}

/// Read a NUL-padded fixed-width ASCII field.
fn fixed_string(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

/// Parse args. Supports `--key value` pairs and bare `--flag` booleans.
fn parse_args() -> HashMap<String, String> {
    let mut map = HashMap::new();
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
        } else {
            i += 1;
        }
    }
    map
}

fn parse_mac(s: &str) -> anyhow::Result<[u8; 6]> {
    let parts: Vec<&str> = s.split([':', '-']).collect();
    anyhow::ensure!(parts.len() == 6, "MAC must be 6 octets, got '{s}'");
    let mut mac = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(p, 16)?;
    }
    Ok(mac)
}

fn fmt_mac(mac: [u8; 6]) -> String {
    mac.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

fn describe(d: &Found) -> String {
    match (d.manufacturer.is_empty(), d.model.is_empty()) {
        (false, false) => format!("{} {}", d.manufacturer, d.model),
        (_, false) => d.model.clone(),
        _ => "GigE camera".to_string(),
    }
}
