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
//!   cargo run --example gev_force_ip --features gev -- --ip 192.168.0.2 --persist
//!
//! The forced IP is temporary (does not survive a camera power-cycle) unless
//! --persist is given, which additionally writes the camera's persistent-IP
//! registers so it boots at the target address from then on. --persist requires
//! the target IP to be on this host's subnet (we must open the camera to write
//! the registers).

use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use tokio::net::UdpSocket;
use viva_gige::gvcp::{self, consts, GigeDevice};
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

    // If a MAC is given explicitly, FORCEIP it directly — no discovery needed
    // (FORCEIP is a MAC-targeted broadcast; the camera need not be reachable yet).
    // Otherwise, limited-broadcast discovery finds it.
    let cam = if let Some(mac) = want_mac {
        println!("Targeting camera by MAC {} directly (skipping discovery).", fmt_mac(mac));
        Found { mac, ip: Ipv4Addr::UNSPECIFIED, manufacturer: String::new(), model: String::new() }
    } else {
        let wait_secs: u64 = args.get("wait").map_or(Ok(0), |s| s.parse())?;
        println!("Discovering via 255.255.255.255 on {}…", iface.name());
        let found = discover_until(iface_ip, wait_secs).await?;
        if found.is_empty() {
            println!(
                "no cameras answered. If this is a DHCP-first camera with no DHCP server here, give it \
                 time to fall back to a 169.254.x.x link-local address — re-run with `--wait 90` to poll \
                 for up to 90 s, or pass --mac <addr> to force it directly. Re-run with --verbose for the raw trace."
            );
            return Ok(());
        }
        for d in &found {
            println!("  • {} @ {} (mac {})", describe(d), d.ip, fmt_mac(d.mac));
        }
        found[0].clone()
    };

    if args.contains_key("discover-only") {
        println!("\n--discover-only: not changing any IP. Re-run without it to force.");
        return Ok(());
    }

    println!(
        "\nForcing {} (mac {}) from {} → {}  mask {}  gw {}",
        describe(&cam), fmt_mac(cam.mac), cam.ip, target_ip, subnet, gateway
    );
    match gvcp::force_ip(cam.mac, target_ip, subnet, gateway, Some(&iface)).await {
        Ok(()) => println!("FORCEIP acknowledged."),
        Err(e) => println!("FORCEIP not acknowledged ({e}); some cameras apply it silently — continuing."),
    }

    // If the new IP is on this host's subnet we can verify (and persist);
    // otherwise (the usual cross-subnet case) the host must move there first.
    let same_subnet = on_same_24(iface_ip, target_ip);
    if args.contains_key("persist") {
        anyhow::ensure!(
            same_subnet,
            "--persist must open the camera at {target_ip}, which is unreachable from this host \
             ({iface_ip}); move the host onto the target subnet first"
        );
        return persist_ip(target_ip, subnet, gateway).await;
    }
    if same_subnet {
        tokio::time::sleep(Duration::from_millis(1500)).await;
        println!("\nVerifying…");
        let after = discover_limited(iface_ip, Duration::from_millis(1500)).await?;
        match after.iter().find(|d| d.mac == cam.mac) {
            Some(d) if d.ip == target_ip => println!("✓ camera now at {}.", d.ip),
            Some(d) => println!("camera reports {} (expected {target_ip}); may still be settling.", d.ip),
            None => println!("camera didn't answer the verify scan; try pinging {target_ip}."),
        }
        println!("This address is temporary (lost on power-cycle); re-run with --persist to make it stick.");
    } else {
        println!(
            "\nNew IP {target_ip} is on a different subnet than this host ({iface_ip}), so it can't be \
             verified from here. Now change this computer's IP onto the {} subnet, then `ping {target_ip}`.",
            subnet_label(target_ip, subnet)
        );
    }
    Ok(())
}

/// Open the camera at its (just-forced) address, write the persistent-IP
/// registers, and enable persistent-IP mode so the address survives
/// power-cycles. Retries the open for a while — the camera may still be
/// applying the FORCEIP.
async fn persist_ip(ip: Ipv4Addr, subnet: Ipv4Addr, gateway: Ipv4Addr) -> anyhow::Result<()> {
    println!("\nOpening {ip} to write persistent-IP registers…");
    let addr = SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT);
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut dev = loop {
        match open_and_claim(addr).await {
            Ok(dev) => break dev,
            Err(e) if Instant::now() < deadline => {
                println!("  …not reachable yet ({e}), retrying");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
            Err(e) => anyhow::bail!("camera at {ip} never became reachable: {e}"),
        }
    };

    dev.write_persistent_ip(ip, subnet, gateway).await?;

    // Enable persistent-IP in the interface-configuration register. The GigE
    // Vision spec puts PersistentIP at bit 0 and DHCP at bit 1, but some
    // implementations document the reverse — set both; at boot, persistent IP
    // takes precedence over DHCP either way.
    let cfg = dev.read_register(consts::CURRENT_IP_CONFIG as u32).await?;
    dev.write_register(consts::CURRENT_IP_CONFIG as u32, cfg | 0x3).await?;

    let (pip, psub, pgw) = dev.read_persistent_ip().await?;
    let cfg_after = dev.read_register(consts::CURRENT_IP_CONFIG as u32).await?;
    let _ = dev.release_control().await;
    println!("✓ persistent IP {pip}  mask {psub}  gw {pgw}  (IP-config register now {cfg_after:#06x})");
    anyhow::ensure!(pip == ip, "read-back persistent IP {pip} != requested {ip}");
    println!("The camera will boot at {pip} from its next power-cycle on.");
    Ok(())
}

async fn open_and_claim(addr: SocketAddr) -> anyhow::Result<GigeDevice> {
    let mut dev = GigeDevice::open(addr).await?;
    dev.claim_control().await?;
    Ok(dev)
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
    // Device-info layout (GigE Vision, bootstrap registers mirrored into the ack):
    // p[8..10] reserved, p[10..16] MAC (2-byte MAC-high at 0x000E + 4-byte MAC-low
    // at 0x0010). The MAC starts at p[10], not p[12].
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&p[10..16]);
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

fn on_same_24(a: Ipv4Addr, b: Ipv4Addr) -> bool {
    a.octets()[..3] == b.octets()[..3]
}

fn subnet_label(ip: Ipv4Addr, mask: Ipv4Addr) -> String {
    let net: Vec<u8> = ip.octets().iter().zip(mask.octets()).map(|(i, m)| i & m).collect();
    let bits = mask.octets().iter().map(|b| b.count_ones()).sum::<u32>();
    format!("{}.{}.{}.{}/{}", net[0], net[1], net[2], net[3], bits)
}

fn describe(d: &Found) -> String {
    match (d.manufacturer.is_empty(), d.model.is_empty()) {
        (false, false) => format!("{} {}", d.manufacturer, d.model),
        (_, false) => d.model.clone(),
        _ => "GigE camera".to_string(),
    }
}
