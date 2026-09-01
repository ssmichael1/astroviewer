//! Give a GigE Vision camera an IP address on this host's subnet.
//!
//! Discovers cameras by broadcast (or targets one by MAC), sends FORCEIP for
//! a temporary address, verifies it answers there, and with `--persist` opens
//! the camera at the new address and writes the persistent-IP registers so
//! it boots there from then on. Uses the app-owned synchronous GigE transport.
//!
//!   cargo run --example gev_force_ip --features gev -- --discover-only
//!   cargo run --example gev_force_ip --features gev -- --ip 192.168.0.10
//!   cargo run --example gev_force_ip --features gev -- --mac 00:11:22:33:44:55 --ip 192.168.0.10 --persist
//!
//! Options: --host-ip <addr> (this host's address on the camera link, default
//! 192.168.0.1) --ip <addr> (target, default 192.168.0.10) --subnet <mask>
//! (default 255.255.255.0) --gateway <addr> (default 0.0.0.0) --mac <addr>
//! (skip discovery) --from <addr> (identify the camera by unicast at its
//! current address, for hosts that drop broadcast replies) --wait <secs>
//! (keep discovering that long) --discover-only --persist

#[path = "../src/gige/mod.rs"]
#[allow(dead_code)]
mod gige;

use std::collections::HashMap;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use gige::gvcp::{self, Device, DeviceInfo};

fn main() -> anyhow::Result<()> {
    let args = parse_args();
    let host_ip: Ipv4Addr = args.get("host-ip").map_or(Ok(Ipv4Addr::new(192, 168, 0, 1)), |s| s.parse())?;
    let target_ip: Ipv4Addr = args.get("ip").map_or(Ok(Ipv4Addr::new(192, 168, 0, 10)), |s| s.parse())?;
    let subnet: Ipv4Addr = args.get("subnet").map_or(Ok(Ipv4Addr::new(255, 255, 255, 0)), |s| s.parse())?;
    let gateway: Ipv4Addr = args.get("gateway").map_or(Ok(Ipv4Addr::UNSPECIFIED), |s| s.parse())?;
    let want_mac = args.get("mac").map(|s| parse_mac(s)).transpose()?;
    let wait_secs: u64 = args.get("wait").map_or(Ok(0), |s| s.parse())?;
    let port: u16 = args.get("port").map_or(Ok(gvcp::GVCP_PORT), |s| s.parse())?;

    let from: Option<Ipv4Addr> = args.get("from").map(|s| s.parse()).transpose()?;
    let cam = if let Some(mac) = want_mac {
        println!("Targeting camera by MAC {} directly (skipping discovery).", fmt_mac(mac));
        DeviceInfo { mac, ip: Ipv4Addr::UNSPECIFIED, manufacturer: None, model: None }
    } else if let Some(from) = from {
        println!("Asking {from} to identify itself (unicast discovery)…");
        let Some(d) = gvcp::discover_unicast(from, port, Duration::from_millis(1000)) else {
            println!("no answer from {from}; check the address, or pass --mac <addr>.");
            return Ok(());
        };
        println!("  • {} @ {} (mac {})", describe(&d), d.ip, fmt_mac(d.mac));
        d
    } else {
        println!("Discovering by broadcast on every interface…");
        let found = discover_until(wait_secs);
        if found.is_empty() {
            println!(
                "no cameras answered. If this is a DHCP-first camera with no DHCP server here, give it \
                 time to fall back to a 169.254.x.x link-local address — re-run with `--wait 90` to poll \
                 for up to 90 s, or pass --mac <addr> to force it directly."
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
    match gvcp::force_ip(cam.mac, target_ip, subnet, gateway, Duration::from_millis(1500)) {
        Ok(true) => println!("FORCEIP acknowledged."),
        Ok(false) => println!("FORCEIP not acknowledged; many cameras apply it silently — continuing."),
        Err(e) => println!("FORCEIP failed ({e}); continuing to verify anyway."),
    }

    let same_subnet = (u32::from(host_ip) & u32::from(subnet)) == (u32::from(target_ip) & u32::from(subnet));
    if args.contains_key("persist") {
        anyhow::ensure!(
            same_subnet,
            "--persist must open the camera at {target_ip}, which is unreachable from this host \
             ({host_ip}); move the host onto the target subnet first"
        );
        return persist_ip(target_ip, port, subnet, gateway);
    }

    if same_subnet {
        std::thread::sleep(Duration::from_millis(1500));
        println!("\nVerifying…");
        let after = gvcp::discover_all(Duration::from_millis(1500));
        match after.iter().find(|d| d.mac == cam.mac) {
            Some(d) if d.ip == target_ip => println!("✓ camera now at {}.", d.ip),
            Some(d) => println!("camera reports {} (expected {target_ip}); may still be settling.", d.ip),
            None => println!("camera didn't answer the verify scan; try pinging {target_ip}."),
        }
        println!("This address is temporary (lost on power-cycle); re-run with --persist to make it stick.");
    } else {
        println!(
            "\nNew IP {target_ip} is on a different subnet than this host ({host_ip}), so it can't be \
             verified from here. Move this computer onto that subnet, then `ping {target_ip}`."
        );
    }
    Ok(())
}

/// Open the camera at its new address (retrying while it settles), write the
/// persistent-IP registers and set the persistent + LLA configuration bits.
fn persist_ip(ip: Ipv4Addr, port: u16, subnet: Ipv4Addr, gateway: Ipv4Addr) -> anyhow::Result<()> {
    println!("\nOpening {ip} to write persistent-IP registers…");
    let addr = SocketAddr::new(IpAddr::V4(ip), port);
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut dev = loop {
        match open_and_claim(addr) {
            Ok(dev) => break dev,
            Err(e) if Instant::now() < deadline => {
                println!("  …not reachable yet ({e}), retrying");
                std::thread::sleep(Duration::from_secs(1));
            }
            Err(e) => anyhow::bail!("camera at {ip} never became reachable: {e}"),
        }
    };
    dev.write_persistent_ip(ip, subnet, gateway)?;
    let cfg = dev.ip_config()?;
    dev.set_ip_config(cfg | gvcp::IP_CONFIG_PERSISTENT | gvcp::IP_CONFIG_LLA)?;
    let (pip, psub, pgw) = dev.read_persistent_ip()?;
    let cfg_after = dev.ip_config()?;
    let _ = dev.release_control();
    println!("✓ persistent IP {pip}  mask {psub}  gw {pgw}  (IP-config register now {cfg_after:#06x})");
    anyhow::ensure!(pip == ip, "read-back persistent IP {pip} != requested {ip}");
    println!("The camera will boot at {pip} from its next power-cycle on.");
    Ok(())
}

fn open_and_claim(addr: SocketAddr) -> anyhow::Result<Device> {
    let mut dev = Device::open(addr)?;
    dev.claim_control()?;
    Ok(dev)
}

/// Discover repeatedly for up to `wait_secs` (at least one pass), returning as
/// soon as anything answers.
fn discover_until(wait_secs: u64) -> Vec<DeviceInfo> {
    let deadline = Instant::now() + Duration::from_secs(wait_secs);
    loop {
        let found = gvcp::discover_all(Duration::from_millis(1000));
        if !found.is_empty() || Instant::now() >= deadline {
            return found;
        }
        println!("  …nothing yet, still listening");
    }
}

fn describe(d: &DeviceInfo) -> String {
    match (&d.manufacturer, &d.model) {
        (Some(m), Some(n)) => format!("{m} {n}"),
        (_, Some(n)) => n.clone(),
        _ => "camera".to_string(),
    }
}

fn fmt_mac(mac: [u8; 6]) -> String {
    mac.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

fn parse_mac(s: &str) -> anyhow::Result<[u8; 6]> {
    let parts: Vec<&str> = s.split([':', '-']).collect();
    anyhow::ensure!(parts.len() == 6, "MAC must have six octets, e.g. 00:11:22:33:44:55");
    let mut mac = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        mac[i] = u8::from_str_radix(p, 16).map_err(|_| anyhow::anyhow!("bad MAC octet {p:?}"))?;
    }
    Ok(mac)
}

/// `--flag` → ("flag", "true"); `--key value` → ("key", "value").
fn parse_args() -> HashMap<String, String> {
    let mut out = HashMap::new();
    let mut it = std::env::args().skip(1).peekable();
    while let Some(a) = it.next() {
        let Some(key) = a.strip_prefix("--") else { continue };
        let value = match it.peek() {
            Some(v) if !v.starts_with("--") => it.next().unwrap(),
            _ => "true".to_string(),
        };
        out.insert(key.to_string(), value);
    }
    out
}
