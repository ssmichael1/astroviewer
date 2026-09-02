//! GigE Vision transport spike — proves discovery, control, GenICam XML fetch,
//! and GVSP streaming against a real camera, with no changes to the viewer app.
//! Uses the app-owned synchronous GigE transport.
//!
//!   cargo run --example gev_spike --features gev
//!   cargo run --example gev_spike --features gev -- 192.168.0.2
//!   cargo run --example gev_spike --features gev -- 127.0.0.1 3957
//!
//! With no camera present it prints "no cameras found" and exits cleanly.

#[path = "../src/gige/mod.rs"]
#[allow(dead_code)]
mod gige;

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::{Duration, Instant};

use gige::gvcp::{self, Device};
use gige::gvsp::{self, GvspPacket};
use gige::nic::{self, Iface};

fn main() -> anyhow::Result<()> {
    // Optional first arg: a target IP to open directly (skips broadcast
    // discovery). Optional second arg: the GVCP port (default 3956).
    let target_ip: Option<Ipv4Addr> = std::env::args().nth(1).and_then(|s| s.parse().ok());
    let port: u16 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(gvcp::GVCP_PORT);

    let ip = match target_ip {
        Some(ip) => {
            println!("Opening {ip} directly (skipping discovery)…");
            ip
        }
        None => {
            println!("Discovering GigE Vision cameras (500 ms)…");
            let devices = gvcp::discover_all(Duration::from_millis(500));
            if devices.is_empty() {
                println!("no cameras found (pass an IP to open directly, e.g. `… -- 192.168.0.2`)");
                return Ok(());
            }
            for d in &devices {
                println!(
                    "  • {} {} @ {} (mac {:02x?})",
                    d.manufacturer.as_deref().unwrap_or("?"),
                    d.model.as_deref().unwrap_or("?"),
                    d.ip,
                    d.mac
                );
            }
            devices[0].ip
        }
    };

    println!("\nOpening {ip}:{port}…");
    let mut dev = Device::open(SocketAddr::new(IpAddr::V4(ip), port)).map_err(|e| anyhow::anyhow!("open: {e}"))?;
    dev.claim_control().map_err(|e| anyhow::anyhow!("claim control: {e}"))?;
    println!("control claimed");

    // First-URL bootstrap register → GenICam XML location.
    let raw = dev.read_mem(0x0200, 512).map_err(|e| anyhow::anyhow!("READMEM: {e}"))?;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let url = String::from_utf8_lossy(&raw[..end]);
    println!("GenICam URL: {}", url.trim());

    // Negotiate a stream channel toward our interface and bind a receive socket.
    let iface = Iface::from_ipv4(nic::local_ipv4_towards(ip, port))?;
    let bind_ip = iface.ipv4().map(IpAddr::V4).unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));
    let (socket, _rcvbuf) = nic::bind_gvsp_socket(bind_ip, 32 << 20, Duration::from_millis(500))?;
    let local_port = socket.local_addr()?.port();
    let params = dev.negotiate_stream(0, &iface, local_port, None).map_err(|e| anyhow::anyhow!("negotiate: {e}"))?;
    println!(
        "stream negotiated: mtu={} packet_size={} -> receiving on {}:{}",
        params.mtu, params.packet_size, bind_ip, local_port
    );

    // Receive a handful of packets and report the first Leader's geometry.
    let mut buf = nic::RecvBuf::new();
    let mut leaders = 0;
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && leaders < 3 {
        match buf.next_datagram(&socket) {
            Ok((pkt, _)) => {
                if let Ok(GvspPacket::Leader { width, height, pixel_format, block_id }) =
                    gvsp::parse_packet(pkt)
                {
                    println!("frame {block_id}: {width}x{height} pixel_format=0x{pixel_format:08x}");
                    leaders += 1;
                }
            }
            Err(e) => match e.kind() {
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut => {
                    println!("(no packets — is acquisition running? try AcquisitionStart)");
                }
                std::io::ErrorKind::ConnectionReset | std::io::ErrorKind::Interrupted => continue,
                _ => {
                    println!("recv error: {e}");
                    break;
                }
            },
        }
    }

    let _ = dev.release_control();
    println!("done");
    Ok(())
}
