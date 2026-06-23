//! GigE Vision transport spike — proves discovery, control, GenICam XML fetch,
//! and GVSP streaming against a real camera, with no changes to the viewer app.
//!
//! Run with a camera on the network:
//!     cargo run --example gev_spike --features gev
//!
//! With no camera present it prints "no cameras found" and exits cleanly.

use std::net::{IpAddr, SocketAddr};
use std::time::Duration;

use viva_gige::gvcp::{self, GigeDevice};
use viva_gige::gvsp::{self, GvspPacket};
use viva_gige::nic::{self, Iface};

#[tokio::main(flavor = "current_thread")]
async fn main() -> anyhow::Result<()> {
    // Optional first arg: a target IP to open directly (skips broadcast discovery,
    // useful when the camera is reachable by unicast but doesn't answer broadcasts).
    let target_ip: Option<std::net::Ipv4Addr> = std::env::args().nth(1).and_then(|s| s.parse().ok());

    let ip = match target_ip {
        Some(ip) => {
            println!("Opening {ip} directly (skipping discovery)…");
            ip
        }
        None => {
            println!("Discovering GigE Vision cameras (500 ms)…");
            let devices = gvcp::discover_all(Duration::from_millis(500)).await?;
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

    println!("\nOpening {ip}…");
    let mut dev = GigeDevice::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT)).await?;
    dev.claim_control().await?;
    println!("control claimed");

    // First-URL bootstrap register → GenICam XML location.
    let raw = dev.read_mem(0x0200, 512).await?;
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len()); // cut at NUL; tail may be garbage
    let url = String::from_utf8_lossy(&raw[..end]);
    let url = url.trim();
    println!("GenICam URL: {url}");

    // Negotiate a stream channel toward our interface and bind a receive socket.
    let iface = Iface::from_ipv4(local_ipv4_towards(ip)?)?;
    let bind_ip = iface.ipv4().map(IpAddr::V4).unwrap_or(nic::default_bind_addr());
    let socket = nic::bind_udp(bind_ip, 0, Some(iface.clone()), None).await?;
    let port = socket.local_addr()?.port();
    let params = dev.negotiate_stream(0, &iface, port, None).await?;
    println!(
        "stream negotiated: mtu={} packet_size={} -> receiving on {}:{}",
        params.mtu, params.packet_size, bind_ip, port
    );

    // Receive a handful of packets and report the first Leader's geometry.
    let mut buf = vec![0u8; 65536];
    let mut leaders = 0;
    let deadline = tokio::time::Instant::now() + Duration::from_secs(3);
    while tokio::time::Instant::now() < deadline && leaders < 3 {
        match tokio::time::timeout(Duration::from_millis(500), socket.recv_from(&mut buf)).await {
            Ok(Ok((n, _))) => {
                if let Ok(GvspPacket::Leader { width, height, pixel_format, block_id, .. }) =
                    gvsp::parse_packet(&buf[..n])
                {
                    println!(
                        "frame {block_id}: {width}x{height} pixel_format=0x{pixel_format:08x}"
                    );
                    leaders += 1;
                }
            }
            Ok(Err(e)) => {
                println!("recv error: {e}");
                break;
            }
            Err(_) => println!("(no packets — is acquisition running? try AcquisitionStart)"),
        }
    }

    dev.release_control().await?;
    println!("done");
    Ok(())
}

fn local_ipv4_towards(target: std::net::Ipv4Addr) -> anyhow::Result<std::net::Ipv4Addr> {
    use std::net::UdpSocket;
    let sock = UdpSocket::bind((std::net::Ipv4Addr::UNSPECIFIED, 0))?;
    sock.connect((target, gvcp::GVCP_PORT))?;
    match sock.local_addr()? {
        SocketAddr::V4(a) => Ok(*a.ip()),
        _ => Ok(std::net::Ipv4Addr::UNSPECIFIED),
    }
}
