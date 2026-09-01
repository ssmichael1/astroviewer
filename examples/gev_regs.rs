//! Read GigE Vision bootstrap registers without claiming control, so it can
//! run alongside a viewer that holds the camera. Shows what the camera thinks
//! the stream channel is configured to, and what it advertises in
//! GevCapability. Uses the app-owned synchronous GigE transport.
//!
//!   cargo run --example gev_regs --features gev -- 192.168.0.2 [gvcp_port]

#[path = "../src/gige/mod.rs"]
#[allow(dead_code)]
mod gige;

use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use gige::gvcp::{self, Device};

fn main() -> anyhow::Result<()> {
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let port: u16 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(gvcp::GVCP_PORT);
    let mut dev = Device::open(SocketAddr::new(IpAddr::V4(ip), port))?;
    let regs: &[(&str, u32)] = &[
        ("GevCapability", 0x0934),
        ("CCP (control privilege)", 0x0A00),
        ("HeartbeatTimeout ms", 0x0938),
        ("CurrentIPAddress", 0x0024),
        ("CurrentSubnetMask", 0x0034),
        ("NumberOfStreamChannels", 0x0904),
        ("SCP0 HostPort", 0x0D00),
        ("SCPS0 PacketSize", 0x0D04),
        ("SCPD0 PacketDelay", 0x0D08),
        ("SCDA0 DestAddress", 0x0D18),
        ("SCSP0 SourcePort", 0x0D1C),
        ("SCC0 Capabilities", 0x0D20),
        ("SCCFG0 Config", 0x0D24),
    ];
    for (name, addr) in regs {
        match dev.read_register(*addr) {
            Ok(v) => {
                let extra = match *addr {
                    0x0934 => format!(
                        "  PR(packet resend)={} W(writemem)={} C(concat)={} PA(pending ack)={} E(event)={}",
                        (v >> 2) & 1, (v >> 1) & 1, v & 1, (v >> 5) & 1, (v >> 3) & 1
                    ),
                    0x0024 | 0x0034 | 0x0D18 => format!("  = {}", Ipv4Addr::from(v)),
                    0x0D00 | 0x0D1C => format!("  = port {}", v & 0xFFFF),
                    0x0D04 => format!("  = size {} (flags {:#x})", v & 0xFFFF, v >> 16),
                    _ => String::new(),
                };
                println!("{name:<26} @{addr:#06x} = {v:#010x}{extra}");
            }
            Err(e) => println!("{name:<26} @{addr:#06x} read error: {e}"),
        }
    }
    Ok(())
}
