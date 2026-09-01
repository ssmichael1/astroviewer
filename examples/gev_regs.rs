//! Read GigE Vision bootstrap registers without claiming control, so it can
//! run alongside a viewer that holds the camera. Shows what the camera thinks
//! the stream channel is configured to.
//!
//!   cargo run --example gev_regs --features gev -- 192.168.0.2

use std::net::{IpAddr, Ipv4Addr, SocketAddr};

use viva_gige::gvcp::{self, GigeDevice};

fn main() -> anyhow::Result<()> {
    let ip: Ipv4Addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.0.2".into()).parse()?;
    let rt = tokio::runtime::Builder::new_current_thread().enable_all().build()?;
    let mut dev = rt.block_on(GigeDevice::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT)))?;
    let regs: &[(&str, u32)] = &[
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
        match rt.block_on(dev.read_register(*addr)) {
            Ok(v) => {
                let extra = match *addr {
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
