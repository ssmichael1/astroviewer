//! Network-interface helpers for the app-owned GigE Vision transport.
//!
//! Wire layouts, constants and the MTU/packet-size accounting are derived from
//! the MIT-licensed `viva-gige` crate (`nic.rs`,
//! <https://github.com/VitalyVorobyev/viva-genicam>). This is a synchronous,
//! std-socket reimplementation owned by the app.

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};

use if_addrs::{get_if_addrs, IfAddr};
use socket2::{Domain, Protocol, Socket, Type};

/// Largest packet an IPv4 datagram can carry. The IPv4 total-length field is 16
/// bits, so no datagram exceeds this regardless of the link MTU. Loopback
/// routinely reports more (Linux `lo` 65536, macOS `lo0` 16384), and an
/// unclamped value is unsendable — a `65536 - 36` payload overflows the
/// 65507-byte UDP maximum. Mirrors viva-gige's `MAX_IPV4_PACKET_SIZE`.
const MAX_IPV4_PACKET_SIZE: u32 = 65535;

/// A host network interface usable for GigE streaming.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Iface {
    name: String,
    ipv4: Option<Ipv4Addr>,
    netmask: Ipv4Addr,
    #[allow(dead_code)]
    index: u32,
}

impl Iface {
    /// Resolve the interface that owns `addr`, preserving the address asked for
    /// (a multi-homed NIC has several; keep the one the caller selected).
    pub fn from_ipv4(addr: Ipv4Addr) -> io::Result<Self> {
        for iface in get_if_addrs()? {
            if let IfAddr::V4(v4) = &iface.addr {
                if v4.ip == addr {
                    return Ok(Self {
                        name: iface.name.clone(),
                        ipv4: Some(addr),
                        netmask: v4.netmask,
                        index: iface.index.unwrap_or(0),
                    });
                }
            }
        }
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("no interface with IPv4 {addr}"),
        ))
    }

    /// Primary IPv4 address bound to the interface, if any.
    pub fn ipv4(&self) -> Option<Ipv4Addr> {
        self.ipv4
    }

    /// Interface name as the OS reports it (e.g. `en0`). Used by [`mtu`] on
    /// Linux; on platforms whose MTU probe does not need it, it is dead.
    #[allow(dead_code)]
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// One IPv4 interface as seen during discovery.
pub struct Ipv4Iface {
    pub ip: Ipv4Addr,
    pub netmask: Ipv4Addr,
    pub is_loopback: bool,
    /// OS interface name (`en21`, `eth0`, a GUID on Windows).
    pub name: String,
    /// Kernel interface index, when the OS reports one.
    pub index: Option<u32>,
}

/// Enumerate every IPv4 interface the host reports.
pub fn ipv4_interfaces() -> Vec<Ipv4Iface> {
    let mut out = Vec::new();
    if let Ok(ifaces) = get_if_addrs() {
        for iface in ifaces {
            if let IfAddr::V4(v4) = iface.addr {
                out.push(Ipv4Iface {
                    ip: v4.ip,
                    netmask: v4.netmask,
                    is_loopback: v4.ip.is_loopback(),
                    name: iface.name.clone(),
                    index: iface.index,
                });
            }
        }
    }
    out
}

/// Directed-broadcast address for an interface, computed from the netmask
/// (the reported broadcast field is unreliable for manually configured APIPA
/// addresses — viva-gige `directed_broadcast`).
pub fn directed_broadcast(ip: Ipv4Addr, netmask: Ipv4Addr) -> Ipv4Addr {
    Ipv4Addr::from(u32::from(ip) | !u32::from(netmask))
}

/// Best-effort local IPv4 on the same route as `target`, found by connecting a
/// UDP socket toward it (no packets are sent). Falls back to `0.0.0.0`.
pub fn local_ipv4_towards(target: Ipv4Addr, port: u16) -> Ipv4Addr {
    if let Ok(sock) = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)) {
        if sock.connect((target, port)).is_ok() {
            if let Ok(SocketAddr::V4(local)) = sock.local_addr() {
                return *local.ip();
            }
        }
    }
    Ipv4Addr::UNSPECIFIED
}

/// Read the link MTU for `iface`.
///
/// Linux reads `/sys/class/net/<if>/mtu`; Windows uses `GetIfEntry2`; every
/// other platform (macOS included) defaults to the canonical Ethernet MTU.
/// Mirrors viva-gige `nic::mtu`.
pub fn mtu(iface: &Iface) -> u32 {
    #[cfg(target_os = "linux")]
    {
        let path = format!("/sys/class/net/{}/mtu", iface.name());
        if let Ok(contents) = std::fs::read_to_string(path) {
            if let Ok(mtu) = contents.trim().parse::<u32>() {
                return mtu;
            }
        }
    }

    #[cfg(windows)]
    {
        use windows_sys::Win32::NetworkManagement::IpHelper::{GetIfEntry2, MIB_IF_ROW2};
        // SAFETY: MIB_IF_ROW2 is a plain-old-data struct; we zero it, set the
        // interface index, and let the kernel fill the rest. GetIfEntry2 reads
        // and writes only this struct.
        let mut row: MIB_IF_ROW2 = unsafe { std::mem::zeroed() };
        row.InterfaceIndex = iface.index;
        let status = unsafe { GetIfEntry2(&mut row) };
        if status == 0 && row.Mtu > 0 {
            return row.Mtu;
        }
    }

    #[cfg(not(any(target_os = "linux", windows)))]
    {
        let _ = iface;
    }

    1500
}

/// GVSP packet size derived from the link MTU. `GevSCPSPacketSize` is the full
/// transmitted IP datagram, so it tracks the MTU directly (aravis corroborates:
/// its payload stride is `packet_size - (IP + UDP + GVSP)`). Clamped to the
/// IPv4 maximum. Mirrors viva-gige `best_packet_size`.
pub fn best_packet_size(mtu: u32) -> u32 {
    mtu.min(MAX_IPV4_PACKET_SIZE)
}

/// Bind a blocking UDP socket tuned for GVSP: large SO_RCVBUF, address reuse, a
/// read timeout so the capture loop can service commands between frames.
/// Returns the socket and the receive-buffer size the OS actually granted.
pub fn bind_gvsp_socket(
    bind: IpAddr,
    rcvbuf: usize,
    read_timeout: std::time::Duration,
) -> io::Result<(UdpSocket, usize)> {
    let domain = match bind {
        IpAddr::V4(_) => Domain::IPV4,
        IpAddr::V6(_) => Domain::IPV6,
    };
    let socket = Socket::new(domain, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    #[cfg(all(unix, not(target_os = "solaris")))]
    let _ = socket.set_reuse_port(true);
    // Best-effort: a rejected buffer request leaves the default in place.
    let _ = socket.set_recv_buffer_size(rcvbuf);
    let actual = socket.recv_buffer_size().unwrap_or(0);
    socket.bind(&SocketAddr::new(bind, 0).into())?;
    let socket: UdpSocket = socket.into();
    socket.set_nonblocking(false)?;
    socket.set_read_timeout(Some(read_timeout))?;
    Ok((socket, actual))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_size_tracks_and_clamps_the_mtu() {
        assert_eq!(best_packet_size(1500), 1500);
        assert_eq!(best_packet_size(9000), 9000);
        // Linux loopback reports 65536; unclamped this is unsendable.
        assert_eq!(best_packet_size(65536), MAX_IPV4_PACKET_SIZE);
        assert_eq!(best_packet_size(u32::MAX), MAX_IPV4_PACKET_SIZE);
    }

    #[test]
    fn directed_broadcast_uses_the_netmask() {
        assert_eq!(
            directed_broadcast(Ipv4Addr::new(169, 254, 1, 10), Ipv4Addr::new(255, 255, 0, 0)),
            Ipv4Addr::new(169, 254, 255, 255)
        );
        assert_eq!(
            directed_broadcast(Ipv4Addr::new(192, 168, 0, 5), Ipv4Addr::new(255, 255, 255, 0)),
            Ipv4Addr::new(192, 168, 0, 255)
        );
    }
}
