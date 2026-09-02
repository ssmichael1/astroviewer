//! Network-interface helpers for the app-owned GigE Vision transport.
//!
//! Wire layouts, constants and the MTU/packet-size accounting are derived from
//! the MIT-licensed `viva-gige` crate (`nic.rs`,
//! <https://github.com/VitalyVorobyev/viva-genicam>). This is a synchronous,
//! std-socket reimplementation owned by the app.

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::time::Duration;

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

/// Largest UDP payload an IPv4 datagram can carry (65535 minus the IP and UDP
/// headers): the coalescing ceiling requested on Windows.
#[cfg(windows)]
const MAX_UDP_PAYLOAD: u32 = 65_507;

/// Bind a blocking UDP socket tuned for GVSP: large SO_RCVBUF, address reuse, a
/// read timeout so the capture loop can service commands between frames. On
/// Windows the socket also asks for UDP receive coalescing (refused on
/// Windows 10 and older; `GEV_COALESCE=0` skips it) and turns off ICMP
/// port-unreachable reporting — see `winsock`. Returns the socket and the
/// receive-buffer size the OS actually granted.
pub fn bind_gvsp_socket(
    bind: IpAddr,
    rcvbuf: usize,
    read_timeout: Duration,
) -> io::Result<(GvspSocket, usize)> {
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
    // Coalescing is requested before bind, the order msquic uses.
    #[cfg(windows)]
    let (coalesce, coalesce_status) = if std::env::var("GEV_COALESCE").is_ok_and(|v| v.trim() == "0") {
        (None, "off (GEV_COALESCE=0)".to_string())
    } else {
        match super::winsock::Coalescer::enable(&socket, MAX_UDP_PAYLOAD) {
            Ok(c) => (Some(c), "on".to_string()),
            Err(e) => (None, format!("off (not available: {e})")),
        }
    };
    #[cfg(not(windows))]
    let coalesce_status = "off (not supported on this OS)".to_string();
    socket.bind(&SocketAddr::new(bind, 0).into())?;
    let socket: UdpSocket = socket.into();
    socket.set_nonblocking(false)?;
    socket.set_read_timeout(Some(read_timeout))?;
    #[cfg(windows)]
    let _ = super::winsock::disable_connreset(&socket);
    Ok((
        GvspSocket {
            inner: socket,
            #[cfg(windows)]
            coalesce,
            coalesce_status,
        },
        actual,
    ))
}

/// One receive from a [`GvspSocket`]: `len` bytes in the caller's buffer from
/// `src`. When the OS coalesced consecutive same-size datagrams (Windows 11)
/// they are laid out `segment` bytes apart, the last possibly shorter;
/// otherwise a single datagram with `segment == len`.
#[derive(Debug, Clone, Copy)]
pub struct Received {
    pub len: usize,
    pub segment: usize,
    pub src: SocketAddr,
}

/// The GVSP receive socket: a blocking UDP socket plus, on Windows, the
/// receive-coalescing state that lets one receive return dozens of packets.
/// Read it through a [`RecvBuf`], which hands a coalesced receive out one
/// datagram at a time.
pub struct GvspSocket {
    inner: UdpSocket,
    #[cfg(windows)]
    coalesce: Option<super::winsock::Coalescer>,
    coalesce_status: String,
}

impl GvspSocket {
    /// Wrap a plain socket: one datagram per receive (tests, other callers).
    #[allow(dead_code)]
    pub fn from_udp(inner: UdpSocket) -> Self {
        Self {
            inner,
            #[cfg(windows)]
            coalesce: None,
            coalesce_status: "off".to_string(),
        }
    }

    /// Whether a receive may carry several datagrams.
    #[allow(dead_code)]
    pub fn coalescing(&self) -> bool {
        #[cfg(windows)]
        {
            self.coalesce.is_some()
        }
        #[cfg(not(windows))]
        {
            false
        }
    }

    /// Coalescing state for the connect log: "on", or why not.
    pub fn coalescing_status(&self) -> &str {
        &self.coalesce_status
    }

    /// One blocking receive into `buf`, which should be 64 KiB: a coalesced
    /// receive can be as large as the largest datagram. Errors are the
    /// socket's, the read timeout included.
    pub fn recv(&self, buf: &mut [u8]) -> io::Result<Received> {
        #[cfg(windows)]
        {
            if let Some(c) = &self.coalesce {
                return c.recv(&self.inner, buf);
            }
        }
        let (len, src) = self.inner.recv_from(buf)?;
        Ok(Received { len, segment: len, src })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.inner.local_addr()
    }

    #[allow(dead_code)]
    pub fn set_read_timeout(&self, timeout: Option<Duration>) -> io::Result<()> {
        self.inner.set_read_timeout(timeout)
    }

    /// Send from the receive socket's own port (the firewall hole punch).
    #[allow(dead_code)]
    pub fn send_to(&self, buf: &[u8], addr: SocketAddr) -> io::Result<usize> {
        self.inner.send_to(buf, addr)
    }

    /// A second handle on the socket for sending from another thread. It
    /// carries no coalescing state and must not be used to receive.
    pub fn try_clone_sender(&self) -> io::Result<UdpSocket> {
        self.inner.try_clone()
    }
}

/// Receive scratch for a GVSP loop: one OS receive, handed out a datagram at
/// a time. A coalesced receive outlives a single [`RecvBuf::next_datagram`]
/// call, so keep the buffer across calls.
pub struct RecvBuf {
    buf: Vec<u8>,
    len: usize,
    segment: usize,
    pos: usize,
    src: SocketAddr,
}

impl Default for RecvBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl RecvBuf {
    pub fn new() -> Self {
        Self {
            buf: vec![0u8; 65536],
            len: 0,
            segment: 1,
            pos: 0,
            src: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0),
        }
    }

    /// The next datagram and its source, receiving from `socket` once the
    /// previous receive is used up (blocking, subject to the socket's read
    /// timeout; the socket's error otherwise). Empty datagrams are skipped.
    pub fn next_datagram(&mut self, socket: &GvspSocket) -> io::Result<(&[u8], SocketAddr)> {
        loop {
            if self.pos < self.len {
                let start = self.pos;
                let end = (start + self.segment).min(self.len);
                self.pos = end;
                return Ok((&self.buf[start..end], self.src));
            }
            let received = socket.recv(&mut self.buf)?;
            self.load(received);
        }
    }

    fn load(&mut self, r: Received) {
        self.len = r.len.min(self.buf.len());
        self.segment = r.segment.max(1);
        self.pos = 0;
        self.src = r.src;
    }
}

/// Payload of the first control message with `level`/`ty` in a Winsock control
/// buffer (`WSAMSG.Control`). Headers are `{cmsg_len: usize, cmsg_level: i32,
/// cmsg_type: i32}`; the data and each following header are aligned to
/// `size_of::<usize>()` (ws2def.h `WSA_CMSG_*`). Native layout, so the test
/// runs everywhere although only Windows consults it.
#[cfg_attr(not(windows), allow(dead_code))]
pub(crate) fn wsa_cmsg_find(control: &[u8], level: i32, ty: i32) -> Option<&[u8]> {
    let align = size_of::<usize>();
    let up = |n: usize| (n + align - 1) & !(align - 1);
    let hdr = up(align + 8);
    let mut off = 0;
    while off + hdr <= control.len() {
        let len = usize::from_ne_bytes(control[off..off + align].try_into().ok()?);
        let lvl = i32::from_ne_bytes(control[off + align..off + align + 4].try_into().ok()?);
        let typ = i32::from_ne_bytes(control[off + align + 4..off + align + 8].try_into().ok()?);
        if len < hdr || off + len > control.len() {
            return None;
        }
        if lvl == level && typ == ty {
            return Some(&control[off + hdr..off + len]);
        }
        off += up(len);
    }
    None
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

#[cfg(test)]
mod recv_tests {
    use super::*;

    /// One Winsock control message with native header layout.
    fn cmsg(level: i32, ty: i32, data: &[u8]) -> Vec<u8> {
        let align = size_of::<usize>();
        let up = |n: usize| (n + align - 1) & !(align - 1);
        let hdr = up(align + 8);
        let mut v = Vec::new();
        v.extend_from_slice(&(hdr + data.len()).to_ne_bytes());
        v.extend_from_slice(&level.to_ne_bytes());
        v.extend_from_slice(&ty.to_ne_bytes());
        v.resize(hdr, 0);
        v.extend_from_slice(data);
        v.resize(up(v.len()), 0);
        v
    }

    #[test]
    fn cmsg_find_skips_unrelated_messages_and_rejects_bad_lengths() {
        // IPPROTO_IP / IP_PKTINFO-shaped message first, then UDP_COALESCED_INFO.
        let mut control = cmsg(0, 19, &[1, 2, 3, 4, 5, 6, 7, 8]);
        control.extend(cmsg(17, 3, &1472u32.to_ne_bytes()));
        let seg = wsa_cmsg_find(&control, 17, 3).unwrap();
        assert_eq!(u32::from_ne_bytes(seg.try_into().unwrap()), 1472);
        assert!(wsa_cmsg_find(&control, 17, 4).is_none());
        assert!(wsa_cmsg_find(&control[..5], 17, 3).is_none());
        // A header claiming more than the buffer holds is rejected, not read past.
        let mut bad = cmsg(17, 3, &[0; 4]);
        let align = size_of::<usize>();
        bad[..align].copy_from_slice(&200usize.to_ne_bytes());
        assert!(wsa_cmsg_find(&bad, 17, 3).is_none());
    }

    #[test]
    fn recv_buf_splits_a_coalesced_receive() {
        // A loopback socket nothing sends to: splitting must not touch it.
        let sock = GvspSocket::from_udp(UdpSocket::bind("127.0.0.1:0").unwrap());
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let mut rb = RecvBuf::new();
        let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 3956);
        for (i, b) in rb.buf.iter_mut().enumerate().take(34) {
            *b = i as u8;
        }
        rb.load(Received { len: 34, segment: 10, src });
        let mut seen = Vec::new();
        for _ in 0..4 {
            let (d, s) = rb.next_datagram(&sock).unwrap();
            assert_eq!(s, src);
            seen.push(d.to_vec());
        }
        assert_eq!(seen.iter().map(Vec::len).collect::<Vec<_>>(), [10, 10, 10, 4]);
        assert_eq!(seen[1][0], 10);
        assert_eq!(seen[3], [30, 31, 32, 33]);
        // Exhausted: the next call goes to the socket, which times out.
        let err = rb.next_datagram(&sock).unwrap_err();
        assert!(matches!(err.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut), "{err}");
    }

    #[test]
    fn recv_buf_passes_a_single_datagram_through() {
        let sock = GvspSocket::from_udp(UdpSocket::bind("127.0.0.1:0").unwrap());
        sock.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        tx.send_to(b"hello", sock.local_addr().unwrap()).unwrap();
        let mut rb = RecvBuf::new();
        let (d, s) = rb.next_datagram(&sock).unwrap();
        assert_eq!(d, b"hello");
        assert_eq!(s, tx.local_addr().unwrap());
        assert!(!sock.coalescing());
    }
}
