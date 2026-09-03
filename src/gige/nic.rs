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

/// The OS's batched receive, where it has one: `recvmmsg` on Linux,
/// `recvmsg_x` on macOS. Windows batches differently (coalescing).
#[cfg(target_os = "linux")]
use super::linux::Batcher;
#[cfg(target_os = "macos")]
use super::macos::Batcher;
#[cfg(target_os = "linux")]
const BATCH_MECHANISM: &str = "recvmmsg";
#[cfg(target_os = "macos")]
const BATCH_MECHANISM: &str = "recvmsg_x";

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
/// Linux reads `/sys/class/net/<if>/mtu`; Windows uses `GetIfEntry2`; macOS
/// asks the interface with `SIOCGIFMTU`; every other platform defaults to the
/// canonical Ethernet MTU. Mirrors viva-gige `nic::mtu`.
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

    #[cfg(target_os = "macos")]
    {
        if let Some(mtu) = super::macos::interface_mtu(iface.name()) {
            return mtu;
        }
    }

    #[cfg(not(any(target_os = "linux", windows, target_os = "macos")))]
    {
        let _ = iface;
    }

    1500
}

/// The largest MTU the interface's driver allows, where the OS can say
/// (macOS `SIOCGIFDEVMTU`): the difference between "jumbo frames are off"
/// and "this adapter cannot do jumbo frames".
pub fn max_mtu(iface: &Iface) -> Option<u32> {
    #[cfg(target_os = "macos")]
    {
        super::macos::interface_max_mtu(iface.name())
    }
    #[cfg(not(target_os = "macos"))]
    {
        let _ = iface;
        None
    }
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

/// Bytes one receive slot holds: any IPv4 datagram fits, and the largest is
/// real — packet-size negotiation over Linux loopback clamps to 65535, so the
/// loopback simulator sends ~65.5 KB datagrams. A [`RecvBuf`] is one slot, or
/// one per datagram of a batched receive.
pub const DATAGRAM_SLOT: usize = 65536;

/// Whether the environment variable `name` is set to `0`: the run-time opt-out
/// switches for the OS receive extensions.
#[cfg(any(windows, target_os = "linux", target_os = "macos"))]
fn env_is_zero(name: &str) -> bool {
    std::env::var(name).is_ok_and(|v| v.trim() == "0")
}

/// Bind a blocking UDP socket tuned for GVSP: large SO_RCVBUF, address reuse, a
/// read timeout so the capture loop can service commands between frames, and
/// whatever the OS offers to return more than one datagram per receive:
///
/// * Windows: UDP receive coalescing (refused on Windows 10 and older;
///   `GEV_COALESCE=0` skips it), plus ICMP port-unreachable reporting turned
///   off — see `winsock`.
/// * Linux: `recvmmsg` batching (`GEV_BATCH=0` skips it) — see `linux`.
/// * macOS: `recvmsg_x` batching (`GEV_BATCH=0` skips it) — see `macos`.
///
/// Returns the socket and the receive-buffer size the OS actually granted.
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
    let (coalesce, coalesce_status) = if env_is_zero("GEV_COALESCE") {
        (None, "off (GEV_COALESCE=0)".to_string())
    } else {
        match super::winsock::Coalescer::enable(&socket, MAX_UDP_PAYLOAD) {
            Ok(c) => (Some(c), "on".to_string()),
            Err(e) => (None, format!("off (not available: {e})")),
        }
    };
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    let (batch, coalesce_status) = if env_is_zero("GEV_BATCH") {
        (None, "off (GEV_BATCH=0)".to_string())
    } else {
        let b = Batcher::new();
        (Some(b), format!("batched ({BATCH_MECHANISM}, up to {} per call)", b.depth()))
    };
    #[cfg(not(any(windows, target_os = "linux", target_os = "macos")))]
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
            #[cfg(any(target_os = "linux", target_os = "macos"))]
            batch,
            coalesce_status,
        },
        actual,
    ))
}

/// One packed receive from a [`GvspSocket`]: `len` bytes in the caller's
/// buffer from `src`. When the OS coalesced consecutive same-size datagrams
/// (Windows 11) they are laid out `segment` bytes apart, the last possibly
/// shorter; otherwise a single datagram with `segment == len`. Produced by
/// `winsock`; elsewhere only tests build one.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(not(windows), allow(dead_code))]
pub struct Received {
    pub len: usize,
    pub segment: usize,
    pub src: SocketAddr,
}

#[cfg_attr(not(windows), allow(dead_code))]
impl Received {
    /// The receive as a [`Layout`] plus its source.
    pub fn layout(self) -> (Layout, SocketAddr) {
        (Layout::Packed { len: self.len, segment: self.segment }, self.src)
    }
}

/// Where one receive put its datagrams in the caller's buffer. Two shapes,
/// because the OS extensions differ: Windows coalescing merges same-size
/// datagrams back to back and reports one size; Linux `recvmmsg` keeps each
/// datagram in its own slot and reports each size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Layout {
    /// `len` bytes of consecutive same-size datagrams `segment` apart, the
    /// last possibly shorter; a single datagram has `segment == len`.
    Packed { len: usize, segment: usize },
    /// `count` datagrams at `stride`-byte slots, slot `i` holding `lens[i]`
    /// bytes of the length table the receive was handed. Produced by `linux`
    /// and `macos`; elsewhere only tests build one.
    #[cfg_attr(not(any(target_os = "linux", target_os = "macos")), allow(dead_code))]
    Slots { count: usize, stride: usize },
}

impl Layout {
    /// Byte range of datagram `i`, or `None` once the receive is used up.
    /// Empty ranges are real: an empty datagram, or a slot the receiver
    /// dropped.
    fn slot(self, i: usize, lens: &[usize]) -> Option<std::ops::Range<usize>> {
        match self {
            Layout::Packed { len, segment } => {
                let start = i.checked_mul(segment)?;
                (start < len).then(|| start..start.saturating_add(segment).min(len))
            }
            Layout::Slots { count, stride } => {
                if i >= count {
                    return None;
                }
                let start = i.checked_mul(stride)?;
                let len = lens.get(i).copied().unwrap_or(0).min(stride);
                Some(start..start + len)
            }
        }
    }
}

/// The GVSP receive socket: a blocking UDP socket plus, where the OS offers
/// one, the extension that lets one receive return many packets (coalescing
/// on Windows, `recvmmsg` on Linux, `recvmsg_x` on macOS). Read it through a [`RecvBuf`],
/// which hands such a receive out one datagram at a time.
pub struct GvspSocket {
    inner: UdpSocket,
    #[cfg(windows)]
    coalesce: Option<super::winsock::Coalescer>,
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    batch: Option<Batcher>,
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
            #[cfg(any(target_os = "linux", target_os = "macos"))]
            batch: None,
            coalesce_status: "off".to_string(),
        }
    }

    /// Whether a receive may carry several datagrams.
    #[allow(dead_code)]
    pub fn coalescing(&self) -> bool {
        #[cfg(windows)]
        if self.coalesce.is_some() {
            return true;
        }
        self.batch_depth() > 1
    }

    /// Coalescing state for the connect log: "on", "batched (…)", or why not.
    pub fn coalescing_status(&self) -> &str {
        &self.coalesce_status
    }

    /// Datagrams one receive may return, each needing a [`DATAGRAM_SLOT`] of
    /// buffer: what a [`RecvBuf`] sizes itself to. One unless batching.
    pub fn batch_depth(&self) -> usize {
        #[cfg(any(target_os = "linux", target_os = "macos"))]
        if let Some(b) = &self.batch {
            return b.depth();
        }
        1
    }

    /// One blocking receive into `buf`, which should hold [`batch_depth`]
    /// slots of [`DATAGRAM_SLOT`] bytes — a coalesced receive or any single
    /// datagram can be as large as one slot. A [`Layout::Slots`] receive
    /// records each slot's length in `lens`, so `lens` should have
    /// [`batch_depth`] entries too. Errors are the socket's, the read timeout
    /// included.
    ///
    /// [`batch_depth`]: Self::batch_depth
    pub fn recv(&self, buf: &mut [u8], lens: &mut [usize]) -> io::Result<(Layout, SocketAddr)> {
        #[cfg(windows)]
        if let Some(c) = &self.coalesce {
            return c.recv(&self.inner, buf).map(Received::layout);
        }
        #[cfg(any(target_os = "linux", target_os = "macos"))]
        if let Some(b) = &self.batch {
            return b.recv(&self.inner, buf, lens);
        }
        #[cfg(not(any(target_os = "linux", target_os = "macos")))]
        let _ = lens;
        let (len, src) = self.inner.recv_from(buf)?;
        Ok((Layout::Packed { len, segment: len }, src))
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
/// a time. A multi-datagram receive outlives a single
/// [`RecvBuf::next_datagram`] call, so keep the buffer across calls. It starts
/// as one [`DATAGRAM_SLOT`] and grows, once, to the socket's
/// [`GvspSocket::batch_depth`] slots on first use — callers build it before
/// they know the socket.
pub struct RecvBuf {
    buf: Vec<u8>,
    /// Per-slot byte counts of a [`Layout::Slots`] receive.
    lens: Vec<usize>,
    layout: Layout,
    /// Index of the next datagram to hand out.
    next: usize,
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
            buf: vec![0u8; DATAGRAM_SLOT],
            lens: Vec::new(),
            layout: Layout::Packed { len: 0, segment: 1 },
            next: 0,
            src: SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0),
        }
    }

    /// The next datagram and its source, receiving from `socket` once the
    /// previous receive is used up (blocking, subject to the socket's read
    /// timeout; the socket's error otherwise). Empty datagrams are skipped.
    pub fn next_datagram(&mut self, socket: &GvspSocket) -> io::Result<(&[u8], SocketAddr)> {
        loop {
            while let Some(range) = self.layout.slot(self.next, &self.lens) {
                self.next += 1;
                let end = range.end.min(self.buf.len());
                let start = range.start.min(end);
                if start < end {
                    return Ok((&self.buf[start..end], self.src));
                }
            }
            self.reserve(socket.batch_depth());
            let (layout, src) = socket.recv(&mut self.buf, &mut self.lens)?;
            self.load(layout, src);
        }
    }

    /// Room for `depth` slots and their lengths; never shrinks.
    fn reserve(&mut self, depth: usize) {
        let depth = depth.max(1);
        if self.lens.len() < depth {
            self.lens.resize(depth, 0);
        }
        if self.buf.len() < depth * DATAGRAM_SLOT {
            self.buf.resize(depth * DATAGRAM_SLOT, 0);
        }
    }

    fn load(&mut self, layout: Layout, src: SocketAddr) {
        self.layout = match layout {
            Layout::Packed { len, segment } => Layout::Packed { len: len.min(self.buf.len()), segment: segment.max(1) },
            Layout::Slots { count, stride } => Layout::Slots { count: count.min(self.lens.len()), stride: stride.max(1) },
        };
        self.next = 0;
        self.src = src;
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

    /// A loopback socket nothing sends to, with a short timeout: splitting a
    /// loaded receive must not touch it, and exhausting one must.
    fn idle_socket() -> GvspSocket {
        let sock = GvspSocket::from_udp(UdpSocket::bind("127.0.0.1:0").unwrap());
        sock.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        sock
    }

    fn assert_timed_out(err: io::Error) {
        assert!(matches!(err.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut), "{err}");
    }

    #[test]
    fn recv_buf_splits_a_coalesced_receive() {
        let sock = idle_socket();
        let mut rb = RecvBuf::new();
        let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 3956);
        for (i, b) in rb.buf.iter_mut().enumerate().take(34) {
            *b = i as u8;
        }
        let (layout, s) = Received { len: 34, segment: 10, src }.layout();
        rb.load(layout, s);
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
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
    }

    #[test]
    fn recv_buf_splits_a_batch_of_unequal_datagrams() {
        let sock = idle_socket();
        let mut rb = RecvBuf::new();
        let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 3956);
        // Five slots 100 bytes apart shaped like a frame: a short leader, two
        // full payloads, a short last payload, a short trailer — and each
        // slot's tail past its length holds junk the reader must not return.
        let stride = 100;
        let lens = [16usize, 100, 100, 37, 24];
        rb.buf.fill(0xEE);
        for (i, &n) in lens.iter().enumerate() {
            for (j, b) in rb.buf[i * stride..i * stride + n].iter_mut().enumerate() {
                *b = (i * 50 + j) as u8;
            }
        }
        rb.lens = lens.to_vec();
        rb.load(Layout::Slots { count: lens.len(), stride }, src);
        let mut seen = Vec::new();
        for _ in 0..lens.len() {
            let (d, s) = rb.next_datagram(&sock).unwrap();
            assert_eq!(s, src);
            seen.push(d.to_vec());
        }
        assert_eq!(seen.iter().map(Vec::len).collect::<Vec<_>>(), lens);
        for (i, d) in seen.iter().enumerate() {
            assert!(d.iter().enumerate().all(|(j, &b)| b == (i * 50 + j) as u8), "slot {i} bytes");
        }
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
    }

    #[test]
    fn recv_buf_skips_empty_slots_and_clamps_to_the_slot() {
        let sock = idle_socket();
        let mut rb = RecvBuf::new();
        let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 3956);
        let stride = 8;
        rb.buf[..32].copy_from_slice(&(0..32).collect::<Vec<u8>>());
        // A dropped (truncated) slot, a real one, an empty datagram, and a
        // length past the stride — which a receiver never reports, but the
        // reader must still stay inside the slot.
        rb.lens = vec![0, 5, 0, 99];
        rb.load(Layout::Slots { count: 4, stride }, src);
        let (d, _) = rb.next_datagram(&sock).unwrap();
        assert_eq!(d, [8, 9, 10, 11, 12]);
        let (d, _) = rb.next_datagram(&sock).unwrap();
        assert_eq!(d, [24, 25, 26, 27, 28, 29, 30, 31]);
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
    }

    #[test]
    fn recv_buf_count_is_bounded_by_its_length_table() {
        let sock = idle_socket();
        let mut rb = RecvBuf::new();
        let src = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), 3956);
        rb.lens = vec![3, 3];
        // A receiver claiming more slots than it was given lengths for.
        rb.load(Layout::Slots { count: 10, stride: 4 }, src);
        assert!(rb.next_datagram(&sock).is_ok());
        assert!(rb.next_datagram(&sock).is_ok());
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
    }

    #[test]
    fn recv_buf_grows_to_the_socket_batch_depth() {
        let sock = idle_socket();
        let mut rb = RecvBuf::new();
        assert_eq!(rb.buf.len(), DATAGRAM_SLOT);
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
        // An unbatched socket: one slot, one length.
        assert_eq!(rb.buf.len(), DATAGRAM_SLOT);
        assert_eq!(rb.lens.len(), 1);
        rb.reserve(4);
        assert_eq!(rb.buf.len(), 4 * DATAGRAM_SLOT);
        assert_eq!(rb.lens.len(), 4);
        rb.reserve(1);
        assert_eq!(rb.buf.len(), 4 * DATAGRAM_SLOT, "never shrinks");
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

    /// Datagrams of a frame's shape through a bound (batching) socket arrive
    /// in order and intact. The kernel decides how many share a call; the
    /// test prints the split rather than asserting it.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    fn batched_socket_delivers_every_datagram_in_order() {
        let (sock, _) =
            bind_gvsp_socket(IpAddr::V4(Ipv4Addr::LOCALHOST), 1 << 20, Duration::from_millis(500)).unwrap();
        assert!(sock.coalescing());
        assert_eq!(sock.batch_depth(), Batcher::new().depth());
        assert!(sock.coalescing_status().starts_with("batched ("), "{}", sock.coalescing_status());
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let dst = sock.local_addr().unwrap();
        // Leader-sized, full payloads, a short last payload, trailer-sized,
        // then a datagram larger than an MTU and an empty one.
        let sizes = [44usize, 1472, 1472, 1472, 300, 16, 9000, 0, 1472];
        let sent: Vec<Vec<u8>> = sizes
            .iter()
            .enumerate()
            .map(|(i, &n)| (0..n).map(|j| (i * 31 + j) as u8).collect())
            .collect();
        for d in &sent {
            tx.send_to(d, dst).unwrap();
        }
        let mut rb = RecvBuf::new();
        let mut got = Vec::new();
        // Informational: a fresh receive rewinds the slot index. (A receive
        // whose first slot is the empty datagram can go uncounted.)
        let mut per_call = Vec::new();
        let mut last_next = usize::MAX;
        let expected = sizes.iter().filter(|&&n| n > 0).count();
        while got.len() < expected {
            let (d, src) = rb.next_datagram(&sock).unwrap();
            assert_eq!(src, tx.local_addr().unwrap());
            got.push(d.to_vec());
            if let (true, Layout::Slots { count, .. }) = (rb.next <= last_next, rb.layout) {
                per_call.push(count);
            }
            last_next = rb.next;
        }
        println!("{BATCH_MECHANISM} returned {per_call:?} datagrams per call");
        assert_eq!(got, sent.iter().filter(|d| !d.is_empty()).cloned().collect::<Vec<_>>());
        assert_timed_out(rb.next_datagram(&sock).unwrap_err());
    }

    /// A batch depth of one still goes through the batched call, one datagram
    /// per call, with the read timeout surfacing as the usual error kind.
    #[cfg(any(target_os = "linux", target_os = "macos"))]
    #[test]
    fn batcher_honors_the_read_timeout() {
        let inner = UdpSocket::bind("127.0.0.1:0").unwrap();
        inner.set_read_timeout(Some(Duration::from_millis(20))).unwrap();
        let b = Batcher::with_depth(1);
        let mut buf = vec![0u8; DATAGRAM_SLOT];
        let mut lens = [0usize; 1];
        let started = std::time::Instant::now();
        assert_timed_out(b.recv(&inner, &mut buf, &mut lens).unwrap_err());
        assert!(started.elapsed() < Duration::from_secs(2), "blocked on the socket timeout, not forever");
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        tx.send_to(b"one", inner.local_addr().unwrap()).unwrap();
        tx.send_to(b"two", inner.local_addr().unwrap()).unwrap();
        let (layout, src) = b.recv(&inner, &mut buf, &mut lens).unwrap();
        assert_eq!(layout, Layout::Slots { count: 1, stride: DATAGRAM_SLOT });
        assert_eq!(src, tx.local_addr().unwrap());
        assert_eq!(&buf[..lens[0]], b"one");
        let (layout, _) = b.recv(&inner, &mut buf, &mut lens).unwrap();
        assert_eq!(layout, Layout::Slots { count: 1, stride: DATAGRAM_SLOT });
        assert_eq!(&buf[..lens[0]], b"two");
    }

    #[cfg(target_os = "macos")]
    #[test]
    fn macos_reads_the_interface_mtu() {
        // Loopback on macOS is 16384; an unknown name is None, not 1500.
        assert_eq!(super::super::macos::interface_mtu("lo0"), Some(16384));
        assert_eq!(super::super::macos::interface_mtu("nosuch0"), None);
        assert_eq!(super::super::macos::interface_mtu(""), None);
        // Not every driver answers the device-MTU query (loopback does not);
        // one that does reports a ceiling no lower than the configured MTU.
        for name in ["lo0", "en0"] {
            if let (Some(mtu), Some(max)) =
                (super::super::macos::interface_mtu(name), super::super::macos::interface_max_mtu(name))
            {
                println!("{name}: mtu {mtu}, driver max {max}");
                assert!(max >= mtu, "{name}: {max} < {mtu}");
            }
        }
        assert_eq!(super::super::macos::interface_max_mtu("nosuch0"), None);
    }
}
