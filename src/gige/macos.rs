//! macOS-only batched receive for the GVSP socket, and the interface MTU.
//!
//! [`Batcher`]: `recvmsg_x`, Darwin's counterpart of Linux `recvmmsg`
//! (`linux::Batcher`). One call returns every datagram the socket has queued,
//! up to [`BATCH`], each in its own slot with its own length. The function is
//! not in the public SDK headers (xnu `bsd/sys/socket_private.h`), but it has
//! been exported from libsystem_kernel since OS X 10.10 and is what high-rate
//! UDP receivers use on macOS. Its semantics, from xnu
//! `bsd/kern/uipc_syscalls.c`: the call blocks for the first datagram under
//! the socket's blocking mode and `SO_RCVTIMEO`, then returns what is queued
//! up to `cnt` (clamped to `kern.ipc.maxrecvmsgx`, 256 by default); an
//! `EWOULDBLOCK` or `EINTR` after at least one message is reported as success
//! with the count so far; each message reports its byte count in
//! `msg_datalen` and `MSG_TRUNC` in `msg_flags`; the return value is the
//! number of messages, or -1 with `errno`.
//!
//! [`interface_mtu`]: `SIOCGIFMTU`, since macOS has no sysfs and the
//! `if-addrs` crate does not expose the link-level `if_data`.

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::os::fd::AsRawFd;

use socket2::SockAddr;

use super::nic::{Layout, DATAGRAM_SLOT};

/// `struct msghdr_x` (xnu `bsd/sys/socket_private.h`): `struct msghdr` plus
/// `msg_datalen`, the byte count the kernel writes back per message.
#[repr(C)]
struct MsgHdrX {
    msg_name: *mut libc::c_void,
    msg_namelen: libc::socklen_t,
    msg_iov: *mut libc::iovec,
    msg_iovlen: libc::c_int,
    msg_control: *mut libc::c_void,
    msg_controllen: libc::socklen_t,
    msg_flags: libc::c_int,
    msg_datalen: usize,
}

extern "C" {
    /// `ssize_t recvmsg_x(int s, const struct msghdr_x *msgp, u_int cnt, int flags)`.
    /// Despite the `const`, the kernel writes `msg_namelen`, `msg_flags` and
    /// `msg_datalen` back into the array.
    fn recvmsg_x(s: libc::c_int, msgp: *const MsgHdrX, cnt: libc::c_uint, flags: libc::c_int) -> isize;
}

/// Datagrams one `recvmsg_x` may return; the same reasoning as `linux::BATCH`
/// (a frame is thousands of packets at a 1500 MTU; 32 per call cuts the
/// syscall rate by that factor with small stack descriptor arrays).
pub const BATCH: usize = 32;

/// Batched receive on one socket: `recvmsg_x` into [`DATAGRAM_SLOT`]-byte slots.
#[derive(Clone, Copy, Debug)]
pub struct Batcher {
    depth: usize,
}

impl Default for Batcher {
    fn default() -> Self {
        Self::new()
    }
}

impl Batcher {
    /// Batch [`BATCH`] deep.
    pub fn new() -> Self {
        Self { depth: BATCH }
    }

    /// Batch at most `depth` deep (clamped to `1..=BATCH`).
    #[allow(dead_code)]
    pub fn with_depth(depth: usize) -> Self {
        Self { depth: depth.clamp(1, BATCH) }
    }

    /// Most datagrams one receive returns; a [`super::nic::RecvBuf`] holds
    /// this many slots.
    pub fn depth(&self) -> usize {
        self.depth
    }

    /// One receive into `buf`, laid out as consecutive [`DATAGRAM_SLOT`]-byte
    /// slots, with slot `i`'s byte count in `lens[i]`. Blocks for the first
    /// datagram subject to the socket's read timeout, then takes whatever else
    /// is queued, up to the depth and to what `buf` and `lens` can hold. The
    /// source is the first datagram's: GVSP has one sender per socket.
    ///
    /// A datagram larger than its slot (impossible over IPv4) is dropped
    /// rather than delivered cut short: the kernel flags it `MSG_TRUNC`, its
    /// slot gets length 0, and the reader skips it.
    pub fn recv(&self, socket: &UdpSocket, buf: &mut [u8], lens: &mut [usize]) -> io::Result<(Layout, SocketAddr)> {
        let stride = DATAGRAM_SLOT.min(buf.len());
        let count = (buf.len() / stride.max(1)).min(lens.len()).min(self.depth);
        if count == 0 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "recvmsg_x needs a buffer and a length table"));
        }
        // SAFETY: all-zero is a valid value for these plain C structs (null
        // pointers, zero lengths, no flags).
        let mut hdrs: [MsgHdrX; BATCH] = unsafe { std::mem::zeroed() };
        let mut iovs: [libc::iovec; BATCH] = unsafe { std::mem::zeroed() };
        // Descriptors are wired up through raw pointers derived once from each
        // array so that no live Rust reference aliases what the kernel writes.
        let data = buf.as_mut_ptr();
        let iov = iovs.as_mut_ptr();
        for (i, hdr) in hdrs.iter_mut().take(count).enumerate() {
            // SAFETY: slot `i` is `buf[i * stride..(i + 1) * stride]`, in bounds
            // because `count * stride <= buf.len()`; `iov.add(i)` is inside
            // `iovs` because `count <= BATCH`. Both arrays outlive the call.
            unsafe {
                iov.add(i).write(libc::iovec { iov_base: data.add(i * stride).cast(), iov_len: stride });
                hdr.msg_iov = iov.add(i);
            }
            hdr.msg_iovlen = 1;
            hdr.msg_datalen = stride;
        }
        let fd = socket.as_raw_fd();
        // SAFETY: `hdrs[..count]` point at `count` distinct in-bounds slots of
        // `buf`, which is exclusively borrowed for the whole call; the
        // address storage and its length belong to `try_init`, which reads
        // them back only after the closure returns. The kernel writes only
        // through these descriptors and back into `hdrs`.
        let (received, addr) = unsafe {
            SockAddr::try_init(|storage, addr_len| {
                hdrs[0].msg_name = storage.cast();
                hdrs[0].msg_namelen = *addr_len;
                let rc = recvmsg_x(fd, hdrs.as_ptr(), count as libc::c_uint, 0);
                if rc < 0 {
                    return Err(io::Error::last_os_error());
                }
                *addr_len = hdrs[0].msg_namelen;
                Ok(rc as usize)
            })
        }?;
        let received = received.min(count);
        for (i, hdr) in hdrs.iter().take(received).enumerate() {
            let truncated = hdr.msg_flags & libc::MSG_TRUNC != 0;
            lens[i] = if truncated { 0 } else { hdr.msg_datalen.min(stride) };
        }
        let src = match addr.as_socket() {
            Some(src) => src,
            // Nothing arrived, so the storage was never written.
            None if received == 0 => SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0),
            None => return Err(io::Error::other("GVSP datagram from a non-IP address")),
        };
        Ok((Layout::Slots { count: received, stride }, src))
    }
}

/// The interface's link MTU, or `None` if the name is unknown.
pub fn interface_mtu(name: &str) -> Option<u32> {
    let bytes = name.as_bytes();
    // SAFETY: all-zero is a valid `ifreq` (empty name, zeroed union).
    let mut req: libc::ifreq = unsafe { std::mem::zeroed() };
    if bytes.is_empty() || bytes.len() >= req.ifr_name.len() {
        return None;
    }
    for (dst, &b) in req.ifr_name.iter_mut().zip(bytes) {
        *dst = b as libc::c_char;
    }
    // SAFETY: a throwaway datagram socket is the conventional ioctl handle;
    // `req` is a live, correctly sized `ifreq` the kernel fills in place; the
    // descriptor is closed before returning on every path.
    unsafe {
        let fd = libc::socket(libc::AF_INET, libc::SOCK_DGRAM, 0);
        if fd < 0 {
            return None;
        }
        let rc = libc::ioctl(fd, libc::SIOCGIFMTU, &mut req as *mut libc::ifreq);
        let mtu = req.ifr_ifru.ifru_mtu;
        libc::close(fd);
        (rc == 0 && mtu > 0).then_some(mtu as u32)
    }
}
