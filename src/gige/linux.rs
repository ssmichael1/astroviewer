//! Linux-only batched receive for the GVSP socket.
//!
//! [`Batcher`]: `recvmmsg(2)`, the Linux counterpart of Windows UDP receive
//! coalescing (`winsock::Coalescer`). One syscall returns every datagram the
//! socket has queued, up to [`BATCH`], so the receive thread makes a syscall
//! per burst rather than per packet at 80k+ packets/s. Unlike coalescing the
//! kernel does not merge datagrams: each lands in its own slot of the caller's
//! buffer and reports its own length, which is what GVSP needs — the leader,
//! trailer and last payload packet of a frame are shorter than the rest.
//!
//! The call is made with `MSG_WAITFORONE` and no timeout struct: it blocks for
//! the first datagram under the socket's `SO_RCVTIMEO` (so a read timeout
//! surfaces as `EAGAIN`, the same `WouldBlock` the plain path reports) and then
//! returns at once with whatever else is queued. `recvmmsg`'s own timeout
//! argument is deliberately unused — it is only checked between datagrams, so
//! it cannot bound a wait for the first one (recvmmsg(2), BUGS).

use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::os::fd::AsRawFd;

use socket2::SockAddr;

use super::nic::{Layout, DATAGRAM_SLOT};

/// Datagrams one `recvmmsg` may return. At a 1500-byte MTU a 5 MP Mono8 frame
/// is ~3500 packets; 32 per call cuts the syscall rate by that factor while the
/// per-call descriptor arrays (an `mmsghdr` and an `iovec` each) stay small on
/// the stack. Deeper batches only help when more is queued at once, which at
/// line rate means the receive thread is already behind by 2 MiB.
pub const BATCH: usize = 32;

/// Batched receive on one socket: `recvmmsg` into [`DATAGRAM_SLOT`]-byte slots.
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

    /// Batch at most `depth` deep (clamped to `1..=BATCH`, the descriptor
    /// array size). Shallower batches are for tests that want several calls.
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
    /// source is the first datagram's: GVSP has one sender per socket, so a
    /// per-datagram address is not worth its storage.
    ///
    /// A datagram larger than its slot (impossible over IPv4, whose datagrams
    /// fit) is dropped rather than delivered cut short: the kernel flags it
    /// `MSG_TRUNC`, its slot gets length 0, and the reader skips it. Its
    /// resend, if it was a payload packet, is a matter for the GVSP layer.
    pub fn recv(&self, socket: &UdpSocket, buf: &mut [u8], lens: &mut [usize]) -> io::Result<(Layout, SocketAddr)> {
        let stride = DATAGRAM_SLOT.min(buf.len());
        let count = (buf.len() / stride.max(1)).min(lens.len()).min(self.depth);
        if count == 0 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "recvmmsg needs a buffer and a length table"));
        }
        // SAFETY: all-zero is a valid value for these plain C structs (null
        // pointers, zero lengths, no flags).
        let mut hdrs: [libc::mmsghdr; BATCH] = unsafe { std::mem::zeroed() };
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
                hdr.msg_hdr.msg_iov = iov.add(i);
            }
            hdr.msg_hdr.msg_iovlen = 1 as _;
        }
        let fd = socket.as_raw_fd();
        // SAFETY: `hdrs[..count]` point at `count` distinct in-bounds slots of
        // `buf`, which is exclusively borrowed for the whole call; the
        // address storage and its length belong to `try_init`, which reads
        // them back only after the closure returns; the timeout is null by
        // design (see the module doc). The kernel writes only through these.
        let (received, addr) = unsafe {
            SockAddr::try_init(|storage, addr_len| {
                hdrs[0].msg_hdr.msg_name = storage.cast();
                hdrs[0].msg_hdr.msg_namelen = *addr_len;
                let rc = libc::recvmmsg(
                    fd,
                    hdrs.as_mut_ptr(),
                    count as libc::c_uint,
                    libc::MSG_WAITFORONE as _,
                    std::ptr::null_mut(),
                );
                if rc < 0 {
                    return Err(io::Error::last_os_error());
                }
                *addr_len = hdrs[0].msg_hdr.msg_namelen;
                Ok(rc as usize)
            })
        }?;
        let received = received.min(count);
        for (i, hdr) in hdrs.iter().take(received).enumerate() {
            let truncated = hdr.msg_hdr.msg_flags & libc::MSG_TRUNC != 0;
            lens[i] = if truncated { 0 } else { (hdr.msg_len as usize).min(stride) };
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
