//! Windows-only Winsock extensions for the GVSP receive socket.
//!
//! * [`disable_connreset`]: stop Winsock from reporting an ICMP
//!   port-unreachable — sent by the camera in reply to a datagram this socket
//!   transmitted, i.e. the firewall hole punch — as a `WSAECONNRESET` error on
//!   a later receive.
//! * [`Coalescer`]: UDP receive coalescing (`UDP_RECV_MAX_COALESCED_SIZE`,
//!   Windows 11 / Server 2022). The stack merges consecutive same-size
//!   datagrams from one source into a single receive and reports the
//!   per-datagram size in a `UDP_COALESCED_INFO` control message, so one
//!   `WSARecvMsg` call returns dozens of GVSP payload packets instead of one —
//!   the Winsock counterpart of Linux `recvmmsg`. Requesting the option on an
//!   older Windows fails, and the caller stays with one datagram per receive.
//!   Usage mirrors msquic's `datapath_winuser.c`.

use std::ffi::c_void;
use std::io;
use std::net::UdpSocket;
use std::os::windows::io::AsRawSocket;
use std::ptr::null_mut;

use socket2::SockAddr;
use windows_sys::core::GUID;
use windows_sys::Win32::Networking::WinSock::{
    setsockopt, WSAGetLastError, WSAIoctl, IPPROTO_UDP, LPFN_WSARECVMSG, SIO_GET_EXTENSION_FUNCTION_POINTER,
    SIO_UDP_CONNRESET, SOCKADDR, SOCKET, SOCKET_ERROR, UDP_COALESCED_INFO, UDP_RECV_MAX_COALESCED_SIZE,
    WSABUF, WSAID_WSARECVMSG, WSAMSG,
};

use super::nic::{wsa_cmsg_find, Received};

fn last_error() -> io::Error {
    // SAFETY: reads the calling thread's Winsock error; no arguments.
    io::Error::from_raw_os_error(unsafe { WSAGetLastError() })
}

/// `SIO_UDP_CONNRESET = FALSE` on `socket`.
pub fn disable_connreset(socket: &UdpSocket) -> io::Result<()> {
    let s = socket.as_raw_socket() as SOCKET;
    let enable: i32 = 0; // BOOL FALSE
    let mut returned = 0u32;
    // SAFETY: the input buffer is a live local of the stated size, there is no
    // output buffer, and no overlapped structure so the call is synchronous.
    let rc = unsafe {
        WSAIoctl(
            s,
            SIO_UDP_CONNRESET,
            (&enable as *const i32).cast::<c_void>(),
            size_of::<i32>() as u32,
            null_mut(),
            0,
            &mut returned,
            null_mut(),
            None,
        )
    };
    if rc == SOCKET_ERROR {
        Err(last_error())
    } else {
        Ok(())
    }
}

/// UDP receive coalescing on one socket: the `WSARecvMsg` entry point (fetched
/// per socket, as Winsock requires) once the option has been accepted.
#[derive(Clone, Copy)]
pub struct Coalescer {
    recvmsg: LPFN_WSARECVMSG,
}

impl Coalescer {
    /// Fetch `WSARecvMsg` for `socket` and set `UDP_RECV_MAX_COALESCED_SIZE`
    /// to `max_bytes`. Call before `bind`, the order msquic uses. Fails on
    /// Windows 10 and older, where the option does not exist.
    pub fn enable<S: AsRawSocket>(socket: &S, max_bytes: u32) -> io::Result<Self> {
        let s = socket.as_raw_socket() as SOCKET;
        let guid: GUID = WSAID_WSARECVMSG;
        let mut recvmsg: LPFN_WSARECVMSG = None;
        let mut returned = 0u32;
        // SAFETY: in/out buffers are live locals of the stated sizes; no
        // overlapped structure, so the call completes before returning.
        let rc = unsafe {
            WSAIoctl(
                s,
                SIO_GET_EXTENSION_FUNCTION_POINTER,
                (&guid as *const GUID).cast::<c_void>(),
                size_of::<GUID>() as u32,
                (&mut recvmsg as *mut LPFN_WSARECVMSG).cast::<c_void>(),
                size_of::<LPFN_WSARECVMSG>() as u32,
                &mut returned,
                null_mut(),
                None,
            )
        };
        if rc == SOCKET_ERROR {
            return Err(last_error());
        }
        if recvmsg.is_none() {
            return Err(io::Error::other("WSARecvMsg entry point not returned"));
        }
        // SAFETY: optval points at a live u32 of the stated length.
        let rc = unsafe {
            setsockopt(
                s,
                IPPROTO_UDP,
                UDP_RECV_MAX_COALESCED_SIZE,
                (&max_bytes as *const u32).cast::<u8>(),
                size_of::<u32>() as i32,
            )
        };
        if rc == SOCKET_ERROR {
            return Err(last_error());
        }
        Ok(Self { recvmsg })
    }

    /// One blocking receive into `buf`, which must hold the largest coalesced
    /// message (64 KiB). Returns the byte count, the per-datagram segment size
    /// from the `UDP_COALESCED_INFO` control message (or the byte count when
    /// the receive is a single datagram) and the source address.
    pub fn recv(&self, socket: &UdpSocket, buf: &mut [u8]) -> io::Result<Received> {
        let s = socket.as_raw_socket() as SOCKET;
        let recvmsg = self.recvmsg.expect("checked in Coalescer::enable");
        let mut data = WSABUF { len: buf.len().min(u32::MAX as usize) as u32, buf: buf.as_mut_ptr() };
        // Control buffer for the coalescing message: a 16-byte WSACMSGHDR plus
        // a u32, aligned like the header. Generous.
        let mut control = [0u64; 8];
        let mut received = 0u32;
        // SAFETY: every pointer in `msg` addresses a live local or the
        // caller's buffer for the duration of the synchronous call; the
        // address storage and its length are owned by `try_init`, which
        // reads them back only after the closure returns.
        let ((len, control_len), addr) = unsafe {
            SockAddr::try_init(|storage, addr_len| {
                let mut msg = WSAMSG {
                    name: storage.cast::<SOCKADDR>(),
                    namelen: *addr_len,
                    lpBuffers: &mut data,
                    dwBufferCount: 1,
                    Control: WSABUF { len: size_of_val(&control) as u32, buf: control.as_mut_ptr().cast::<u8>() },
                    dwFlags: 0,
                };
                if recvmsg(s, &mut msg, &mut received, null_mut(), None) == SOCKET_ERROR {
                    return Err(last_error());
                }
                *addr_len = msg.namelen;
                Ok((received as usize, msg.Control.len as usize))
            })
        }?;
        let src = addr
            .as_socket()
            .ok_or_else(|| io::Error::other("GVSP datagram from a non-IP address"))?;
        // SAFETY: `control` is a live u64 array viewed as bytes; the length is
        // clamped to its size.
        let control_bytes = unsafe {
            std::slice::from_raw_parts(control.as_ptr().cast::<u8>(), control_len.min(size_of_val(&control)))
        };
        let segment = wsa_cmsg_find(control_bytes, IPPROTO_UDP, UDP_COALESCED_INFO as i32)
            .filter(|d| d.len() >= 4)
            .map(|d| u32::from_ne_bytes([d[0], d[1], d[2], d[3]]) as usize)
            .filter(|&seg| seg > 0)
            .unwrap_or(len);
        Ok(Received { len, segment: segment.max(1), src })
    }
}
