//! GVCP (GigE Vision Control Protocol) over UDP — synchronous, std-only.
//!
//! Wire layouts, register addresses, status codes and the spec-derived
//! golden-byte tests are adapted from the MIT-licensed `viva-gige` /
//! `viva-gencp` crates (`gvcp.rs`, `lib.rs`,
//! <https://github.com/VitalyVorobyev/viva-genicam>). This is a blocking-socket
//! reimplementation owned by the app: no async runtime, no `bytes`.

use std::io::{self, ErrorKind};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use socket2::{Domain, Protocol, Socket, Type};

use super::nic::{self, Iface};

/// GVCP control port (GigE Vision §7.3).
pub const GVCP_PORT: u16 = 3956;

/// First byte of every GVCP command packet.
const CMD_KEY: u8 = 0x42;
/// Flags byte bits.
const FLAG_ACK_REQUIRED: u8 = 0x01;
const FLAG_BROADCAST: u8 = 0x10;
/// PACKETRESEND flag: the payload uses the GigE Vision 2.0 extended-ID layout
/// (64-bit block id). The same bit as `FLAG_BROADCAST`; flags are per command.
const FLAG_EXTENDED_IDS: u8 = 0x10;

/// GenCP / GVCP acknowledgement header size.
const HEADER_SIZE: usize = 8;

// ── Opcodes ─────────────────────────────────────────────────────────────────
const DISCOVERY_CMD: u16 = 0x0002;
const DISCOVERY_ACK: u16 = 0x0003;
/// FORCEIP: give the device with a given MAC a temporary IP configuration.
/// Broadcast, since the device may currently hold an address this host
/// cannot route to. 56-byte payload — see [`encode_forceip_payload`].
const FORCEIP_CMD: u16 = 0x0004;
const FORCEIP_ACK: u16 = 0x0005;
const READREG_CMD: u16 = 0x0080;
#[allow(dead_code)]
const READREG_ACK: u16 = 0x0081;
const WRITEREG_CMD: u16 = 0x0082;
#[allow(dead_code)]
const WRITEREG_ACK: u16 = 0x0083;
const READMEM_CMD: u16 = 0x0084;
#[allow(dead_code)]
const READMEM_ACK: u16 = 0x0085;
const WRITEMEM_CMD: u16 = 0x0086;
#[allow(dead_code)]
const WRITEMEM_ACK: u16 = 0x0087;
/// PENDING_ACK (GigE Vision 1.2 §18.5): the device wants more time.
const PENDING_ACK: u16 = 0x0089;
/// PACKETRESEND: ask the device to retransmit stream packets. GigE Vision 1.x
/// payload (12 bytes, as aravis encodes it): `stream_channel(2) | block_id(2)
/// | first_packet_id(4) | last_packet_id(4)`; packet ids are 24-bit in the
/// 32-bit fields. (The 8-byte `block | reserved | first16 | last16` form some
/// libraries send is not the specified layout and cameras ignore it.)
const PACKET_RESEND_CMD: u16 = 0x0040;

// ── Bootstrap registers ───────────────────────────────────────────────────--
/// Control Channel Privilege.
pub const CCP_REGISTER: u32 = 0x0a00;
/// `GevCurrentIPConfiguration`: bit 0 LLA, bit 1 persistent IP, bit 2 DHCP.
pub const IP_CONFIG_REGISTER: u32 = 0x0014;
/// `GevCurrentIPConfiguration` bit: link-local addressing enabled.
pub const IP_CONFIG_LLA: u32 = 0x1;
/// `GevCurrentIPConfiguration` bit: boot with the persistent IP.
pub const IP_CONFIG_PERSISTENT: u32 = 0x2;
/// `GevCurrentIPConfiguration` bit: DHCP enabled.
pub const IP_CONFIG_DHCP: u32 = 0x4;
/// Persistent IP address / subnet mask / default gateway registers (the
/// address sits in the last word of each 16-byte block).
const PERSISTENT_IP_REGISTER: u32 = 0x064c;
const PERSISTENT_SUBNET_REGISTER: u32 = 0x065c;
const PERSISTENT_GATEWAY_REGISTER: u32 = 0x066c;
/// CCP value claiming control access.
const CCP_CONTROL: u32 = 1 << 1;
/// Bits of CCP that mean "this application is the controller".
#[allow(dead_code)]
pub const CCP_CONTROLLER_BITS: u32 = (1 << 1) | (1 << 0);
/// Stream channel 0 block base; each channel strides by 0x40.
const STREAM_CHANNEL_BASE: u64 = 0x0d00;
const STREAM_CHANNEL_STRIDE: u64 = 0x40;
const STREAM_DEST_PORT: u64 = 0x00; // GevSCPHostPort
const STREAM_PACKET_SIZE: u64 = 0x04; // GevSCPSPacketSize
const STREAM_PACKET_DELAY: u64 = 0x08; // GevSCPD
const STREAM_DEST_ADDRESS: u64 = 0x18; // GevSCDA

/// Bits 0-15 of `GevSCPSPacketSize` hold the size; masking matters on read too
/// (a device may leave the do-not-fragment bit set). Aravis
/// `ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_MASK = 0x0000ffff`
/// (`arvgvcpprivate.h`, line 129).
const STREAM_PACKET_SIZE_MASK: u32 = 0xFFFF;
/// `GevSCPSPacketSize` do-not-fragment flag: the device sets the IP
/// Don't-Fragment bit on the packets of this channel. Aravis
/// `ARV_GVBS_STREAM_CHANNEL_0_PACKET_DO_NOT_FRAGMENT = 1 << 30`
/// (`arvgvcpprivate.h`, line 132).
const SCPS_DO_NOT_FRAGMENT: u32 = 1 << 30;
/// `GevSCPSPacketSize` fire-test-packet flag: writing the register with this
/// bit set makes the device emit one test packet of the requested size (the
/// bit is self-clearing / edge triggered). Aravis
/// `ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_FIRE_TEST = 1 << 31`
/// (`arvgvcpprivate.h`, line 133).
const SCPS_FIRE_TEST_PACKET: u32 = 1 << 31;

/// Encode a `GevSCPSPacketSize` register value: the 16-bit packet size in the
/// low bits, optionally OR-ed with the do-not-fragment and fire-test-packet
/// flag bits. Masks per aravis `arvgvcpprivate.h`.
pub(crate) fn encode_stream_packet_size(size: u32, do_not_fragment: bool, fire_test: bool) -> u32 {
    let mut v = size & STREAM_PACKET_SIZE_MASK;
    if do_not_fragment {
        v |= SCPS_DO_NOT_FRAGMENT;
    }
    if fire_test {
        v |= SCPS_FIRE_TEST_PACKET;
    }
    v
}

/// Binary-search the largest packet size in `floor..=ceil` for which
/// `arrives(size)` is true, assuming monotonicity: if a size is carried, every
/// smaller one is too (the path MTU is a threshold). `ceil` is tried first (the
/// common good path — a jumbo link whose current size already works — returns
/// on the first probe), then `floor`; `None` means even `floor` did not arrive,
/// so the caller can fall back cleanly (the camera ignores test packets, or the
/// path is narrower than the floor). `arrives` is expected to be reliable — any
/// per-candidate retrying belongs inside it.
pub(crate) fn largest_carried_packet_size(
    floor: u32,
    ceil: u32,
    mut arrives: impl FnMut(u32) -> bool,
) -> Option<u32> {
    if ceil < floor {
        return None;
    }
    if arrives(ceil) {
        return Some(ceil);
    }
    if !arrives(floor) {
        return None;
    }
    // Invariant: `lo` is known to arrive, `hi` is known not to.
    let mut lo = floor;
    let mut hi = ceil;
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if arrives(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(lo)
}

// ── Transaction policy ──────────────────────────────────────────────────────
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);
const MAX_RETRIES: usize = 4;
const MAX_PENDING_ACKS: usize = 100;
const MAX_PENDING_ACK_WAIT: Duration = Duration::from_secs(10);
const RETRY_BASE_DELAY: Duration = Duration::from_millis(20);
/// Largest READMEM/WRITEMEM chunk (GenCP block limit).
const GENCP_MAX_BLOCK: usize = 512;

/// GVCP/GenCP acknowledgement status codes (the shared core the two protocols
/// define identically). Values corroborated by Wireshark's `GEV_STATUS_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Success,
    NotImplemented,
    InvalidParameter,
    InvalidAddress,
    WriteProtect,
    BadAlignment,
    AccessDenied,
    Busy,
    LocalProblem,
    MsgMismatch,
    InvalidProtocol,
    NoMsg,
    PacketUnavailable,
    DataOverrun,
    InvalidHeader,
    GenericError,
    Unknown(u16),
}

impl Status {
    pub fn from_raw(raw: u16) -> Self {
        match raw {
            0x0000 => Status::Success,
            0x8001 => Status::NotImplemented,
            0x8002 => Status::InvalidParameter,
            0x8003 => Status::InvalidAddress,
            0x8004 => Status::WriteProtect,
            0x8005 => Status::BadAlignment,
            0x8006 => Status::AccessDenied,
            0x8007 => Status::Busy,
            0x8008 => Status::LocalProblem,
            0x8009 => Status::MsgMismatch,
            0x800A => Status::InvalidProtocol,
            0x800B => Status::NoMsg,
            0x800C => Status::PacketUnavailable,
            0x800D => Status::DataOverrun,
            0x800E => Status::InvalidHeader,
            0x8FFF => Status::GenericError,
            other => Status::Unknown(other),
        }
    }

    pub fn to_raw(self) -> u16 {
        match self {
            Status::Success => 0x0000,
            Status::NotImplemented => 0x8001,
            Status::InvalidParameter => 0x8002,
            Status::InvalidAddress => 0x8003,
            Status::WriteProtect => 0x8004,
            Status::BadAlignment => 0x8005,
            Status::AccessDenied => 0x8006,
            Status::Busy => 0x8007,
            Status::LocalProblem => 0x8008,
            Status::MsgMismatch => 0x8009,
            Status::InvalidProtocol => 0x800A,
            Status::NoMsg => 0x800B,
            Status::PacketUnavailable => 0x800C,
            Status::DataOverrun => 0x800D,
            Status::InvalidHeader => 0x800E,
            Status::GenericError => 0x8FFF,
            Status::Unknown(code) => code,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Status::Success => "SUCCESS",
            Status::NotImplemented => "NOT_IMPLEMENTED",
            Status::InvalidParameter => "INVALID_PARAMETER",
            Status::InvalidAddress => "INVALID_ADDRESS",
            Status::WriteProtect => "WRITE_PROTECT",
            Status::BadAlignment => "BAD_ALIGNMENT",
            Status::AccessDenied => "ACCESS_DENIED",
            Status::Busy => "BUSY",
            Status::LocalProblem => "LOCAL_PROBLEM",
            Status::MsgMismatch => "MSG_MISMATCH",
            Status::InvalidProtocol => "INVALID_PROTOCOL",
            Status::NoMsg => "NO_MSG",
            Status::PacketUnavailable => "PACKET_UNAVAILABLE",
            Status::DataOverrun => "DATA_OVERRUN",
            Status::InvalidHeader => "INVALID_HEADER",
            Status::GenericError => "ERROR",
            Status::Unknown(_) => "UNKNOWN",
        }
    }

    /// Only `BUSY` is congestion; everything else is a definite answer.
    pub fn is_retryable(self) -> bool {
        matches!(self, Status::Busy)
    }
}

impl std::fmt::Display for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} (0x{:04X})", self.name(), self.to_raw())
    }
}

/// Errors from the GVCP control path.
#[derive(Debug)]
pub enum GvcpError {
    Io(io::Error),
    Timeout,
    Protocol(String),
    Status(Status),
}

impl std::fmt::Display for GvcpError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GvcpError::Io(e) => write!(f, "io: {e}"),
            GvcpError::Timeout => write!(f, "timeout waiting for acknowledgement"),
            GvcpError::Protocol(s) => write!(f, "protocol: {s}"),
            GvcpError::Status(s) => write!(f, "device reported status {s}"),
        }
    }
}

impl std::error::Error for GvcpError {}

impl From<io::Error> for GvcpError {
    fn from(e: io::Error) -> Self {
        GvcpError::Io(e)
    }
}

/// Encode a GVCP command packet (`key | flags | command | length | request_id |
/// payload`).
fn encode_command(flags: u8, command: u16, request_id: u16, payload: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len());
    buf.push(CMD_KEY);
    buf.push(flags);
    buf.extend_from_slice(&command.to_be_bytes());
    buf.extend_from_slice(&(payload.len() as u16).to_be_bytes());
    buf.extend_from_slice(&request_id.to_be_bytes());
    buf.extend_from_slice(payload);
    buf
}

/// A decoded acknowledgement: `status(2) | command(2) | length(2) |
/// request_id(2) | payload`.
struct Ack {
    status: Status,
    command: u16,
    request_id: u16,
    payload: Vec<u8>,
}

fn decode_ack(buf: &[u8]) -> Result<Ack, GvcpError> {
    if buf.len() < HEADER_SIZE {
        return Err(GvcpError::Protocol("acknowledgement too short".into()));
    }
    let status = Status::from_raw(u16::from_be_bytes([buf[0], buf[1]]));
    let command = u16::from_be_bytes([buf[2], buf[3]]);
    let length = u16::from_be_bytes([buf[4], buf[5]]) as usize;
    let request_id = u16::from_be_bytes([buf[6], buf[7]]);
    if buf.len() < HEADER_SIZE + length {
        return Err(GvcpError::Protocol("acknowledgement truncated".into()));
    }
    Ok(Ack {
        status,
        command,
        request_id,
        payload: buf[HEADER_SIZE..HEADER_SIZE + length].to_vec(),
    })
}

/// Decode a PENDING_ACK: returns `(request_id, extra_time)`. Layout per §18.5:
/// the 8-byte header, then reserved(2) and a 16-bit `time_to_completion`.
fn parse_pending_ack(buf: &[u8]) -> Option<(u16, Duration)> {
    if buf.len() < HEADER_SIZE {
        return None;
    }
    if u16::from_be_bytes([buf[2], buf[3]]) != PENDING_ACK {
        return None;
    }
    let request_id = u16::from_be_bytes([buf[6], buf[7]]);
    let payload = &buf[HEADER_SIZE..];
    // A truncated PENDING_ACK is still an unambiguous request for more time.
    let millis = match payload {
        [_, _, hi, lo, ..] => u16::from_be_bytes([*hi, *lo]),
        _ => return Some((request_id, CONTROL_TIMEOUT)),
    };
    Some((request_id, Duration::from_millis(u64::from(millis))))
}

/// Information from a Discovery ACK.
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    pub ip: Ipv4Addr,
    pub mac: [u8; 6],
    pub model: Option<String>,
    pub manufacturer: Option<String>,
}

/// A connected GVCP device handle.
pub struct Device {
    socket: UdpSocket,
    remote: SocketAddr,
    request_id: u16,
}

/// Values written to the device during stream negotiation.
#[derive(Debug, Clone, Copy)]
pub struct StreamParams {
    pub packet_size: u32,
    pub mtu: u32,
    pub host: Ipv4Addr,
    #[allow(dead_code)]
    pub port: u16,
}

impl Device {
    /// Connect to a GVCP endpoint (does not claim control). The address may name
    /// any port, so tests can target a simulator on a non-standard port.
    pub fn open(addr: SocketAddr) -> Result<Self, GvcpError> {
        let local = match addr.ip() {
            IpAddr::V4(_) => IpAddr::V4(Ipv4Addr::UNSPECIFIED),
            IpAddr::V6(_) => return Err(GvcpError::Protocol("IPv6 GVCP unsupported".into())),
        };
        let socket = UdpSocket::bind(SocketAddr::new(local, 0))?;
        socket.connect(addr)?;
        Ok(Self {
            socket,
            remote: addr,
            request_id: 1,
        })
    }

    pub fn remote_addr(&self) -> SocketAddr {
        self.remote
    }

    fn next_request_id(&mut self) -> u16 {
        let id = self.request_id;
        self.request_id = self.request_id.wrapping_add(1);
        if self.request_id == 0 {
            self.request_id = 1; // request ids are 1..=65535, never 0
        }
        id
    }

    /// Send `command` with `payload` and wait for the matching acknowledgement,
    /// retrying the SAME request id on BUSY / timeout / transient IO. PENDING_ACK
    /// extends the deadline without resending (a resend could run a flash write
    /// twice); a mismatched ack id is ignored, bounded.
    fn transact(&mut self, command: u16, payload: &[u8]) -> Result<Ack, GvcpError> {
        let request_id = self.next_request_id();
        let encoded = encode_command(FLAG_ACK_REQUIRED, command, request_id, payload);
        let ack_command = command + 1;
        let mut buf = vec![0u8; HEADER_SIZE + GENCP_MAX_BLOCK + 8];

        'attempts: for attempt in 1..=MAX_RETRIES {
            if let Err(e) = self.socket.send(&encoded) {
                if attempt >= MAX_RETRIES {
                    return Err(e.into());
                }
                self.backoff(attempt);
                continue;
            }

            let mut mismatched = 0usize;
            let mut wait = CONTROL_TIMEOUT;
            let mut pending_seen = 0usize;
            loop {
                match self.recv_within(&mut buf, wait) {
                    RecvOutcome::Received(len) => {
                        // PENDING_ACK: extend the deadline, keep waiting.
                        if let Some((pending_id, requested)) = parse_pending_ack(&buf[..len]) {
                            if pending_id == request_id {
                                pending_seen += 1;
                                if pending_seen > MAX_PENDING_ACKS {
                                    return Err(GvcpError::Timeout);
                                }
                                wait = requested.clamp(CONTROL_TIMEOUT, MAX_PENDING_ACK_WAIT);
                            }
                            continue;
                        }
                        let ack = decode_ack(&buf[..len])?;
                        if ack.request_id != request_id {
                            // A delayed ack from an earlier command; bounded skip.
                            mismatched += 1;
                            if mismatched < MAX_RETRIES {
                                continue;
                            }
                            if attempt >= MAX_RETRIES {
                                return Err(GvcpError::Protocol("acknowledgement id mismatch".into()));
                            }
                            self.backoff(attempt);
                            continue 'attempts;
                        }
                        if ack.command != ack_command {
                            return Err(GvcpError::Protocol(format!(
                                "unexpected ack opcode {:#06x}",
                                ack.command
                            )));
                        }
                        match ack.status {
                            Status::Success => return Ok(ack),
                            s if s.is_retryable() && attempt < MAX_RETRIES => {
                                self.backoff(attempt);
                                continue 'attempts;
                            }
                            other => return Err(GvcpError::Status(other)),
                        }
                    }
                    RecvOutcome::Io(e) => {
                        if attempt >= MAX_RETRIES {
                            return Err(e.into());
                        }
                        self.backoff(attempt);
                        continue 'attempts;
                    }
                    RecvOutcome::TimedOut => {
                        if attempt >= MAX_RETRIES {
                            return Err(GvcpError::Timeout);
                        }
                        self.backoff(attempt);
                        continue 'attempts;
                    }
                }
            }
        }
        Err(GvcpError::Timeout)
    }

    /// Receive one datagram, waiting at most `wait` (via the socket read
    /// timeout). Distinguishes a datagram, a transient IO error, and a deadline.
    fn recv_within(&self, buf: &mut [u8], wait: Duration) -> RecvOutcome {
        if self.socket.set_read_timeout(Some(wait)).is_err() {
            return RecvOutcome::TimedOut;
        }
        match self.socket.recv(buf) {
            Ok(len) => RecvOutcome::Received(len),
            Err(e) => match e.kind() {
                ErrorKind::WouldBlock | ErrorKind::TimedOut => RecvOutcome::TimedOut,
                // Windows surfaces an ICMP port-unreachable from an earlier send
                // as ConnectionReset on the next recv; not fatal.
                ErrorKind::ConnectionReset | ErrorKind::Interrupted => RecvOutcome::TimedOut,
                _ => RecvOutcome::Io(e),
            },
        }
    }

    fn backoff(&self, attempt: usize) {
        let mult = 1u32 << (attempt.saturating_sub(1)).min(3);
        let delay = RETRY_BASE_DELAY.saturating_mul(mult);
        std::thread::sleep(delay);
    }

    /// Read a 32-bit register.
    pub fn read_register(&mut self, addr: u32) -> Result<u32, GvcpError> {
        let ack = self.transact(READREG_CMD, &addr.to_be_bytes())?;
        if ack.payload.len() != 4 {
            return Err(GvcpError::Protocol(format!(
                "expected 4-byte register value, got {}",
                ack.payload.len()
            )));
        }
        Ok(u32::from_be_bytes([
            ack.payload[0],
            ack.payload[1],
            ack.payload[2],
            ack.payload[3],
        ]))
    }

    /// Write a 32-bit register.
    pub fn write_register(&mut self, addr: u32, value: u32) -> Result<(), GvcpError> {
        let mut payload = [0u8; 8];
        payload[..4].copy_from_slice(&addr.to_be_bytes());
        payload[4..].copy_from_slice(&value.to_be_bytes());
        let ack = self.transact(WRITEREG_CMD, &payload)?;
        // WRITEREG_ACK carries a 4-byte data-index placeholder.
        if ack.payload.len() != 4 {
            return Err(GvcpError::Protocol(format!(
                "expected 4-byte write ack, got {}",
                ack.payload.len()
            )));
        }
        Ok(())
    }

    /// Read a block of memory, chunked and 4-byte-count-aligned (strict cameras
    /// reject unaligned counts). READMEM_ACK = 4-byte address echo + data.
    pub fn read_mem(&mut self, addr: u64, len: usize) -> Result<Vec<u8>, GvcpError> {
        let mut out = Vec::with_capacity(len);
        let mut remaining = len;
        let mut offset = 0usize;
        while remaining > 0 {
            let chunk = remaining.min(GENCP_MAX_BLOCK);
            let request = chunk.next_multiple_of(4);
            let mut payload = [0u8; 8];
            payload[..4].copy_from_slice(&((addr + offset as u64) as u32).to_be_bytes());
            // payload[4..6] reserved = 0
            payload[6..8].copy_from_slice(&(request as u16).to_be_bytes());
            let ack = self.transact(READMEM_CMD, &payload)?;
            let data = if ack.payload.len() >= 4 + request {
                &ack.payload[4..4 + request]
            } else if ack.payload.len() == request {
                &ack.payload[..request] // some devices omit the address echo
            } else {
                return Err(GvcpError::Protocol(format!(
                    "READMEM expected {request} bytes, got {}",
                    ack.payload.len()
                )));
            };
            out.extend_from_slice(&data[..chunk]);
            remaining -= chunk;
            offset += chunk;
        }
        Ok(out)
    }

    /// Write a block of memory, chunked. WRITEMEM = 4-byte address + data.
    pub fn write_mem(&mut self, addr: u64, data: &[u8]) -> Result<(), GvcpError> {
        const OVERHEAD: usize = 4;
        let mut offset = 0usize;
        while offset < data.len() {
            let chunk = (data.len() - offset).min(GENCP_MAX_BLOCK - OVERHEAD);
            if chunk == 0 {
                return Err(GvcpError::Protocol("zero write chunk".into()));
            }
            let mut payload = Vec::with_capacity(OVERHEAD + chunk);
            payload.extend_from_slice(&((addr + offset as u64) as u32).to_be_bytes());
            payload.extend_from_slice(&data[offset..offset + chunk]);
            let ack = self.transact(WRITEMEM_CMD, &payload)?;
            if ack.payload.len() > 4 {
                return Err(GvcpError::Protocol("write ack carried payload".into()));
            }
            offset += chunk;
        }
        Ok(())
    }

    /// A [`ResendSender`] for the receive thread: a second handle on this
    /// socket, so PACKETRESEND leaves from the port holding control privilege.
    pub fn resend_sender(&self, channel: u16) -> io::Result<ResendSender> {
        Ok(ResendSender { socket: self.socket.try_clone()?, channel, request_id: 0x8000 })
    }

    /// Read the persistent (boot-time) IP, subnet mask and gateway.
    pub fn read_persistent_ip(&mut self) -> Result<(Ipv4Addr, Ipv4Addr, Ipv4Addr), GvcpError> {
        Ok((
            Ipv4Addr::from(self.read_register(PERSISTENT_IP_REGISTER)?),
            Ipv4Addr::from(self.read_register(PERSISTENT_SUBNET_REGISTER)?),
            Ipv4Addr::from(self.read_register(PERSISTENT_GATEWAY_REGISTER)?),
        ))
    }

    /// Write the persistent IP, subnet mask and gateway. Takes effect at the
    /// next boot, and only if [`Self::enable_persistent_ip`] has set the
    /// configuration bit. Requires control privilege.
    pub fn write_persistent_ip(&mut self, ip: Ipv4Addr, subnet: Ipv4Addr, gateway: Ipv4Addr) -> Result<(), GvcpError> {
        self.write_register(PERSISTENT_IP_REGISTER, u32::from(ip))?;
        self.write_register(PERSISTENT_SUBNET_REGISTER, u32::from(subnet))?;
        self.write_register(PERSISTENT_GATEWAY_REGISTER, u32::from(gateway))
    }

    /// Read `GevCurrentIPConfiguration` (see the `IP_CONFIG_*` bits).
    pub fn ip_config(&mut self) -> Result<u32, GvcpError> {
        self.read_register(IP_CONFIG_REGISTER)
    }

    /// Write `GevCurrentIPConfiguration`. Requires control privilege.
    pub fn set_ip_config(&mut self, bits: u32) -> Result<(), GvcpError> {
        self.write_register(IP_CONFIG_REGISTER, bits)
    }

    /// Set the persistent-IP bit in `GevCurrentIPConfiguration`, leaving the
    /// others as they are.
    pub fn enable_persistent_ip(&mut self) -> Result<(), GvcpError> {
        let cfg = self.ip_config()?;
        self.set_ip_config(cfg | IP_CONFIG_PERSISTENT)
    }

    /// Claim control-channel privilege (required before configuring streaming).
    pub fn claim_control(&mut self) -> Result<(), GvcpError> {
        self.write_register(CCP_REGISTER, CCP_CONTROL)
    }

    /// Release control-channel privilege.
    pub fn release_control(&mut self) -> Result<(), GvcpError> {
        self.write_register(CCP_REGISTER, 0)
    }

    fn stream_reg(channel: u32, offset: u64) -> u64 {
        STREAM_CHANNEL_BASE + channel as u64 * STREAM_CHANNEL_STRIDE + offset
    }

    /// Configure the GVSP stream channel toward the host `iface`:`port`, sizing
    /// packets from the link MTU (optionally capped). Writes SCDA, host port,
    /// packet size and packet delay via WRITEMEM.
    pub fn negotiate_stream(
        &mut self,
        channel: u32,
        iface: &Iface,
        port: u16,
        packet_cap: Option<u32>,
    ) -> Result<StreamParams, GvcpError> {
        let host = iface
            .ipv4()
            .ok_or_else(|| GvcpError::Protocol("interface lacks IPv4 address".into()))?;
        let iface_mtu = nic::mtu(iface);
        let mtu = packet_cap.map_or(iface_mtu, |cap| cap.min(iface_mtu));
        let packet_size = nic::best_packet_size(mtu);
        // GevSCPD is in 80 ns ticks; space packets ~2 µs apart on a 1500 MTU link.
        let packet_delay: u32 = if mtu <= 1500 { 2_000 / 80 } else { 0 };

        // Destination address (network byte order) then host port (low 16 bits
        // of a 32-bit register).
        self.write_mem(Self::stream_reg(channel, STREAM_DEST_ADDRESS), &host.octets())?;
        self.write_mem(
            Self::stream_reg(channel, STREAM_DEST_PORT),
            &(port as u32).to_be_bytes(),
        )?;
        self.write_mem(
            Self::stream_reg(channel, STREAM_PACKET_SIZE),
            &packet_size.to_be_bytes(),
        )?;
        self.write_mem(
            Self::stream_reg(channel, STREAM_PACKET_DELAY),
            &packet_delay.to_be_bytes(),
        )?;

        Ok(StreamParams {
            packet_size,
            mtu,
            host,
            port,
        })
    }

    /// Read back the effective `GevSCPSPacketSize` (a camera may clamp the write;
    /// masked to the low 16 bits). A raw READREG, not a cached GenApi node read.
    pub fn get_stream_packet_size(&mut self, channel: u32) -> Result<u32, GvcpError> {
        let addr = Self::stream_reg(channel, STREAM_PACKET_SIZE) as u32;
        Ok(self.read_register(addr)? & STREAM_PACKET_SIZE_MASK)
    }

    /// Overwrite `GevSCPSPacketSize` with a plain `size` (the do-not-fragment
    /// and fire-test flag bits cleared). Used to settle on the probed size and
    /// to restore a clean register after a probe.
    pub fn set_stream_packet_size(&mut self, channel: u32, size: u32) -> Result<(), GvcpError> {
        let addr = Self::stream_reg(channel, STREAM_PACKET_SIZE) as u32;
        self.write_register(addr, encode_stream_packet_size(size, false, false))
    }

    /// Ask the device to emit one test packet of `size` bytes (the full IP
    /// datagram) with the IP do-not-fragment flag set, by writing the
    /// fire-test-packet and do-not-fragment bits into `GevSCPSPacketSize`. The
    /// device sends it to the currently configured `GevSCDA`/`GevSCPHostPort`,
    /// so those must be set (and control held) first. The fire bit is
    /// self-clearing. Used by the auto packet-size probe.
    pub fn fire_test_packet(&mut self, channel: u32, size: u32) -> Result<(), GvcpError> {
        let addr = Self::stream_reg(channel, STREAM_PACKET_SIZE) as u32;
        self.write_register(addr, encode_stream_packet_size(size, true, true))
    }
}

/// Encode a PACKETRESEND payload. GigE Vision 1.x layout, 12 bytes:
/// `stream_channel(2) | block_id(2) | first_packet_id(4) | last_packet_id(4)`
/// (packet ids are 24-bit in the 32-bit fields). Extended-ID layout (2.0, sent
/// with `FLAG_EXTENDED_IDS`), 20 bytes: `stream_channel(2) | reserved(2) |
/// first_packet_id(4) | last_packet_id(4) | block_id(8)`. Both as aravis
/// encodes them (`arv_gvcp_packet_new_packet_resend_cmd`).
pub fn encode_packet_resend(channel: u16, block_id: u64, first: u32, last: u32, extended: bool) -> Vec<u8> {
    let mut p = Vec::with_capacity(20);
    p.extend_from_slice(&channel.to_be_bytes());
    if extended {
        p.extend_from_slice(&[0, 0]);
        p.extend_from_slice(&first.to_be_bytes());
        p.extend_from_slice(&last.to_be_bytes());
        p.extend_from_slice(&block_id.to_be_bytes());
    } else {
        p.extend_from_slice(&(block_id as u16).to_be_bytes());
        p.extend_from_slice(&first.to_be_bytes());
        p.extend_from_slice(&last.to_be_bytes());
    }
    p
}

/// Fire-and-forget PACKETRESEND for the receive thread: a handle on the
/// control socket (the camera honors the command only from the port holding
/// control privilege) with its own request-id sequence. GigE Vision 1.x
/// devices never acknowledge PACKETRESEND and this never asks for one, so it
/// cannot collide with the control thread's transactions; a stray ack from a
/// device that answers anyway is absorbed by the next transaction's id check.
pub struct ResendSender {
    socket: UdpSocket,
    channel: u16,
    request_id: u16,
}

impl ResendSender {
    /// A sender toward `addr` on a fresh socket, for tests and tools without
    /// a [`Device`]; a camera ignores it unless that socket holds control.
    #[allow(dead_code)]
    pub fn to(addr: SocketAddr, channel: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0))?;
        socket.connect(addr)?;
        Ok(Self { socket, channel, request_id: 0x8000 })
    }

    /// Ask for payload packets `first..=last` (1-based) of `block_id`. The
    /// extended layout is required once block ids exceed 16 bits.
    pub fn request(&mut self, block_id: u64, first: u32, last: u32, extended: bool) -> io::Result<()> {
        let payload = encode_packet_resend(self.channel, block_id, first, last, extended);
        let flags = if extended { FLAG_EXTENDED_IDS } else { 0 };
        let id = self.request_id;
        self.request_id = self.request_id.wrapping_add(1).max(1);
        self.socket.send(&encode_command(flags, PACKET_RESEND_CMD, id, &payload)).map(|_| ())
    }
}

/// A decoded PACKETRESEND command (tests).
#[cfg(test)]
pub(crate) struct ResendRequest {
    pub request_id: u16,
    pub block_id: u64,
    pub first: u32,
    pub last: u32,
    pub extended: bool,
}

#[cfg(test)]
pub(crate) fn decode_packet_resend(pkt: &[u8]) -> Option<ResendRequest> {
    if pkt.len() < HEADER_SIZE || pkt[0] != CMD_KEY {
        return None;
    }
    if u16::from_be_bytes([pkt[2], pkt[3]]) != PACKET_RESEND_CMD {
        return None;
    }
    let len = u16::from_be_bytes([pkt[4], pkt[5]]) as usize;
    let request_id = u16::from_be_bytes([pkt[6], pkt[7]]);
    let p = &pkt[HEADER_SIZE..];
    if p.len() != len {
        return None;
    }
    let extended = pkt[1] & FLAG_EXTENDED_IDS != 0;
    let u32_at = |o: usize| u32::from_be_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
    let (block_id, first, last) = match (extended, len) {
        (true, 20) => (u64::from_be_bytes(p[12..20].try_into().ok()?), u32_at(4), u32_at(8)),
        (false, 12) => (u16::from_be_bytes([p[2], p[3]]) as u64, u32_at(4), u32_at(8)),
        _ => return None,
    };
    Some(ResendRequest { request_id, block_id, first, last, extended })
}

enum RecvOutcome {
    Received(usize),
    Io(io::Error),
    TimedOut,
}

// ── Discovery ─────────────────────────────────────────────────────────────--

/// Find every camera this host can see: a broadcast DISCOVERY on every IPv4
/// interface, plus a unicast DISCOVERY to each address of the /24 around
/// every private, non-default-route interface (see [`sweep_send`]). Collects
/// for `timeout` and dedupes by MAC.
pub fn discover_all(timeout: Duration) -> Vec<DeviceInfo> {
    let request_id: u16 = 0x0100;
    let broadcast = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, DISCOVERY_CMD, request_id, &[]);
    let unicast = encode_command(FLAG_ACK_REQUIRED, DISCOVERY_CMD, request_id, &[]);
    let mut sockets = broadcast_send(&broadcast);
    sockets.extend(sweep_send(&unicast));
    let mut seen: std::collections::HashMap<[u8; 6], DeviceInfo> = std::collections::HashMap::new();
    collect_replies(&sockets, Instant::now() + timeout, |buf| {
        if let Some(info) = parse_discovery_ack(buf, request_id) {
            seen.entry(info.mac).or_insert(info);
        }
        false
    });
    let mut devices: Vec<_> = seen.into_values().collect();
    devices.sort_by_key(|d| d.ip);
    devices
}

/// Discover one device by unicast: send DISCOVERY to `ip` directly and wait
/// up to `timeout` for its acknowledgement. Hosts whose endpoint security
/// drops broadcast replies can still identify a camera they can route to.
pub fn discover_unicast(ip: Ipv4Addr, port: u16, timeout: Duration) -> Option<DeviceInfo> {
    let request_id: u16 = 0x0101;
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).ok()?;
    let packet = encode_command(FLAG_ACK_REQUIRED, DISCOVERY_CMD, request_id, &[]);
    socket.send_to(&packet, (ip, port)).ok()?;
    socket.set_read_timeout(Some(timeout)).ok()?;
    let mut buf = vec![0u8; 2048];
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match socket.recv_from(&mut buf) {
            Ok((len, _)) => {
                if let Some(info) = parse_discovery_ack(&buf[..len], request_id) {
                    return Some(info);
                }
            }
            Err(e) if matches!(e.kind(), ErrorKind::ConnectionReset | ErrorKind::Interrupted) => continue,
            Err(_) => return None,
        }
    }
    None
}

/// The 56-byte FORCEIP payload: reserved(2) | MAC(6) | reserved(12) |
/// IP(4) | reserved(12) | subnet mask(4) | reserved(12) | gateway(4).
pub fn encode_forceip_payload(mac: [u8; 6], ip: Ipv4Addr, subnet: Ipv4Addr, gateway: Ipv4Addr) -> [u8; 56] {
    let mut p = [0u8; 56];
    p[2..8].copy_from_slice(&mac);
    p[20..24].copy_from_slice(&ip.octets());
    p[36..40].copy_from_slice(&subnet.octets());
    p[52..56].copy_from_slice(&gateway.octets());
    p
}

/// Broadcast FORCEIP: tell the device with `mac` to adopt `ip`/`subnet`/
/// `gateway` until its next power cycle. Waits up to `timeout` for the
/// acknowledgement: `Ok(true)` acknowledged, `Ok(false)` no reply (many
/// cameras apply it silently, or answer from an address this host can't
/// hear), `Err` on a send failure or a rejecting status. To make the address
/// stick, open the device at its new address and use
/// [`Device::write_persistent_ip`] + [`Device::enable_persistent_ip`].
pub fn force_ip(
    mac: [u8; 6],
    ip: Ipv4Addr,
    subnet: Ipv4Addr,
    gateway: Ipv4Addr,
    timeout: Duration,
) -> Result<bool, GvcpError> {
    let request_id: u16 = 0x0200;
    let payload = encode_forceip_payload(mac, ip, subnet, gateway);
    let packet = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, FORCEIP_CMD, request_id, &payload);
    let sockets = broadcast_send(&packet);
    if sockets.is_empty() {
        return Err(GvcpError::Protocol("FORCEIP could not be sent on any interface".into()));
    }
    let mut result: Result<bool, GvcpError> = Ok(false);
    collect_replies(&sockets, Instant::now() + timeout, |buf| {
        let Ok(ack) = decode_ack(buf) else { return false };
        if ack.command != FORCEIP_ACK || ack.request_id != request_id {
            return false; // discovery replies and other chatter share the port
        }
        result = match ack.status {
            Status::Success => Ok(true),
            other => Err(GvcpError::Status(other)),
        };
        true
    });
    result
}

/// Send a broadcast GVCP command on every IPv4 interface and return the
/// sockets it went out on, so their replies can be collected.
///
/// One socket per interface, bound to that interface's own address: on
/// macOS and the BSDs a limited broadcast from a socket bound to 0.0.0.0
/// leaves only through the default-route interface, so a camera on a
/// secondary NIC never hears it. Each interface gets the limited broadcast
/// and its directed broadcast (a camera whose subnet mask differs from the
/// host's still accepts the former); loopback gets unicast to itself.
fn broadcast_send(packet: &[u8]) -> Vec<UdpSocket> {
    let mut sockets = Vec::new();
    for iface in nic::ipv4_interfaces() {
        let Ok(socket) = bind_broadcast_socket(&iface) else { continue };
        let dests: &[Ipv4Addr] = if iface.is_loopback {
            &[iface.ip]
        } else {
            &[Ipv4Addr::BROADCAST, nic::directed_broadcast(iface.ip, iface.netmask)]
        };
        let sent = dests.iter().any(|d| socket.send_to(packet, (*d, GVCP_PORT)).is_ok());
        if sent && socket.set_nonblocking(true).is_ok() {
            sockets.push(socket);
        }
    }
    sockets
}

/// Send a unicast GVCP command to every address in the /24 containing each
/// private, non-loopback interface that is not the default route, and return
/// the sockets it went out on.
///
/// Why: endpoint-security filters (the corporate VPN/EDR on the bench Mac)
/// admit an inbound datagram only when it matches an outbound flow, and a
/// camera's reply to a broadcast never matches a flow addressed to
/// 255.255.255.255 — while a reply to a unicast request does. A camera link
/// is a private subnet off the default route, so sweeping its /24 (254 tiny
/// datagrams) finds the camera without touching the corporate network.
fn sweep_send(packet: &[u8]) -> Vec<UdpSocket> {
    let default_src = nic::local_ipv4_towards(Ipv4Addr::new(1, 1, 1, 1), 53);
    let mut sockets = Vec::new();
    for iface in nic::ipv4_interfaces() {
        if iface.is_loopback || iface.ip == default_src || !iface.ip.is_private() && !iface.ip.is_link_local() {
            continue;
        }
        let Ok(socket) = bind_broadcast_socket(&iface) else { continue };
        let base = u32::from(iface.ip) & 0xFFFF_FF00;
        let mut sent = false;
        for host in 1..=254u32 {
            let ip = Ipv4Addr::from(base | host);
            if ip != iface.ip {
                sent |= socket.send_to(packet, (ip, GVCP_PORT)).is_ok();
            }
        }
        if sent && socket.set_nonblocking(true).is_ok() {
            sockets.push(socket);
        }
    }
    sockets
}

/// Poll `sockets` round-robin until `deadline`, handing every datagram to
/// `on_reply`; stop early when it returns true.
fn collect_replies(sockets: &[UdpSocket], deadline: Instant, mut on_reply: impl FnMut(&[u8]) -> bool) {
    let mut buf = vec![0u8; 2048];
    while Instant::now() < deadline {
        for socket in sockets {
            loop {
                match socket.recv_from(&mut buf) {
                    Ok((len, _src)) => {
                        if on_reply(&buf[..len]) {
                            return;
                        }
                    }
                    Err(e) if e.kind() == ErrorKind::WouldBlock => break,
                    // Windows: an ICMP unreachable from the broadcast surfaces here.
                    Err(e) if matches!(e.kind(), ErrorKind::ConnectionReset | ErrorKind::Interrupted) => continue,
                    Err(_) => break,
                }
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}

/// A broadcast-capable socket pinned to one interface. Binding the source
/// address is not enough on macOS/BSD: the route for 255.255.255.255 still
/// resolves to the default interface, so the datagram would leave on the
/// wrong NIC with a foreign source address. IP_BOUND_IF (Apple, illumos) /
/// SO_BINDTODEVICE (Linux) pins it; Windows picks the interface from the
/// bound address on its own. Best-effort: a failure leaves the plain bind.
fn bind_broadcast_socket(iface: &nic::Ipv4Iface) -> io::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    socket.set_broadcast(true)?;
    #[cfg(any(target_vendor = "apple", target_os = "illumos", target_os = "solaris"))]
    if let Some(index) = iface.index.and_then(std::num::NonZeroU32::new) {
        let _ = socket.bind_device_by_index_v4(Some(index));
    }
    #[cfg(any(target_os = "linux", target_os = "android"))]
    {
        let _ = socket.bind_device(Some(iface.name.as_bytes()));
    }
    socket.bind(&SocketAddr::new(IpAddr::V4(iface.ip), 0).into())?;
    let socket: UdpSocket = socket.into();
    socket.set_nonblocking(false)?;
    Ok(socket)
}

/// Parse a Discovery ACK datagram. Returns `None` for anything that is not a
/// well-formed Discovery ACK for our request (foreign traffic is expected on a
/// broadcast socket).
fn parse_discovery_ack(buf: &[u8], expected_request: u16) -> Option<DeviceInfo> {
    if buf.len() < HEADER_SIZE + 40 {
        return None;
    }
    let status = u16::from_be_bytes([buf[0], buf[1]]);
    let command = u16::from_be_bytes([buf[2], buf[3]]);
    let request_id = u16::from_be_bytes([buf[6], buf[7]]);
    if command != DISCOVERY_ACK || status != 0 || request_id != expected_request {
        return None;
    }
    let p = &buf[HEADER_SIZE..];
    // Discovery ACK payload offsets (GigE Vision §7-4 / Wireshark
    // dissect_discovery_ack): MAC at 10, current IP at 36, manufacturer at 72,
    // model at 104.
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&p[10..16]);
    let ip = Ipv4Addr::new(p[36], p[37], p[38], p[39]);
    let manufacturer = fixed_string(&p, 72, 32);
    let model = fixed_string(&p, 104, 32);
    Some(DeviceInfo {
        ip,
        mac,
        manufacturer,
        model,
    })
}

fn fixed_string(payload: &[u8], at: usize, len: usize) -> Option<String> {
    let end = (at + len).min(payload.len());
    if at >= end {
        return None;
    }
    let bytes = &payload[at..end];
    let nul = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    let s = String::from_utf8_lossy(&bytes[..nul]).trim().to_string();
    if s.is_empty() {
        None
    } else {
        Some(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forceip_payload_sits_at_the_specified_offsets() {
        let p = encode_forceip_payload(
            [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe],
            Ipv4Addr::new(192, 168, 0, 10),
            Ipv4Addr::new(255, 255, 255, 0),
            Ipv4Addr::new(192, 168, 0, 1),
        );
        assert_eq!(p.len(), 56);
        assert_eq!(&p[0..2], &[0, 0]);
        assert_eq!(&p[2..8], &[0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe]);
        assert!(p[8..20].iter().all(|&b| b == 0));
        assert_eq!(&p[20..24], &[192, 168, 0, 10]);
        assert!(p[24..36].iter().all(|&b| b == 0));
        assert_eq!(&p[36..40], &[255, 255, 255, 0]);
        assert!(p[40..52].iter().all(|&b| b == 0));
        assert_eq!(&p[52..56], &[192, 168, 0, 1]);
        // And the command header around it.
        let pkt = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, FORCEIP_CMD, 0x0200, &p);
        assert_eq!(&pkt[..8], &[0x42, 0x11, 0x00, 0x04, 0x00, 56, 0x02, 0x00]);
    }

    #[test]
    fn status_table_matches_the_specification() {
        // Literal values from the spec, not derived from `to_raw`.
        for (raw, expected, name) in [
            (0x0000u16, Status::Success, "SUCCESS"),
            (0x8001, Status::NotImplemented, "NOT_IMPLEMENTED"),
            (0x8002, Status::InvalidParameter, "INVALID_PARAMETER"),
            (0x8003, Status::InvalidAddress, "INVALID_ADDRESS"),
            (0x8004, Status::WriteProtect, "WRITE_PROTECT"),
            (0x8005, Status::BadAlignment, "BAD_ALIGNMENT"),
            (0x8006, Status::AccessDenied, "ACCESS_DENIED"),
            (0x8007, Status::Busy, "BUSY"),
            (0x8008, Status::LocalProblem, "LOCAL_PROBLEM"),
            (0x8009, Status::MsgMismatch, "MSG_MISMATCH"),
            (0x800A, Status::InvalidProtocol, "INVALID_PROTOCOL"),
            (0x800B, Status::NoMsg, "NO_MSG"),
            (0x800C, Status::PacketUnavailable, "PACKET_UNAVAILABLE"),
            (0x800D, Status::DataOverrun, "DATA_OVERRUN"),
            (0x800E, Status::InvalidHeader, "INVALID_HEADER"),
            (0x8FFF, Status::GenericError, "ERROR"),
        ] {
            assert_eq!(Status::from_raw(raw), expected, "decode {raw:#06x}");
            assert_eq!(expected.to_raw(), raw, "encode {name}");
            assert_eq!(expected.name(), name);
        }
    }

    #[test]
    fn only_busy_is_retryable() {
        assert!(Status::Busy.is_retryable());
        assert!(!Status::WriteProtect.is_retryable());
        assert!(!Status::AccessDenied.is_retryable());
        assert!(!Status::Success.is_retryable());
        // An unknown code round-trips.
        assert_eq!(Status::from_raw(0xABCD), Status::Unknown(0xABCD));
        assert_eq!(Status::Unknown(0xABCD).to_raw(), 0xABCD);
    }

    /// A command header written from the spec field table, indexed by offset.
    #[test]
    fn command_header_sits_at_the_specified_offsets() {
        let b = encode_command(FLAG_ACK_REQUIRED, READMEM_CMD, 0x00AB, &[0, 0, 0x0D, 0x04, 0, 0, 0, 4]);
        assert_eq!(b[0], CMD_KEY, "command key at 0");
        assert_eq!(b[1], 0x01, "ACK_REQUIRED flag at 1");
        assert_eq!(u16::from_be_bytes([b[2], b[3]]), 0x0084, "READMEM at 2");
        assert_eq!(u16::from_be_bytes([b[4], b[5]]), 8, "payload length at 4");
        assert_eq!(u16::from_be_bytes([b[6], b[7]]), 0x00AB, "request id at 6");
        assert_eq!(b.len(), HEADER_SIZE + 8);
    }

    /// A READREG_ACK returning 0x0000_3EF2, byte for byte (viva-gencp golden).
    #[test]
    fn ack_fields_sit_at_the_specified_offsets() {
        let bytes = [
            0x00, 0x00, // status: SUCCESS
            0x00, 0x81, // READREG_ACK
            0x00, 0x04, // length
            0x12, 0x34, // request id
            0x00, 0x00, 0x3E, 0xF2, // payload
        ];
        let ack = decode_ack(&bytes).expect("decode");
        assert_eq!(ack.status, Status::Success);
        assert_eq!(ack.command, READREG_ACK);
        assert_eq!(ack.request_id, 0x1234);
        assert_eq!(ack.payload, &[0x00, 0x00, 0x3E, 0xF2]);
    }

    /// A PENDING_ACK reports SUCCESS status, so the status cannot be the signal;
    /// the command id 0x0089 is. Spec field table: header, reserved(2), ms(2).
    #[test]
    fn pending_ack_is_a_command_id_not_a_status() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&0u16.to_be_bytes()); // status: success
        buf.extend_from_slice(&PENDING_ACK.to_be_bytes());
        buf.extend_from_slice(&4u16.to_be_bytes()); // length
        buf.extend_from_slice(&0x1234u16.to_be_bytes());
        buf.extend_from_slice(&0u16.to_be_bytes()); // reserved
        buf.extend_from_slice(&750u16.to_be_bytes());
        let (id, wait) = parse_pending_ack(&buf).expect("pending");
        assert_eq!(id, 0x1234);
        assert_eq!(wait, Duration::from_millis(750));

        // Junk in the reserved field must not be read as part of the time.
        let mut junk = buf.clone();
        junk[8..10].copy_from_slice(&0xDEADu16.to_be_bytes());
        assert_eq!(parse_pending_ack(&junk).unwrap().1.as_millis(), 750);

        // A real READREG ack is not a pending ack.
        let mut readreg = Vec::new();
        readreg.extend_from_slice(&0u16.to_be_bytes());
        readreg.extend_from_slice(&READREG_ACK.to_be_bytes());
        readreg.extend_from_slice(&4u16.to_be_bytes());
        readreg.extend_from_slice(&0x1234u16.to_be_bytes());
        readreg.extend_from_slice(&0u32.to_be_bytes());
        assert!(parse_pending_ack(&readreg).is_none());
    }

    /// A Discovery ACK payload written from the spec field table (JAI-shaped
    /// fixture from viva-gige): MAC at 10, IP at 36, manufacturer at 72, model
    /// at 104.
    #[test]
    fn discovery_ack_reads_the_spec_offsets() {
        let mut p = vec![0u8; 248];
        p[10..16].copy_from_slice(&[0x00, 0x0C, 0xDF, 0x06, 0x5B, 0x2F]);
        p[36..40].copy_from_slice(&[169, 254, 78, 62]);
        p[72..72 + 15].copy_from_slice(b"JAI Corporation");
        p[104..104 + 17].copy_from_slice(b"FS-3200T-10GE-NNC");

        let mut datagram = Vec::new();
        datagram.extend_from_slice(&0u16.to_be_bytes()); // status
        datagram.extend_from_slice(&DISCOVERY_ACK.to_be_bytes());
        datagram.extend_from_slice(&(p.len() as u16).to_be_bytes());
        datagram.extend_from_slice(&0x0100u16.to_be_bytes()); // request id
        datagram.extend_from_slice(&p);

        let info = parse_discovery_ack(&datagram, 0x0100).expect("parse");
        assert_eq!(info.mac, [0x00, 0x0C, 0xDF, 0x06, 0x5B, 0x2F]);
        assert_eq!(info.ip, Ipv4Addr::new(169, 254, 78, 62));
        assert_eq!(info.manufacturer.as_deref(), Some("JAI Corporation"));
        assert_eq!(info.model.as_deref(), Some("FS-3200T-10GE-NNC"));

        // Foreign traffic is ignored, not fatal.
        assert!(parse_discovery_ack(&datagram, 0x0999).is_none()); // wrong request
        let mut wrong_status = datagram.clone();
        wrong_status[0..2].copy_from_slice(&0x8002u16.to_be_bytes());
        assert!(parse_discovery_ack(&wrong_status, 0x0100).is_none());
        assert!(parse_discovery_ack(&[0u8; 4], 0x0100).is_none());
    }
}

#[cfg(test)]
mod resend_tests {
    use super::*;

    #[test]
    fn packet_resend_payloads_match_the_aravis_layouts() {
        assert_eq!(encode_packet_resend(0, 0x1234, 5, 9, false), [0, 0, 0x12, 0x34, 0, 0, 0, 5, 0, 0, 0, 9]);
        assert_eq!(
            encode_packet_resend(0, 0x0001_0002_0003_0004, 5, 9, true),
            [0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 9, 0, 1, 0, 2, 0, 3, 0, 4]
        );
    }

    #[test]
    fn resend_commands_round_trip_and_carry_the_extended_flag() {
        let cam = UdpSocket::bind("127.0.0.1:0").unwrap();
        cam.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
        let mut s = ResendSender::to(cam.local_addr().unwrap(), 0).unwrap();
        s.request(7, 2, 3, false).unwrap();
        s.request(70_000, 4, 4, true).unwrap();
        let mut b = [0u8; 64];
        let n = cam.recv(&mut b).unwrap();
        let r = decode_packet_resend(&b[..n]).expect("standard request");
        assert_eq!((r.block_id, r.first, r.last, r.extended), (7, 2, 3, false));
        assert_eq!(b[1], 0, "no flags on a standard request");
        let n = cam.recv(&mut b).unwrap();
        let r2 = decode_packet_resend(&b[..n]).expect("extended request");
        assert_eq!((r2.block_id, r2.first, r2.last, r2.extended), (70_000, 4, 4, true));
        assert_eq!(b[1], FLAG_EXTENDED_IDS);
        assert_ne!(r.request_id, r2.request_id);
        assert_ne!(r.request_id, 0);
    }
}

#[cfg(test)]
mod packet_probe_tests {
    use super::*;

    /// The SCPS register encoding against the aravis `arvgvcpprivate.h` masks:
    /// size in the low 16 bits, do-not-fragment = `1 << 30`, fire-test = `1 << 31`.
    #[test]
    fn scps_encoding_matches_the_aravis_masks() {
        // Plain size, no flags: exactly the 16-bit size.
        assert_eq!(encode_stream_packet_size(9000, false, false), 9000);
        assert_eq!(encode_stream_packet_size(1500, false, false), 1500);
        // The size is masked to 16 bits; high junk never bleeds into the flags.
        assert_eq!(encode_stream_packet_size(0x1_2345, false, false), 0x2345);
        // Flag bits, verified as literal bit positions (not derived from consts).
        assert_eq!(encode_stream_packet_size(0, true, false), 1 << 30);
        assert_eq!(encode_stream_packet_size(0, false, true), 1 << 31);
        // A fired do-not-fragment test packet of 9000: size | DNF | FIRE.
        assert_eq!(
            encode_stream_packet_size(9000, true, true),
            9000 | (1 << 30) | (1 << 31)
        );
        // And the constants themselves are the aravis values.
        assert_eq!(STREAM_PACKET_SIZE_MASK, 0x0000_ffff);
        assert_eq!(SCPS_DO_NOT_FRAGMENT, 1 << 30);
        assert_eq!(SCPS_FIRE_TEST_PACKET, 1 << 31);
    }

    /// A monotone predicate with a threshold: the largest carried size is the
    /// threshold itself, found without ever probing above `ceil` or below
    /// `floor`.
    #[test]
    fn search_finds_the_threshold() {
        for threshold in [576u32, 1500, 4000, 8999, 9000] {
            let mut probed = Vec::new();
            let got = largest_carried_packet_size(576, 9000, |s| {
                probed.push(s);
                s <= threshold
            });
            assert_eq!(got, Some(threshold.min(9000)), "threshold {threshold}");
            assert!(probed.iter().all(|&s| (576..=9000).contains(&s)), "stayed in range: {probed:?}");
        }
    }

    /// The current (ceil) size working is the common case and costs one probe.
    #[test]
    fn search_returns_ceil_on_the_first_probe_when_it_works() {
        let mut calls = 0usize;
        let got = largest_carried_packet_size(576, 9000, |_| {
            calls += 1;
            true
        });
        assert_eq!(got, Some(9000));
        assert_eq!(calls, 1, "no further probing once the ceiling carries");
    }

    /// A camera that answers no test packet at all yields `None` (fall back),
    /// after at most the ceil-then-floor pair of probes.
    #[test]
    fn search_gives_up_when_nothing_arrives() {
        let mut calls = 0usize;
        let got = largest_carried_packet_size(576, 9000, |_| {
            calls += 1;
            false
        });
        assert_eq!(got, None);
        assert_eq!(calls, 2, "ceil then floor, then give up");
    }

    /// Only the floor carries: it is returned, and a degenerate range is safe.
    #[test]
    fn search_handles_floor_only_and_empty_ranges() {
        assert_eq!(largest_carried_packet_size(576, 9000, |s| s <= 576), Some(576));
        // floor == ceil: a single working size.
        assert_eq!(largest_carried_packet_size(1500, 1500, |_| true), Some(1500));
        assert_eq!(largest_carried_packet_size(1500, 1500, |_| false), None);
        // ceil below floor: nothing to probe.
        assert_eq!(largest_carried_packet_size(1500, 576, |_| true), None);
    }
}
