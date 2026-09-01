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

/// GenCP / GVCP acknowledgement header size.
const HEADER_SIZE: usize = 8;

// ── Opcodes ─────────────────────────────────────────────────────────────────
const DISCOVERY_CMD: u16 = 0x0002;
const DISCOVERY_ACK: u16 = 0x0003;
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

// ── Bootstrap registers ───────────────────────────────────────────────────--
/// Control Channel Privilege.
pub const CCP_REGISTER: u32 = 0x0a00;
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
/// (a device may leave the do-not-fragment bit set).
const STREAM_PACKET_SIZE_MASK: u32 = 0xFFFF;

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
}

enum RecvOutcome {
    Received(usize),
    Io(io::Error),
    TimedOut,
}

// ── Discovery ─────────────────────────────────────────────────────────────--

/// Broadcast a GVCP discovery command on every IPv4 interface plus the limited
/// broadcast, collect for `timeout`, and dedupe by MAC.
pub fn discover_all(timeout: Duration) -> Vec<DeviceInfo> {
    let request_id: u16 = 0x0100;
    let socket = match make_broadcast_socket() {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };
    let packet = encode_command(FLAG_ACK_REQUIRED | FLAG_BROADCAST, DISCOVERY_CMD, request_id, &[]);

    // Send to the limited broadcast and each interface's directed broadcast (a
    // loopback interface only accepts unicast to itself).
    let _ = socket.send_to(&packet, (Ipv4Addr::BROADCAST, GVCP_PORT));
    for iface in nic::ipv4_interfaces() {
        let dest = if iface.is_loopback {
            iface.ip
        } else {
            nic::directed_broadcast(iface.ip, iface.netmask)
        };
        let _ = socket.send_to(&packet, (dest, GVCP_PORT));
    }

    let mut seen: std::collections::HashMap<[u8; 6], DeviceInfo> = std::collections::HashMap::new();
    let deadline = Instant::now() + timeout;
    let mut buf = vec![0u8; 2048];
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            break;
        }
        if socket.set_read_timeout(Some(remaining)).is_err() {
            break;
        }
        match socket.recv_from(&mut buf) {
            Ok((len, _src)) => {
                if let Some(info) = parse_discovery_ack(&buf[..len], request_id) {
                    seen.entry(info.mac).or_insert(info);
                }
            }
            Err(e) => match e.kind() {
                ErrorKind::WouldBlock | ErrorKind::TimedOut => break,
                // Windows: an ICMP unreachable from the broadcast surfaces here.
                ErrorKind::ConnectionReset | ErrorKind::Interrupted => continue,
                _ => break,
            },
        }
    }
    let mut devices: Vec<_> = seen.into_values().collect();
    devices.sort_by_key(|d| d.ip);
    devices
}

fn make_broadcast_socket() -> io::Result<UdpSocket> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
    socket.set_reuse_address(true)?;
    socket.set_broadcast(true)?;
    socket.bind(&SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0).into())?;
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
