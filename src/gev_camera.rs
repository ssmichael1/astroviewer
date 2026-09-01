//! GigE Vision (GEV) camera source — pure Rust, no C/system dependencies.
//!
//! Transport is provided by the app-owned [`crate::gige`] module (GVCP
//! discovery + control, GVSP stream parsing and reassembly, NIC/socket
//! helpers) — a synchronous, std-socket implementation with no async runtime.
//! GenICam feature access (Exposure, Gain, Width/Height, PixelFormat,
//! AcquisitionStart/Stop, …) is provided by `cameleon-genapi`, which parses the
//! camera's GenICam XML and interprets its feature nodes; its register
//! reads/writes are bridged onto the GVCP [`Device`] via a small synchronous
//! adapter.
//!
//! This module owns a self-contained control thread that services GenICam
//! commands and heartbeats, plus a dedicated receive thread that drains the
//! GVSP socket and reassembles frames. The control thread decodes each complete
//! raw frame into [`FrameData`] on the shared `frame_tx` channel. Every GVCP
//! call is a direct blocking transaction — no `block_on`, no tokio.

use std::io::ErrorKind;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{bounded, Receiver, Sender};

use cameleon_genapi::elem_type::{IntegerRepresentation, Visibility};
use cameleon_genapi::interface::ICategoryKind;
use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeId, NodeStore};
use cameleon_genapi::{GenApiError, ValueCtxt};

use crate::gige::gvcp::{self, Device, DeviceInfo};
use crate::gige::gvsp::{self, FrameAssembly, GvspPacket};
use crate::gige::nic::{self, Iface};

use crate::{FrameData, LogEntry};

/// GVCP Control Channel Privilege register — re-read periodically as a heartbeat
/// so the camera does not reclaim control from us.
const CCP_REGISTER: u32 = 0x0a00;
/// Bootstrap "First URL" register pointing at the on-device GenICam XML.
const FIRST_URL_REGISTER: u64 = 0x0200;
/// Heartbeat period. GigE devices default to a ~3 s heartbeat timeout.
const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(1000);
/// How often to re-read non-writable (telemetry) feature values so the UI
/// shows live temperature, status, etc.
const TELEMETRY_INTERVAL: Duration = Duration::from_secs(2);
/// Stream channel index (single-channel cameras use 0).
const STREAM_CHANNEL: u32 = 0;
/// How long to wait for stream packets before returning to service commands.
const POLL_TIMEOUT: Duration = Duration::from_millis(200);
/// How long an in-flight frame may take to reassemble before being abandoned.
const FRAME_DEADLINE: Duration = Duration::from_millis(1000);
/// Stream channel 0 source-port register (`GevSCSP`): the UDP port the camera
/// transmits GVSP from once the channel is open (0 = unspecified).
const SCSP_REGISTER: u32 = 0x0d1c;
/// Bytes of a GVSP data packet that are not image payload: IPv4 (20), UDP (8)
/// and the GVSP header (8). `GevSCPSPacketSize` is the *full IP datagram*
/// size, so the image bytes per packet (the stride reassembly places packets
/// at) is `packet_size` minus 36. viva-gige 0.2 reported the UDP payload
/// instead, so this used to subtract 8; that silent semantic change truncated
/// every frame.
const GVSP_PACKET_OVERHEAD: u32 = 20 + 8 + 8;
/// How long after AcquisitionStart to wait before warning that no packets (or
/// no complete frame) have arrived, with the likely causes.
const SILENCE_GRACE: Duration = Duration::from_secs(3);
/// GVSP socket receive buffer to request (the OS may grant less).
const RECV_BUFFER_REQUEST: usize = 64 << 20;

// ── Public types ────────────────────────────────────────────────────────────

/// A discovered GigE Vision camera. `id` is a stable, hashable identity (the MAC
/// rendered as hex) used by the app's `CameraSource::Gev(String)`.
#[derive(Clone)]
pub struct GevDeviceInfo {
    pub ip: Ipv4Addr,
    pub model: String,
    pub manufacturer: String,
    pub id: String,
}

impl GevDeviceInfo {
    pub fn display_name(&self) -> String {
        match (self.manufacturer.is_empty(), self.model.is_empty()) {
            (false, false) => format!("{} {}", self.manufacturer, self.model),
            (_, false) => self.model.clone(),
            _ => format!("GigE @ {}", self.ip),
        }
    }
}

/// What kind of GenICam feature a control maps to, controlling how the UI renders it.
/// How a GenICam feature maps to a UI widget.
#[derive(Clone, PartialEq)]
pub enum GevControlKind {
    /// Integer feature with [min, max] (in `value`/`min`/`max`).
    Integer,
    /// Integer feature representing an IPv4 address (network byte order in
    /// `value`); rendered as four octet boxes.
    IpV4,
    /// Integer feature representing a MAC address (lower 48 bits of `value`);
    /// rendered as hex text.
    MacAddr,
    /// Float feature with [min, max] (in `fvalue`/`fmin`/`fmax`).
    Float,
    /// Enumeration: symbolic options; `value` is the selected index.
    Enumeration(Vec<String>),
    /// Boolean feature; `value` is 0/1.
    Boolean,
    /// Command (button); ignores value.
    Command,
    /// Read-only float display (e.g. DeviceTemperature) — no editable range.
    ReadOnly,
}

/// A UI-facing control descriptor built from the camera's GenICam feature tree.
#[derive(Clone)]
pub struct GevControl {
    /// GenICam feature node name (used to set the value back on the camera).
    pub name: String,
    pub display: String,
    /// GenICam category (used to group controls in the UI).
    pub category: String,
    pub kind: GevControlKind,
    pub unit: String,
    pub value: i64,
    pub min: i64,
    pub max: i64,
    pub fvalue: f64,
    pub fmin: f64,
    pub fmax: f64,
    pub writable: bool,
    /// Changing this feature requires stopping/restarting acquisition (e.g.
    /// PixelFormat, Width/Height, binning).
    pub needs_restart: bool,
    /// For IpV4/MacAddr kinds: the camera reports the address byte-swapped
    /// relative to the GigE convention (first octet in the most significant
    /// byte), so the UI must reverse byte order when displaying/composing.
    pub ip_swapped: bool,
}

/// Commands from the UI thread to the capture thread.
pub enum GevCmd {
    /// Set an integer feature by node name.
    SetInt(String, i64),
    /// Set a float feature by node name.
    SetFloat(String, f64),
    /// Set an enumeration feature by node name to a symbolic value.
    SetEnum(String, String),
    /// Set a boolean feature by node name.
    SetBool(String, bool),
    /// Execute a command feature by node name.
    Execute(String),
    Stop,
}

/// Handle to a running GEV capture. Mirrors `camera::CameraHandle`.
pub struct GevHandle {
    pub controls: Vec<GevControl>,
    pub cmd_tx: Sender<GevCmd>,
    /// Refreshed control snapshots pushed by the capture thread (so flipping an
    /// `Auto` toggle live-unlocks its companion value control in the UI).
    pub controls_rx: Receiver<Vec<GevControl>>,
    join_handle: Option<JoinHandle<()>>,
}

impl GevHandle {
    /// Stop acquisition and join the capture thread.
    pub fn stop(&mut self) {
        let _ = self.cmd_tx.send(GevCmd::Stop);
        if let Some(jh) = self.join_handle.take() {
            let _ = jh.join();
        }
    }
}

impl Drop for GevHandle {
    fn drop(&mut self) {
        let _ = self.cmd_tx.send(GevCmd::Stop);
        if let Some(jh) = self.join_handle.take() {
            let _ = jh.join();
        }
    }
}

// ── Discovery ─────────────────────────────────────────────────────────────--

/// Discover GigE Vision cameras on all interfaces. Returns an empty vec on error.
pub fn enumerate() -> Vec<GevDeviceInfo> {
    gvcp::discover_all(Duration::from_millis(500))
        .into_iter()
        .map(device_info_to_gev)
        .collect()
}

fn device_info_to_gev(d: DeviceInfo) -> GevDeviceInfo {
    let id = d
        .mac
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect::<Vec<_>>()
        .join(":");
    GevDeviceInfo {
        ip: d.ip,
        model: d.model.unwrap_or_default(),
        manufacturer: d.manufacturer.unwrap_or_default(),
        id,
    }
}

// ── GenICam node store + value context bundle ───────────────────────────────

/// The parsed GenICam model plus the value/cache context needed to evaluate it.
struct GenApi {
    store: DefaultNodeStore,
    ctxt: ValueCtxt<DefaultValueStore, DefaultCacheStore>,
}

/// Synchronous [`cameleon_genapi::Device`] bridge over the GVCP control channel.
/// Borrows the device owned by the control thread and issues each GenApi
/// register access as a direct blocking GVCP transaction.
struct DeviceBridge<'a> {
    dev: &'a mut Device,
}

impl<'a> cameleon_genapi::Device for DeviceBridge<'a> {
    fn read_mem(
        &mut self,
        address: i64,
        buf: &mut [u8],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let data = self.dev.read_mem(address as u64, buf.len())?;
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(())
    }

    fn write_mem(
        &mut self,
        address: i64,
        data: &[u8],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.dev.write_mem(address as u64, data)?;
        Ok(())
    }
}

/// Features whose change alters frame geometry or encoding, so acquisition
/// must stop/restart for the new GVSP leader to take effect.
const RESTART_FEATURES: &[&str] = &[
    "PixelFormat", "Width", "Height", "OffsetX", "OffsetY",
    "BinningVertical", "BinningHorizontal",
];

/// Whether changing a feature requires an acquisition stop/restart.
fn needs_restart(name: &str) -> bool {
    RESTART_FEATURES.contains(&name)
}

/// A float range is usable for a slider only if finite and not absurdly wide
/// (some GenICam floats report ±1e308 when unbounded).
fn sane_range(lo: f64, hi: f64) -> bool {
    lo.is_finite() && hi.is_finite() && hi > lo && (hi - lo) < 1e12
}

// ── Start ────────────────────────────────────────────────────────────────--

/// Open a GigE camera, configure full-frame mono acquisition, start streaming,
/// and spawn the capture thread.
pub fn start_camera(
    info: &GevDeviceInfo,
    frame_tx: Sender<FrameData>,
    log_tx: Sender<LogEntry>,
) -> anyhow::Result<GevHandle> {
    start_camera_at(
        SocketAddr::new(IpAddr::V4(info.ip), gvcp::GVCP_PORT),
        info.display_name(),
        frame_tx,
        log_tx,
    )
}

/// Open a GigE camera addressed by an explicit GVCP `SocketAddr`. `start_camera`
/// wraps this with the well-known port; tests use it to target a simulator on a
/// non-standard port.
pub(crate) fn start_camera_at(
    gvcp_addr: SocketAddr,
    cam_name: String,
    frame_tx: Sender<FrameData>,
    log_tx: Sender<LogEntry>,
) -> anyhow::Result<GevHandle> {
    let ip = match gvcp_addr.ip() {
        IpAddr::V4(ip) => ip,
        IpAddr::V6(_) => anyhow::bail!("IPv6 GigE addresses are not supported"),
    };

    // Connect + claim exclusive control.
    let mut dev = Device::open(gvcp_addr).map_err(|e| anyhow::anyhow!("GVCP open {gvcp_addr}: {e}"))?;
    dev.claim_control().map_err(|e| anyhow::anyhow!("GVCP claim control: {e}"))?;
    let _ = log_tx.try_send(LogEntry::info(format!("GigE: claimed control of {cam_name}")));

    // Fetch + parse the GenICam XML.
    let mut genapi = match load_genapi(&mut dev) {
        Ok(g) => Some(g),
        Err(e) => {
            let _ = log_tx.try_send(LogEntry::warn(format!(
                "GigE: GenICam XML unavailable ({e}); controls disabled, streaming whatever the camera emits"
            )));
            None
        }
    };

    // Configure mono full-frame acquisition and read back geometry, then build
    // the control list. All GVCP access here is a direct blocking transaction.
    let mut controls = Vec::new();
    if let Some(g) = genapi.as_mut() {
        // best-effort configuration; ignore individual feature failures.
        configure_acquisition(g, &mut dev, &log_tx);
        controls = build_controls(g, &mut dev, ip);
    }

    // Negotiate the GVSP stream channel against our receiving interface. Route
    // the probe over the same GVCP port so a simulator on a loopback port is
    // reached on the interface it actually answers on.
    let iface = Iface::from_ipv4(nic::local_ipv4_towards(ip, gvcp_addr.port())).ok();
    // Bind the GVSP receive socket first, then point the camera at it. Ask for a
    // large receive buffer: a frame arrives as a line-rate burst and the buffer
    // is the only slack the receive thread has. The OS clamps (macOS
    // kern.ipc.maxsockbuf, Linux net.core.rmem_max, Windows grants it); log
    // what we actually got.
    let bind_ip = iface
        .as_ref()
        .and_then(|i| i.ipv4())
        .map(IpAddr::V4)
        .unwrap_or(IpAddr::V4(Ipv4Addr::UNSPECIFIED));
    let (socket, recv_buffer) = nic::bind_gvsp_socket(bind_ip, RECV_BUFFER_REQUEST, POLL_TIMEOUT)?;
    let local_port = socket.local_addr()?.port();

    // Optional cap on the GVSP packet size (bytes, full IP datagram), e.g. to
    // force 1500 on a jumbo-capable NIC whose path can't actually carry jumbo
    // frames. Cameras that can't carry the negotiated size stream nothing.
    let packet_cap: Option<u32> = std::env::var("GEV_PACKET_SIZE")
        .ok()
        .and_then(|s| s.trim().parse().ok());
    let stream_params = dev
        .negotiate_stream(
            STREAM_CHANNEL,
            iface.as_ref().ok_or_else(|| anyhow::anyhow!("no usable network interface for GigE streaming"))?,
            local_port,
            packet_cap,
        )
        .map_err(|e| anyhow::anyhow!("GVSP stream negotiation: {e}"))?;
    // A camera may silently clamp the requested size; the wire follows what the
    // register actually holds, so read it back over raw GVCP (not GenApi, whose
    // cache the raw write bypassed).
    let effective_packet_size = dev
        .get_stream_packet_size(STREAM_CHANNEL)
        .ok()
        .filter(|&v| v > GVSP_PACKET_OVERHEAD)
        .unwrap_or(stream_params.packet_size);
    let packet_payload = gvsp_stride(effective_packet_size);
    let _ = log_tx.try_send(LogEntry::info(format!(
        "GigE: stream to {}:{} (mtu={}, packet_size={} requested / {} effective, {} image bytes per packet, \
         {} MiB socket buffer)",
        stream_params.host, local_port, stream_params.mtu, stream_params.packet_size,
        effective_packet_size, packet_payload, recv_buffer >> 20
    )));
    if effective_packet_size != stream_params.packet_size {
        let _ = log_tx.try_send(LogEntry::warn(format!(
            "GigE: camera clamped GevSCPSPacketSize {} -> {}; following the camera",
            stream_params.packet_size, effective_packet_size
        )));
    }

    // Acquisition is started inside the capture thread once everything is wired.

    let (cmd_tx, cmd_rx) = bounded::<GevCmd>(32);
    let (controls_tx, controls_rx) = bounded::<Vec<GevControl>>(4);

    let snapshot = controls.clone();
    let join_handle = std::thread::Builder::new()
        .name("gev-capture".into())
        .spawn(move || {
            capture_loop(
                dev, socket, genapi, packet_payload, &cam_name, frame_tx, cmd_rx, controls_tx,
                snapshot, log_tx,
            );
        })?;

    Ok(GevHandle {
        controls,
        cmd_tx,
        controls_rx,
        join_handle: Some(join_handle),
    })
}

/// Read the bootstrap First-URL register, resolve the on-device XML location, and
/// parse it into a GenICam model.
fn load_genapi(dev: &mut Device) -> anyhow::Result<GenApi> {
    let raw = dev.read_mem(FIRST_URL_REGISTER, 512).map_err(|e| anyhow::anyhow!("READMEM First-URL: {e}"))?;
    // The register is NUL-terminated; bytes past the terminator are undefined
    // (some cameras pad with 0xFF garbage), so cut at the first NUL.
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    let url = String::from_utf8_lossy(&raw[..end]);
    let url = url.trim();

    // Expected form: "Local:<filename>;<hex addr>;<hex length>", optionally with
    // a "?SchemaVersion=x.y.z" query suffix (GigE Vision 2.x).
    let url_no_query = url.split('?').next().unwrap_or(url);
    let rest = url_no_query
        .strip_prefix("Local:")
        .or_else(|| url_no_query.strip_prefix("local:"))
        .ok_or_else(|| anyhow::anyhow!("unsupported GenICam URL scheme: {url}"))?;
    let mut parts = rest.split(';');
    let filename = parts.next().unwrap_or_default().trim().to_string();
    let addr = parse_hex_field(parts.next(), "address", url)?;
    let len = parse_hex_field(parts.next(), "length", url)? as usize;
    anyhow::ensure!(len > 0 && len < 16 * 1024 * 1024, "implausible GenICam XML length {len}");

    let bytes = read_mem_chunked(dev, addr, len)?;

    let xml = if filename.to_ascii_lowercase().ends_with(".zip") {
        inflate_genicam_zip(&bytes)
            .map_err(|e| anyhow::anyhow!("inflating zipped GenICam XML ({filename}): {e}"))?
    } else {
        String::from_utf8(bytes)?
    };

    let (_reg_desc, store, ctxt) = cameleon_genapi::builder::GenApiBuilder::<
        DefaultNodeStore,
        DefaultValueStore,
        DefaultCacheStore,
    >::default()
        .build(&xml)
        .map_err(|e| anyhow::anyhow!("GenICam XML parse failed: {e}"))?;
    Ok(GenApi { store, ctxt })
}

/// Parse one hex field of a GenICam "Local:" URL, naming the field and quoting
/// the whole URL on failure so a camera's odd format shows up in the log.
fn parse_hex_field(part: Option<&str>, what: &str, url: &str) -> anyhow::Result<u64> {
    let s = part.unwrap_or("0").trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    u64::from_str_radix(s, 16)
        .map_err(|e| anyhow::anyhow!("bad {what} field in GenICam URL {url:?}: {e}"))
}

/// Inflate a zipped GenICam description. Per the GenICam standard the blob is a
/// standard ZIP archive containing a single XML entry; extract the first `.xml`.
fn inflate_genicam_zip(bytes: &[u8]) -> anyhow::Result<String> {
    use std::io::Read;
    let mut archive = zip::ZipArchive::new(std::io::Cursor::new(bytes))?;
    for i in 0..archive.len() {
        let mut entry = archive.by_index(i)?;
        if entry.name().to_ascii_lowercase().ends_with(".xml") {
            let mut s = String::new();
            entry.read_to_string(&mut s)?;
            return Ok(s);
        }
    }
    anyhow::bail!("no .xml entry in zipped GenICam archive")
}

/// Read a memory region larger than a single GenCP block by chunking.
fn read_mem_chunked(dev: &mut Device, addr: u64, len: usize) -> anyhow::Result<Vec<u8>> {
    const CHUNK: usize = 512;
    let mut out = Vec::with_capacity(len);
    let mut off = 0usize;
    while off < len {
        let want = CHUNK.min(len - off);
        // GVCP READMEM requires a 4-byte-aligned count; round up, then keep
        // only the bytes we actually need.
        let req = (want + 3) & !3;
        let part = dev
            .read_mem(addr + off as u64, req)
            .map_err(|e| anyhow::anyhow!("READMEM {req} B @ {:#x}: {e}", addr + off as u64))?;
        out.extend_from_slice(&part[..want.min(part.len())]);
        off += want;
    }
    Ok(out)
}

/// Configure full-frame mono acquisition: set PixelFormat to the widest mono
/// format the camera offers, set Width/Height to max, AcquisitionMode=Continuous.
fn configure_acquisition(g: &mut GenApi, dev: &mut Device, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };

    // Width/Height to their max.
    for dim in ["Width", "Height"] {
        if let Some(nid) = g.store.id_by_name(dim) {
            if let Some(int) = nid.as_iinteger_kind(&g.store) {
                if let Ok(max) = int.max(&mut bridge, &g.store, &mut g.ctxt) {
                    let _ = int.set_value(max, &mut bridge, &g.store, &mut g.ctxt);
                }
            }
        }
    }

    // Prefer a mono pixel format, widest bit depth available. GEV_PIXEL_FORMAT
    // names a symbolic entry to try first (diagnostics; simulators that only
    // emit 8-bit payloads).
    if let Some(nid) = g.store.id_by_name("PixelFormat") {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            let forced = std::env::var("GEV_PIXEL_FORMAT").ok();
            let prefs = ["Mono16", "Mono14", "Mono12", "Mono10", "Mono8"];
            for want in forced.iter().map(String::as_str).chain(prefs) {
                if en.entry_by_symbolic(want, &g.store).is_some()
                    && en.set_entry_by_symbolic(want, &mut bridge, &g.store, &mut g.ctxt).is_ok()
                {
                    let _ = log_tx.try_send(LogEntry::info(format!("GigE: PixelFormat={want}")));
                    break;
                }
            }
        }
    }

    // Continuous acquisition.
    if let Some(nid) = g.store.id_by_name("AcquisitionMode") {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            let _ = en.set_entry_by_symbolic("Continuous", &mut bridge, &g.store, &mut g.ctxt);
        }
    }
    // Free-run: don't wait for an external/software trigger.
    if let Some(nid) = g.store.id_by_name("TriggerMode") {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            let _ = en.set_entry_by_symbolic("Off", &mut bridge, &g.store, &mut g.ctxt);
        }
    }
}

/// Build the UI control list by walking the camera's own GenICam category tree
/// from `Root`, reading each feature live (type, value, range, writability,
/// enum options). Invisible nodes and non-value kinds (ports, registers,
/// plain strings) are skipped; nested categories are flattened under their
/// own display name.
fn build_controls(g: &mut GenApi, dev: &mut Device, cam_ip: Ipv4Addr) -> Vec<GevControl> {
    let mut out = Vec::new();
    let mut b = DeviceBridge { dev };
    let Some(root) = g.store.id_by_name("Root") else { return out };
    walk_category(g, &mut b, root, None, &mut out);
    orient_addresses(&mut out, cam_ip);
    out
}

/// The GigE convention stores an IPv4 address as a 32-bit integer with the
/// first octet in the most significant byte, but some cameras' XMLs read the
/// address registers with the opposite endianness (the Raptor Hawk does).
/// Detect the orientation by comparing what GevCurrentIPAddress reports with
/// the address we are actually talking to, and mark every address-valued
/// control so the UI can compensate.
fn orient_addresses(controls: &mut [GevControl], cam_ip: Ipv4Addr) {
    if cam_ip == Ipv4Addr::UNSPECIFIED {
        return;
    }
    let canonical = u32::from(cam_ip);
    let Some(cur) = controls
        .iter()
        .find(|c| c.kind == GevControlKind::IpV4 && c.name.contains("CurrentIPAddress"))
    else {
        return;
    };
    let raw = cur.value as u32;
    if raw.swap_bytes() == canonical && raw != canonical {
        for c in controls.iter_mut() {
            if matches!(c.kind, GevControlKind::IpV4 | GevControlKind::MacAddr) {
                c.ip_swapped = true;
            }
        }
    }
}

/// Recurse through a category node, appending controls for its features.
fn walk_category(
    g: &mut GenApi,
    b: &mut DeviceBridge<'_>,
    cat: NodeId,
    label: Option<&str>,
    out: &mut Vec<GevControl>,
) {
    let Some(ICategoryKind::Category(n)) = cat.as_icategory_kind(&g.store) else { return };
    let children: Vec<NodeId> = n.p_features().to_vec();
    for nid in children {
        if nid.as_icategory_kind(&g.store).is_some() {
            let name = node_display(g, nid);
            walk_category(g, b, nid, Some(&name), out);
        } else if let Some(c) = control_from_node(g, b, nid, label.unwrap_or("Features")) {
            out.push(c);
        }
    }
}

/// The node's display name, falling back to its raw feature name.
fn node_display(g: &GenApi, nid: NodeId) -> String {
    g.store
        .node_opt(nid)
        .and_then(|n| n.node_base().display_name())
        .unwrap_or_else(|| nid.name(&g.store))
        .to_string()
}

/// Build one UI control from a feature node by reading its live state. Returns
/// None for non-value kinds, invisible features, and features that are neither
/// readable nor writable (not implemented / not available).
fn control_from_node(
    g: &mut GenApi,
    b: &mut DeviceBridge<'_>,
    nid: NodeId,
    category: &str,
) -> Option<GevControl> {
    // Gate node_base() (panics on kinds it doesn't cover, e.g. EnumEntry) on the
    // node being one of the value kinds we render.
    let is_value_kind = nid.as_iboolean_kind(&g.store).is_some()
        || nid.as_ienumeration_kind(&g.store).is_some()
        || nid.as_ifloat_kind(&g.store).is_some()
        || nid.as_iinteger_kind(&g.store).is_some()
        || nid.as_icommand_kind(&g.store).is_some();
    if !is_value_kind {
        return None;
    }
    let nb = g.store.node_opt(nid)?.node_base();
    if nb.visibility() == Visibility::Invisible {
        return None;
    }
    let name = nid.name(&g.store).to_string();
    let display = nb.display_name().unwrap_or(&name).to_string();

    let base = |kind, unit: String, writable: bool| GevControl {
        name: name.clone(),
        display: display.clone(),
        category: category.to_string(),
        kind,
        unit,
        value: 0, min: 0, max: 0,
        fvalue: 0.0, fmin: 0.0, fmax: 0.0,
        writable,
        needs_restart: needs_restart(&name),
        ip_swapped: false,
    };

    // Order matters: boolean/enumeration before integer, since some are both.
    if let Some(bn) = nid.as_iboolean_kind(&g.store) {
        let readable = bn.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = bn.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let v = match bn.value(b, &g.store, &mut g.ctxt) {
            Err(GenApiError::ChunkDataMissing) => return None, // chunk-backed
            v => v.unwrap_or(false),
        };
        let mut c = base(GevControlKind::Boolean, String::new(), writable);
        c.value = v as i64;
        Some(c)
    } else if let Some(en) = nid.as_ienumeration_kind(&g.store) {
        let readable = en.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = en.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let opts: Vec<String> = en.entries(&g.store).iter()
            .filter_map(|e| e.expect_enum_entry(&g.store).ok().map(|x| x.symbolic().to_string()))
            .collect();
        let cur = match en.current_entry(b, &g.store, &mut g.ctxt) {
            Err(GenApiError::ChunkDataMissing) => return None, // chunk-backed
            e => e.ok().and_then(|e| e.expect_enum_entry(&g.store).ok().map(|x| x.symbolic().to_string())),
        };
        let idx = cur.as_ref().and_then(|s| opts.iter().position(|o| o == s)).unwrap_or(0);
        let mut c = base(GevControlKind::Enumeration(opts), String::new(), writable);
        c.value = idx as i64;
        Some(c)
    } else if let Some(f) = nid.as_ifloat_kind(&g.store) {
        let readable = f.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = f.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let unit = f.unit(&g.store).unwrap_or("").to_string();
        let value = match f.value(b, &g.store, &mut g.ctxt) {
            Err(GenApiError::ChunkDataMissing) => return None, // chunk-backed
            v => v.unwrap_or(0.0),
        };
        let fmin = f.min(b, &g.store, &mut g.ctxt).unwrap_or(0.0);
        let fmax = f.max(b, &g.store, &mut g.ctxt).unwrap_or(0.0);
        // Unbounded floats (e.g. DeviceTemperature) → read-only display.
        let kind = if writable && sane_range(fmin, fmax) { GevControlKind::Float } else { GevControlKind::ReadOnly };
        let mut c = base(kind, unit, writable);
        c.fvalue = value; c.fmin = fmin; c.fmax = fmax;
        Some(c)
    } else if let Some(i) = nid.as_iinteger_kind(&g.store) {
        let readable = i.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = i.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let unit = i.unit(&g.store).unwrap_or("").to_string();
        let value = match i.value(b, &g.store, &mut g.ctxt) {
            Err(GenApiError::ChunkDataMissing) => return None, // chunk-backed
            v => v.unwrap_or(0),
        };
        let min = i.min(b, &g.store, &mut g.ctxt).unwrap_or(0);
        let max = i.max(b, &g.store, &mut g.ctxt).unwrap_or(0);
        // Address-valued integers get dedicated rendering. Trust the XML's
        // Representation, with a name heuristic for XMLs that omit it.
        let repr = i.representation(&g.store);
        let kind = if repr == IntegerRepresentation::IpV4Address
            || name.contains("IPAddress") || name.contains("SubnetMask") || name.contains("DefaultGateway")
        {
            GevControlKind::IpV4
        } else if repr == IntegerRepresentation::MacAddress || name.contains("MACAddress") {
            GevControlKind::MacAddr
        } else {
            GevControlKind::Integer
        };
        let mut c = base(kind, unit, writable);
        c.value = value; c.min = min; c.max = max;
        Some(c)
    } else if let Some(cn) = nid.as_icommand_kind(&g.store) {
        let writable = cn.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !writable { return None; }
        Some(base(GevControlKind::Command, String::new(), writable))
    } else {
        None
    }
}

/// Re-read the current values of non-writable (telemetry) controls in place.
/// Returns true if any value changed. Much cheaper than `build_controls`:
/// skips ranges, writability, and all writable features.
fn refresh_telemetry(g: &mut GenApi, dev: &mut Device, controls: &mut [GevControl]) -> bool {
    enum Upd { I(i64), F(f64) }
    let mut b = DeviceBridge { dev };
    let mut changed = false;
    for c in controls.iter_mut().filter(|c| !c.writable) {
        let Some(nid) = g.store.id_by_name(&c.name) else { continue };
        let upd = match &c.kind {
            GevControlKind::Float | GevControlKind::ReadOnly => nid
                .as_ifloat_kind(&g.store)
                .and_then(|f| f.value(&mut b, &g.store, &mut g.ctxt).ok())
                .map(Upd::F),
            GevControlKind::Integer | GevControlKind::IpV4 | GevControlKind::MacAddr => nid
                .as_iinteger_kind(&g.store)
                .and_then(|i| i.value(&mut b, &g.store, &mut g.ctxt).ok())
                .map(Upd::I),
            GevControlKind::Boolean => nid
                .as_iboolean_kind(&g.store)
                .and_then(|bn| bn.value(&mut b, &g.store, &mut g.ctxt).ok())
                .map(|v| Upd::I(v as i64)),
            GevControlKind::Enumeration(opts) => nid
                .as_ienumeration_kind(&g.store)
                .and_then(|en| en.current_entry(&mut b, &g.store, &mut g.ctxt).ok())
                .and_then(|e| e.expect_enum_entry(&g.store).ok())
                .and_then(|x| opts.iter().position(|o| o == x.symbolic()))
                .map(|i| Upd::I(i as i64)),
            GevControlKind::Command => None,
        };
        match upd {
            Some(Upd::F(v)) if v != c.fvalue => { c.fvalue = v; changed = true; }
            Some(Upd::I(v)) if v != c.value => { c.value = v; changed = true; }
            _ => {}
        }
    }
    changed
}

// ── Capture loop ─────────────────────────────────────────────────────────--

/// What the receive thread publishes for the control thread: diagnostics
/// counters and the UDP source port the camera was actually seen streaming
/// from (0 = nothing received yet), which the hole-punch targets.
#[derive(Default)]
struct RxShared {
    packets: AtomicU64,
    completed: AtomicU64,
    src_port: AtomicU32,
}

/// UDP source ports GigE cameras are commonly seen streaming from (the Hawk
/// uses 1051, 1054, …). Sprayed with one-byte punches only while no packet has
/// arrived and the camera reports `GevSCSP` = 0.
const PUNCH_SPRAY_PORTS: std::ops::RangeInclusive<u16> = 1024..=2048;

/// Receive-side state, owned by the receive thread and persisting across poll
/// windows and frames.
struct RxState {
    assembly: Option<FrameAssembly>,
    geom: Option<FrameGeometry>,
    /// Image bytes per GVSP payload packet — the stride reassembly places
    /// packets at. Seeded from the effective `GevSCPSPacketSize`, then
    /// corrected from the first payload packet actually seen on the wire, so a
    /// camera that interprets the register differently still reassembles.
    stride: usize,
    shared: Arc<RxShared>,
    /// Last source port published to `shared` (avoids a store per packet).
    src_port: u16,
}

impl RxState {
    fn new(stride: usize) -> Self {
        Self { assembly: None, geom: None, stride: stride.max(1), shared: Arc::default(), src_port: 0 }
    }
}

/// The receive thread: drain the GVSP socket, reassemble, and hand complete raw
/// frames to the control thread. It does nothing else — no GVCP, no decode — so
/// a burst is only ever competing with a memcpy for the socket buffer. Frames
/// the decoder hasn't picked up yet are dropped, never queued.
fn rx_thread(
    socket: UdpSocket,
    mut rx: RxState,
    raw_tx: Sender<(Vec<u8>, FrameGeometry)>,
    stop: Arc<AtomicBool>,
    log_tx: Sender<LogEntry>,
) {
    let mut buf = vec![0u8; 65536];
    while !stop.load(Ordering::Relaxed) {
        if let Some(frame) = receive_until_frame(&socket, &mut buf, &mut rx, &log_tx) {
            rx.shared.completed.fetch_add(1, Ordering::Relaxed);
            let _ = raw_tx.try_send(frame);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn capture_loop(
    mut dev: Device,
    socket: UdpSocket,
    mut genapi: Option<GenApi>,
    packet_payload: usize,
    cam_name: &str,
    frame_tx: Sender<FrameData>,
    cmd_rx: Receiver<GevCmd>,
    controls_tx: Sender<Vec<GevControl>>,
    mut snapshot: Vec<GevControl>,
    log_tx: Sender<LogEntry>,
) {
    let cam_ip = match dev.remote_addr().ip() {
        IpAddr::V4(ip) => ip,
        _ => Ipv4Addr::UNSPECIFIED,
    };

    // Start the receive thread before acquisition so the first packets land
    // in a drained socket. The control thread keeps a clone for hole-punching.
    let punch_socket = socket.try_clone().ok();
    let rx = RxState::new(packet_payload);
    let shared = Arc::clone(&rx.shared);
    let stop = Arc::new(AtomicBool::new(false));
    let (raw_tx, raw_rx) = bounded::<(Vec<u8>, FrameGeometry)>(2);
    let rx_join = {
        let stop = Arc::clone(&stop);
        let log_tx = log_tx.clone();
        std::thread::Builder::new()
            .name("gev-rx".into())
            .spawn(move || rx_thread(socket, rx, raw_tx, stop, log_tx))
            .ok()
    };
    let shutdown = |dev: &mut Device, genapi: &mut Option<GenApi>| {
        stop.store(true, Ordering::Relaxed);
        if let Some(g) = genapi.as_mut() {
            execute_command(g, dev, "AcquisitionStop", &log_tx);
            set_int_feature(g, dev, "TLParamsLocked", 0, &log_tx);
        }
        let _ = dev.release_control();
    };

    // Kick off acquisition now that everything is wired. TLParamsLocked=1 arms
    // the stream transport — required by FLIR/Point Grey before frames flow.
    if let Some(g) = genapi.as_mut() {
        set_int_feature(g, &mut dev, "TLParamsLocked", 1, &log_tx);
        execute_command(g, &mut dev, "AcquisitionStart", &log_tx);
    }
    // Open the reverse path through stateful host firewalls (Windows Defender
    // Firewall, macOS network-extension filters): they admit inbound UDP only
    // for a flow the socket has already sent on — which is why GVCP replies get
    // through — and the GVSP socket otherwise never transmits.
    let mut scsp = punch_stream_port(&mut dev, punch_socket.as_ref(), cam_ip, None, &shared);

    let started = Instant::now();
    let mut warned_silence = false;
    let mut last_heartbeat = Instant::now();
    let mut last_telemetry = Instant::now();
    let mut frames = 0u64;
    let mut short_frames = 0u64;
    // GEV_TRACE=1: log once a second where the time goes.
    let trace = std::env::var_os("GEV_TRACE").is_some();
    let mut tr = TraceAcc::default();

    loop {
        let t_loop = Instant::now();
        // 1. Service pending commands (sync GenICam access, not inside block_on).
        let mut stop_req = false;
        let mut changed = false;
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                GevCmd::Stop => { stop_req = true; break; }
                other => {
                    if let Some(g) = genapi.as_mut() {
                        apply_set(g, &mut dev, other, &log_tx);
                        changed = true;
                    }
                }
            }
        }
        // After any change, push a fresh control snapshot so the UI reflects new
        // values and writability (e.g. ExposureAuto=Off unlocks ExposureTime).
        if changed {
            if let Some(g) = genapi.as_mut() {
                snapshot = build_controls(g, &mut dev, cam_ip);
                let _ = controls_tx.try_send(snapshot.clone());
            }
        }
        if stop_req {
            shutdown(&mut dev, &mut genapi);
            if let Some(jh) = rx_join { let _ = jh.join(); }
            return;
        }

        // 2. Heartbeat; re-punch the stream flow while we're at it (a camera
        //    that reported GevSCSP=0 at start may know it now).
        if last_heartbeat.elapsed() >= HEARTBEAT_INTERVAL {
            if dev.read_register(CCP_REGISTER).is_err() {
                let _ = log_tx.try_send(LogEntry::error(format!("{cam_name}: camera disconnected")));
                stop.store(true, Ordering::Relaxed);
                if let Some(jh) = rx_join { let _ = jh.join(); }
                return;
            }
            last_heartbeat = Instant::now();
            scsp = punch_stream_port(&mut dev, punch_socket.as_ref(), cam_ip, scsp, &shared);
        }

        // 2b. Telemetry: re-read non-writable feature values periodically so
        // the UI shows live temperature/status without a full rebuild.
        if last_telemetry.elapsed() >= TELEMETRY_INTERVAL {
            last_telemetry = Instant::now();
            if let Some(g) = genapi.as_mut() {
                if refresh_telemetry(g, &mut dev, &mut snapshot) {
                    let _ = controls_tx.try_send(snapshot.clone());
                }
            }
        }

        // 2c. Silence diagnostic: a stream that produces nothing looks the same
        // for a firewall block, a packet-size mismatch, a lost privilege and an
        // untriggered camera. Say which half we're in and name the candidates.
        let packets = shared.packets.load(Ordering::Relaxed);
        if !warned_silence && frames == 0 && started.elapsed() >= SILENCE_GRACE {
            warned_silence = true;
            let msg = if packets == 0 {
                format!(
                    "{cam_name}: no GVSP packets received {}s after AcquisitionStart. Likely causes: a host \
                     firewall/EDR drops inbound UDP to this app (allow it; the camera streams from UDP port {}), \
                     the negotiated packet size exceeds what the link carries (try GEV_PACKET_SIZE=1500), \
                     the camera is waiting for a trigger, or another application holds control.",
                    SILENCE_GRACE.as_secs(),
                    scsp.map_or("unknown".to_string(), |p| p.to_string()),
                )
            } else {
                format!(
                    "{cam_name}: {} GVSP packets received but no frame completed ({} short frames). \
                     Likely packet loss (socket buffer / link) or a packet-size mismatch.",
                    packets, short_frames
                )
            };
            let _ = log_tx.try_send(LogEntry::warn(msg));
        }
        tr.ctl += t_loop.elapsed();

        // 3. Wait for the receive thread to complete a frame (or the poll window
        //    to expire), then decode it here so decoding never stalls the socket.
        let Ok((payload, g)) = raw_rx.recv_timeout(POLL_TIMEOUT) else {
            if rx_join.as_ref().is_some_and(|jh| jh.is_finished()) {
                let _ = log_tx.try_send(LogEntry::error(format!("{cam_name}: receive thread exited")));
                shutdown(&mut dev, &mut genapi);
                return;
            }
            trace_tick(trace, &mut tr, &shared, frames, &log_tx);
            continue;
        };
        let t_decode = Instant::now();
        let npix = g.width as usize * g.height as usize;
        match decode_payload(&payload, &g) {
            Some((mono, w, h, bit_depth)) => {
                if frames == 0 {
                    let _ = log_tx.try_send(LogEntry::info(format!(
                        "{cam_name}: streaming {w}x{h} pf={:#010x} ({} B/frame)",
                        g.pixel_format, payload.len()
                    )));
                }
                frames += 1;
                let frame = FrameData::new(mono, w, h, bit_depth);
                if frame_tx.try_send(frame).is_err() && frame_tx.is_empty() {
                    // Receiver gone.
                    shutdown(&mut dev, &mut genapi);
                    if let Some(jh) = rx_join { let _ = jh.join(); }
                    return;
                }
            }
            None => {
                short_frames += 1;
                if short_frames == 1 {
                    let needed = frame_payload_bytes(g.pixel_format, npix);
                    let msg = if payload.len() < needed {
                        format!(
                            "{cam_name}: frame {}x{} pf={:#010x} reassembled to {} B, need {} B \
                             — packet-size mismatch; dropping frames",
                            g.width, g.height, g.pixel_format, payload.len(), needed
                        )
                    } else {
                        format!(
                            "{cam_name}: unsupported pixel format {:#010x} ({}x{}); dropping frames",
                            g.pixel_format, g.width, g.height
                        )
                    };
                    let _ = log_tx.try_send(LogEntry::warn(msg));
                }
            }
        }
        tr.decode += t_decode.elapsed();
        trace_tick(trace, &mut tr, &shared, frames, &log_tx);
    }
}

/// Accumulator for the `GEV_TRACE` once-a-second capture-thread report.
struct TraceAcc {
    since: Instant,
    ctl: Duration,
    decode: Duration,
    packets: u64,
    completed: u64,
    decoded: u64,
}

impl Default for TraceAcc {
    fn default() -> Self {
        Self { since: Instant::now(), ctl: Duration::ZERO, decode: Duration::ZERO, packets: 0, completed: 0, decoded: 0 }
    }
}

fn trace_tick(enabled: bool, tr: &mut TraceAcc, shared: &RxShared, decoded: u64, log_tx: &Sender<LogEntry>) {
    if !enabled || tr.since.elapsed() < Duration::from_secs(1) {
        return;
    }
    let packets = shared.packets.load(Ordering::Relaxed);
    let completed = shared.completed.load(Ordering::Relaxed);
    let _ = log_tx.try_send(LogEntry::info(format!(
        "trace: {} pkt/s, {} frames completed, {} decoded, control {} ms, decode {} ms",
        packets - tr.packets, completed - tr.completed, decoded - tr.decoded,
        tr.ctl.as_millis(), tr.decode.as_millis()
    )));
    *tr = TraceAcc { since: Instant::now(), ctl: Duration::ZERO, decode: Duration::ZERO, packets, completed, decoded };
}

/// Per-frame geometry captured from the GVSP Leader packet.
#[derive(Clone, Copy)]
struct FrameGeometry {
    width: u32,
    height: u32,
    pixel_format: u32,
}

/// Image bytes per GVSP data packet for a `GevSCPSPacketSize` value.
fn gvsp_stride(packet_size: u32) -> usize {
    packet_size.saturating_sub(GVSP_PACKET_OVERHEAD).max(1) as usize
}

/// Start reassembling a block whose Leader announced geometry `g`.
fn new_assembly(g: &FrameGeometry, block_id: u64, stride: usize) -> FrameAssembly {
    let total = frame_payload_bytes(g.pixel_format, g.width as usize * g.height as usize);
    let expected = total.div_ceil(stride).max(1);
    FrameAssembly::new(block_id, expected, stride, Instant::now() + FRAME_DEADLINE)
}

/// Send one datagram from the receive socket to the port the camera streams
/// from, so stateful host firewalls admit the camera's packets as replies.
/// The port is the one the receive thread has actually seen, else `GevSCSP`,
/// else the last known; while nothing has arrived and none is known, spray
/// the usual range. Returns the port for reuse. Best-effort: every failure is
/// ignored, and hosts without such a firewall are unaffected.
fn punch_stream_port(
    dev: &mut Device,
    socket: Option<&UdpSocket>,
    cam_ip: Ipv4Addr,
    known: Option<u16>,
    shared: &RxShared,
) -> Option<u16> {
    let observed = shared.src_port.load(Ordering::Relaxed) as u16;
    let port = if observed != 0 {
        Some(observed)
    } else {
        match dev.read_register(SCSP_REGISTER) {
            Ok(v) if (v & 0xFFFF) != 0 => Some((v & 0xFFFF) as u16),
            _ => known,
        }
    };
    let Some(s) = socket else { return port };
    match port {
        Some(p) => {
            let _ = s.send_to(&[0u8], SocketAddr::new(IpAddr::V4(cam_ip), p));
        }
        None if shared.packets.load(Ordering::Relaxed) == 0 => {
            for p in PUNCH_SPRAY_PORTS {
                let _ = s.send_to(&[0u8], SocketAddr::new(IpAddr::V4(cam_ip), p));
            }
        }
        None => {}
    }
    port
}

/// Drain GVSP packets from the blocking socket, assembling the current frame.
/// Returns the finished payload + geometry when a Trailer completes a frame, or
/// `None` once the poll window elapses (the socket's read timeout) so the caller
/// can service commands. A partially received frame persists in `rx` across
/// calls, so a frame slower than one window still completes.
fn receive_until_frame(
    socket: &UdpSocket,
    buf: &mut [u8],
    rx: &mut RxState,
    log_tx: &Sender<LogEntry>,
) -> Option<(Vec<u8>, FrameGeometry)> {
    let deadline = Instant::now() + POLL_TIMEOUT;
    loop {
        let n = match socket.recv_from(buf) {
            Ok((n, src)) => {
                if src.port() != rx.src_port {
                    rx.src_port = src.port();
                    rx.shared.src_port.store(src.port() as u32, Ordering::Relaxed);
                }
                n
            }
            Err(e) => match e.kind() {
                // Read timeout: the poll window is over.
                ErrorKind::WouldBlock | ErrorKind::TimedOut => return None,
                // Windows reports an ICMP port-unreachable for an earlier send
                // (our hole punch) as ConnectionReset on the next recv. Harmless.
                ErrorKind::Interrupted | ErrorKind::ConnectionReset => {
                    if Instant::now() >= deadline { return None; }
                    continue;
                }
                _ => return None,
            },
        };
        rx.shared.packets.fetch_add(1, Ordering::Relaxed);
        let packet = match gvsp::parse_packet(&buf[..n]) {
            Ok(p) => p,
            Err(_) => continue,
        };
        match packet {
            GvspPacket::Leader { block_id, width, height, pixel_format, .. } => {
                let g = FrameGeometry { width, height, pixel_format };
                rx.geom = Some(g);
                rx.assembly = Some(new_assembly(&g, block_id, rx.stride));
            }
            GvspPacket::Payload { block_id, packet_id, data } => {
                let Some(g) = rx.geom else { continue };
                if rx.assembly.as_ref().map(FrameAssembly::block_id) != Some(block_id) {
                    continue;
                }
                // The wire is the authority on the stride: the first payload
                // packet of a block is full-sized unless the whole frame fits in
                // one packet, and any packet larger than the assumed stride
                // proves the assumption wrong. Re-seat the block on the real one.
                let total = frame_payload_bytes(g.pixel_format, g.width as usize * g.height as usize);
                let wire = data.len();
                if wire != rx.stride && wire < total && (packet_id == 1 || wire > rx.stride) {
                    let _ = log_tx.try_send(LogEntry::warn(format!(
                        "GigE: GVSP payload stride corrected {} -> {} B/packet (camera's packet size \
                         differs from the negotiated value)",
                        rx.stride, wire
                    )));
                    rx.stride = wire;
                    rx.assembly = Some(new_assembly(&g, block_id, wire));
                }
                if let Some(a) = rx.assembly.as_mut() {
                    // Leader/Trailer are packet 0 and N+1; payload ids are 1-based.
                    a.ingest(packet_id.saturating_sub(1) as usize, data);
                }
            }
            GvspPacket::Trailer { block_id, .. } => {
                if rx.assembly.as_ref().map(FrameAssembly::block_id) == Some(block_id) {
                    let a = rx.assembly.take().unwrap();
                    if let (Some(payload), Some(g)) = (a.finish(), rx.geom) {
                        return Some((payload, g));
                    }
                }
            }
        }
        if Instant::now() >= deadline {
            return None;
        }
    }
}

// ── Pixel decode ─────────────────────────────────────────────────────────--

/// Total GVSP payload bytes for a frame, given the pixel format and pixel count.
/// Packed formats use fractional bytes/pixel.
fn frame_payload_bytes(pixel_format: u32, npix: usize) -> usize {
    match pixel_format {
        0x01080001 => npix,                            // Mono8
        0x010C0006 | 0x010C0047 => npix * 3 / 2,       // Mono12Packed / Mono12p
        0x010A0046 => npix * 5 / 4,                     // Mono10p
        _ => npix * 2,                                  // 16-bit container (Mono10/12/14/16)
    }
}

/// Decode a reassembled mono payload into f32 pixels + bit depth.
fn decode_payload(payload: &[u8], g: &FrameGeometry) -> Option<(Vec<f32>, u32, u32, u8)> {
    let npix = g.width as usize * g.height as usize;
    if npix == 0 {
        return None;
    }
    match g.pixel_format {
        // Mono8
        0x01080001 => {
            if payload.len() < npix { return None; }
            let mono = payload[..npix].iter().map(|&v| v as f32).collect();
            Some((mono, g.width, g.height, 8))
        }
        // Mono10/12/14/16 unpacked little-endian 16-bit
        0x01100003 | 0x01100005 | 0x01100025 | 0x01100007 => {
            if payload.len() < npix * 2 { return None; }
            let mono = payload[..npix * 2]
                .chunks_exact(2)
                .map(|c| u16::from_le_bytes([c[0], c[1]]) as f32)
                .collect();
            let bit_depth = match g.pixel_format {
                0x01100003 => 10,
                0x01100005 => 12,
                0x01100025 => 14,
                _ => 16,
            };
            Some((mono, g.width, g.height, bit_depth))
        }
        // Mono12p packed: 2 pixels per 3 bytes (little-endian nibble order).
        0x010C0047 => {
            let needed = npix * 3 / 2;
            if payload.len() < needed { return None; }
            let mut mono = Vec::with_capacity(npix);
            for chunk in payload[..needed].chunks_exact(3) {
                let p0 = (chunk[0] as u16) | (((chunk[1] & 0x0F) as u16) << 8);
                let p1 = ((chunk[1] >> 4) as u16) | ((chunk[2] as u16) << 4);
                mono.push(p0 as f32);
                mono.push(p1 as f32);
            }
            mono.truncate(npix);
            Some((mono, g.width, g.height, 12))
        }
        // Mono12Packed (GEV 1.x / FLIR): 2 pixels per 3 bytes, high byte first.
        0x010C0006 => {
            let needed = npix * 3 / 2;
            if payload.len() < needed { return None; }
            let mut mono = Vec::with_capacity(npix);
            for chunk in payload[..needed].chunks_exact(3) {
                let p0 = ((chunk[0] as u16) << 4) | ((chunk[1] & 0x0F) as u16);
                let p1 = ((chunk[2] as u16) << 4) | ((chunk[1] >> 4) as u16);
                mono.push(p0 as f32);
                mono.push(p1 as f32);
            }
            mono.truncate(npix);
            Some((mono, g.width, g.height, 12))
        }
        // Mono10p packed: 4 pixels per 5 bytes.
        0x010A0046 => {
            let needed = npix * 5 / 4;
            if payload.len() < needed { return None; }
            let mut mono = Vec::with_capacity(npix);
            for chunk in payload[..needed].chunks_exact(5) {
                let p0 = (chunk[0] as u16) | (((chunk[1] & 0x03) as u16) << 8);
                let p1 = ((chunk[1] >> 2) as u16) | (((chunk[2] & 0x0F) as u16) << 6);
                let p2 = ((chunk[2] >> 4) as u16) | (((chunk[3] & 0x3F) as u16) << 4);
                let p3 = ((chunk[3] >> 6) as u16) | ((chunk[4] as u16) << 2);
                mono.extend_from_slice(&[p0 as f32, p1 as f32, p2 as f32, p3 as f32]);
            }
            mono.truncate(npix);
            Some((mono, g.width, g.height, 10))
        }
        _ => None,
    }
}

// ── GenICam feature setters ─────────────────────────────────────────────────

fn set_float_feature(g: &mut GenApi, dev: &mut Device, name: &str, v: f64, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(f) = nid.as_ifloat_kind(&g.store) {
            if let Err(e) = f.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn set_int_feature(g: &mut GenApi, dev: &mut Device, name: &str, v: i64, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(i) = nid.as_iinteger_kind(&g.store) {
            if let Err(e) = i.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn set_enum_feature(g: &mut GenApi, dev: &mut Device, name: &str, sym: &str, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            if let Err(e) = en.set_entry_by_symbolic(sym, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={sym}: {e}")));
            }
        }
    }
}

fn set_bool_feature(g: &mut GenApi, dev: &mut Device, name: &str, v: bool, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(bn) = nid.as_iboolean_kind(&g.store) {
            if let Err(e) = bn.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn execute_command(g: &mut GenApi, dev: &mut Device, name: &str, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(c) = nid.as_icommand_kind(&g.store) {
            if let Err(e) = c.execute(&mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE execute {name}: {e}")));
            }
        }
    }
}

/// Apply a Set* / Execute command. Features that change frame geometry
/// (PixelFormat, Width/Height, binning) can't be written while streaming, so for
/// those we stop acquisition, apply, and restart.
fn apply_set(g: &mut GenApi, dev: &mut Device, cmd: GevCmd, log_tx: &Sender<LogEntry>) {
    let restart = match &cmd {
        GevCmd::SetInt(n, _) | GevCmd::SetFloat(n, _) | GevCmd::SetEnum(n, _) | GevCmd::SetBool(n, _) => needs_restart(n),
        _ => false,
    };
    if restart {
        execute_command(g, dev, "AcquisitionStop", log_tx);
        set_int_feature(g, dev, "TLParamsLocked", 0, log_tx);
    }
    match cmd {
        GevCmd::SetInt(n, v) => set_int_feature(g, dev, &n, v, log_tx),
        GevCmd::SetFloat(n, v) => set_float_feature(g, dev, &n, v, log_tx),
        GevCmd::SetEnum(n, s) => set_enum_feature(g, dev, &n, &s, log_tx),
        GevCmd::SetBool(n, v) => set_bool_feature(g, dev, &n, v, log_tx),
        GevCmd::Execute(n) => execute_command(g, dev, &n, log_tx),
        GevCmd::Stop => {}
    }
    if restart {
        set_int_feature(g, dev, "TLParamsLocked", 1, log_tx);
        execute_command(g, dev, "AcquisitionStart", log_tx);
    }
}

#[cfg(test)]
mod tests {
    //! Drive the real receive path over loopback with synthetic GVSP packets.
    use super::*;

    const MONO8: u32 = 0x0108_0001;

    fn header(format: u8, block: u16, packet_id: u32) -> Vec<u8> {
        let mut v = vec![0u8, 0]; // status
        v.extend_from_slice(&block.to_be_bytes());
        v.push(format);
        v.extend_from_slice(&packet_id.to_be_bytes()[1..]);
        v
    }

    fn leader(block: u16, w: u32, h: u32) -> Vec<u8> {
        let mut v = header(0x01, block, 0);
        v.extend_from_slice(&[0, 0]); // reserved
        v.extend_from_slice(&1u16.to_be_bytes()); // payload type: image
        v.extend_from_slice(&0u64.to_be_bytes()); // timestamp
        v.extend_from_slice(&MONO8.to_be_bytes());
        v.extend_from_slice(&w.to_be_bytes());
        v.extend_from_slice(&h.to_be_bytes());
        v
    }

    fn trailer(block: u16, packet_id: u32, h: u32) -> Vec<u8> {
        let mut v = header(0x02, block, packet_id);
        v.extend_from_slice(&[0, 0]);
        v.extend_from_slice(&1u16.to_be_bytes());
        v.extend_from_slice(&h.to_be_bytes());
        v
    }

    /// Send one Mono8 frame split at `wire_stride` bytes per packet, and
    /// reassemble it with a receiver seeded with `seed_stride`.
    fn round_trip(wire_stride: usize, seed_stride: usize) -> (Vec<u8>, Option<Vec<u8>>, usize) {
        let (w, h) = (100u32, 40u32);
        let image: Vec<u8> = (0..(w * h) as usize).map(|i| (i * 7 % 251) as u8).collect();

        let rx_sock = UdpSocket::bind("127.0.0.1:0").unwrap();
        rx_sock.set_read_timeout(Some(POLL_TIMEOUT)).unwrap();
        let tx = UdpSocket::bind("127.0.0.1:0").unwrap();
        let dst = rx_sock.local_addr().unwrap();

        tx.send_to(&leader(7, w, h), dst).unwrap();
        let mut id = 1u32;
        for chunk in image.chunks(wire_stride) {
            let mut v = header(0x03, 7, id);
            v.extend_from_slice(chunk);
            tx.send_to(&v, dst).unwrap();
            id += 1;
        }
        tx.send_to(&trailer(7, id, h), dst).unwrap();

        let (log_tx, _log_rx) = bounded::<LogEntry>(16);
        let mut rx = RxState::new(seed_stride);
        let mut buf = vec![0u8; 65536];
        let got = receive_until_frame(&rx_sock, &mut buf, &mut rx, &log_tx).map(|(p, _)| p);
        (image, got, rx.stride)
    }

    /// Full `start_camera_at` → capture-thread path against a running simulator
    /// (viva-fake-gige). Ignored: needs the simulator. The GVCP port defaults to
    /// 3957 (a second instance alongside one on the standard 3956) and can be
    /// overridden with `GEV_TEST_PORT`. Run the fake with
    /// `GEV_PIXEL_FORMAT=Mono8`, since it emits 1 byte/pixel regardless of the
    /// selected PixelFormat.
    ///
    ///   cargo test --features gev -- --ignored streams_frames_from_fake_camera --nocapture
    #[test]
    #[ignore]
    fn streams_frames_from_fake_camera() {
        let port: u16 = std::env::var("GEV_TEST_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(3957);
        let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port);
        let (frame_tx, frame_rx) = bounded::<FrameData>(2);
        let (log_tx, log_rx) = bounded::<LogEntry>(256);
        let mut handle = start_camera_at(addr, "fake".into(), frame_tx, log_tx).expect("start_camera_at");
        let deadline = Instant::now() + Duration::from_secs(15);
        let mut frames = 0;
        while Instant::now() < deadline && frames < 3 {
            if let Ok(f) = frame_rx.recv_timeout(Duration::from_millis(500)) {
                frames += 1;
                eprintln!("frame {}x{} bit_depth={} mean={:.1}", f.width, f.height, f.bit_depth, f.mean);
            }
            while let Ok(e) = log_rx.try_recv() {
                eprintln!("log: {}", e.message);
            }
        }
        handle.stop();
        while let Ok(e) = log_rx.try_recv() {
            eprintln!("log: {}", e.message);
        }
        assert!(frames >= 3, "expected at least 3 frames, got {frames}");
    }

    #[test]
    fn gvsp_stride_excludes_ip_udp_and_gvsp_headers() {
        assert_eq!(gvsp_stride(1500), 1464);
        assert_eq!(gvsp_stride(1458), 1422);
        assert_eq!(gvsp_stride(9000), 8964);
    }

    #[test]
    fn matching_stride_reassembles_the_frame() {
        let (image, got, stride) = round_trip(1464, 1464);
        assert_eq!(got.as_deref(), Some(&image[..]));
        assert_eq!(stride, 1464);
    }

    #[test]
    fn too_large_seed_stride_is_corrected_from_the_wire() {
        // The viva-gige 0.2 -> 0.5 regression: seeded with packet_size - 8
        // while the camera sends packet_size - 36 per packet.
        let (image, got, stride) = round_trip(1464, 1492);
        assert_eq!(got.as_deref(), Some(&image[..]));
        assert_eq!(stride, 1464);
    }

    #[test]
    fn too_small_seed_stride_is_corrected_from_the_wire() {
        let (image, got, stride) = round_trip(1464, 1000);
        assert_eq!(got.as_deref(), Some(&image[..]));
        assert_eq!(stride, 1464);
    }
}

// Trait imports needed for the `*Kind` method calls above.
use cameleon_genapi::interface::{IBoolean, ICommand, IEnumeration, IFloat, IInteger};
