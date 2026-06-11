//! GigE Vision (GEV) camera source — pure Rust, no C/system dependencies.
//!
//! Transport is provided by the `viva-gige` crate (GVCP discovery + control,
//! GVSP streaming building blocks). GenICam feature access (Exposure, Gain,
//! Width/Height, PixelFormat, AcquisitionStart/Stop, …) is provided by
//! `cameleon-genapi`, which parses the camera's GenICam XML and interprets its
//! feature nodes; its register reads/writes are bridged onto `viva-gige`'s
//! async control channel via a small synchronous [`Device`] adapter.
//!
//! This module owns a self-contained
//! capture thread that produces [`FrameData`] over the shared `frame_tx`
//! channel and accepts control changes over a command channel. Because
//! `viva-gige` is async (tokio) while the rest of the app is sync threads +
//! crossbeam channels, the capture thread owns a current-thread tokio runtime
//! and drives each async operation with a discrete `block_on` — never nested,
//! so the sync `Device` bridge can itself `block_on` when applying controls.

use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use bytes::BytesMut;
use crossbeam_channel::{bounded, Receiver, Sender};
use tokio::net::UdpSocket;
use tokio::runtime::Runtime;

use cameleon_genapi::elem_type::Visibility;
use cameleon_genapi::interface::ICategoryKind;
use cameleon_genapi::store::{DefaultCacheStore, DefaultNodeStore, DefaultValueStore, NodeId, NodeStore};
use cameleon_genapi::ValueCtxt;
use viva_gige::gvcp::{self, DeviceInfo, GigeDevice};
use viva_gige::gvsp::{self, FrameAssembly, GvspPacket};
use viva_gige::nic::{self, Iface};

use crate::{FrameData, LogEntry};

/// GVCP Control Channel Privilege register — re-read periodically as a heartbeat
/// so the camera does not reclaim control from us.
const CCP_REGISTER: u32 = 0x0a00;
/// Bootstrap "First URL" register pointing at the on-device GenICam XML.
const FIRST_URL_REGISTER: u64 = 0x0200;
/// Heartbeat period. GigE devices default to a ~3 s heartbeat timeout.
const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(1000);
/// Stream channel index (single-channel cameras use 0).
const STREAM_CHANNEL: u32 = 0;
/// How long to wait for stream packets before returning to service commands.
const POLL_TIMEOUT: Duration = Duration::from_millis(200);
/// How long an in-flight frame may take to reassemble before being abandoned.
const FRAME_DEADLINE: Duration = Duration::from_millis(1000);

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
    let rt = match build_runtime() {
        Ok(rt) => rt,
        Err(_) => return Vec::new(),
    };
    let devices = rt
        .block_on(gvcp::discover_all(Duration::from_millis(500)))
        .unwrap_or_default();
    devices.into_iter().map(device_info_to_gev).collect()
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

fn build_runtime() -> anyhow::Result<Runtime> {
    Ok(tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?)
}

// ── GenICam node store + value context bundle ───────────────────────────────

/// The parsed GenICam model plus the value/cache context needed to evaluate it.
struct GenApi {
    store: DefaultNodeStore,
    ctxt: ValueCtxt<DefaultValueStore, DefaultCacheStore>,
}

/// Synchronous [`cameleon_genapi::Device`] bridge over the async `viva-gige`
/// control channel. Holds raw pointers to the runtime and device owned by the
/// capture thread; only ever used transiently while applying a feature, and
/// never while already inside `runtime.block_on`, so the nested-`block_on`
/// panic is avoided by construction.
struct DeviceBridge<'a> {
    rt: &'a Runtime,
    dev: &'a mut GigeDevice,
}

impl<'a> cameleon_genapi::Device for DeviceBridge<'a> {
    fn read_mem(
        &mut self,
        address: i64,
        buf: &mut [u8],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let data = self
            .rt
            .block_on(self.dev.read_mem(address as u64, buf.len()))?;
        let n = buf.len().min(data.len());
        buf[..n].copy_from_slice(&data[..n]);
        Ok(())
    }

    fn write_mem(
        &mut self,
        address: i64,
        data: &[u8],
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.rt.block_on(self.dev.write_mem(address as u64, data))?;
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
    let rt = build_runtime()?;
    let ip = info.ip;

    // Connect + claim exclusive control.
    let mut dev = rt.block_on(GigeDevice::open(SocketAddr::new(IpAddr::V4(ip), gvcp::GVCP_PORT)))?;
    rt.block_on(dev.claim_control())?;
    let _ = log_tx.try_send(LogEntry::info(format!("GigE: claimed control of {}", info.display_name())));

    // Fetch + parse the GenICam XML.
    let mut genapi = match load_genapi(&rt, &mut dev) {
        Ok(g) => Some(g),
        Err(e) => {
            let _ = log_tx.try_send(LogEntry::warn(format!(
                "GigE: GenICam XML unavailable ({e}); controls disabled, streaming whatever the camera emits"
            )));
            None
        }
    };

    // Configure mono full-frame acquisition and read back geometry, then build
    // the control list. All of this uses synchronous GenICam access (which
    // internally block_on's the device) — we are not inside block_on here.
    let mut controls = Vec::new();
    if let Some(g) = genapi.as_mut() {
        // best-effort configuration; ignore individual feature failures.
        configure_acquisition(g, &mut dev, &rt, &log_tx);
        controls = build_controls(g, &mut dev, &rt);
    }

    // Negotiate the GVSP stream channel against our receiving interface.
    let iface = Iface::from_ipv4(local_ipv4_towards(ip)).ok();
    // Bind the GVSP receive socket first, then point the camera at it.
    let bind_ip = iface
        .as_ref()
        .and_then(|i| i.ipv4())
        .map(IpAddr::V4)
        .unwrap_or(nic::default_bind_addr());
    let socket = rt.block_on(nic::bind_udp(bind_ip, 0, iface.clone(), None))?;
    let local_port = socket.local_addr()?.port();

    let stream_params = rt.block_on(dev.negotiate_stream(
        STREAM_CHANNEL,
        iface.as_ref().ok_or_else(|| anyhow::anyhow!("no usable network interface for GigE streaming"))?,
        local_port,
        None,
    ))?;
    let _ = log_tx.try_send(LogEntry::info(format!(
        "GigE: stream channel negotiated (mtu={}, packet_size={})",
        stream_params.mtu, stream_params.packet_size
    )));

    // Acquisition is started inside the capture thread once everything is wired.

    let (cmd_tx, cmd_rx) = bounded::<GevCmd>(32);
    let (controls_tx, controls_rx) = bounded::<Vec<GevControl>>(4);
    let packet_payload = stream_params.packet_size.saturating_sub(8).max(1) as usize;
    let cam_name = info.display_name();

    let join_handle = std::thread::Builder::new()
        .name("gev-capture".into())
        .spawn(move || {
            capture_loop(
                rt, dev, socket, genapi, packet_payload, &cam_name, frame_tx, cmd_rx, controls_tx, log_tx,
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
fn load_genapi(rt: &Runtime, dev: &mut GigeDevice) -> anyhow::Result<GenApi> {
    let raw = rt.block_on(dev.read_mem(FIRST_URL_REGISTER, 512))?;
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

    let bytes = read_mem_chunked(rt, dev, addr, len)?;

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
fn read_mem_chunked(
    rt: &Runtime,
    dev: &mut GigeDevice,
    addr: u64,
    len: usize,
) -> anyhow::Result<Vec<u8>> {
    const CHUNK: usize = 512;
    let mut out = Vec::with_capacity(len);
    let mut off = 0usize;
    while off < len {
        let want = CHUNK.min(len - off);
        // GVCP READMEM requires a 4-byte-aligned count; round up, then keep
        // only the bytes we actually need.
        let req = (want + 3) & !3;
        let part = rt
            .block_on(dev.read_mem(addr + off as u64, req))
            .map_err(|e| anyhow::anyhow!("READMEM {req} B @ {:#x}: {e}", addr + off as u64))?;
        out.extend_from_slice(&part[..want.min(part.len())]);
        off += want;
    }
    Ok(out)
}

/// Configure full-frame mono acquisition: set PixelFormat to the widest mono
/// format the camera offers, set Width/Height to max, AcquisitionMode=Continuous.
fn configure_acquisition(
    g: &mut GenApi,
    dev: &mut GigeDevice,
    rt: &Runtime,
    log_tx: &Sender<LogEntry>,
) {
    let mut bridge = DeviceBridge { rt, dev };

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

    // Prefer a mono pixel format, widest bit depth available.
    if let Some(nid) = g.store.id_by_name("PixelFormat") {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            for want in ["Mono16", "Mono14", "Mono12", "Mono10", "Mono8"] {
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
fn build_controls(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime) -> Vec<GevControl> {
    let mut out = Vec::new();
    let mut b = DeviceBridge { rt, dev };
    let Some(root) = g.store.id_by_name("Root") else { return out };
    walk_category(g, &mut b, root, None, &mut out);
    out
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
    };

    // Order matters: boolean/enumeration before integer, since some are both.
    if let Some(bn) = nid.as_iboolean_kind(&g.store) {
        let readable = bn.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = bn.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let v = bn.value(b, &g.store, &mut g.ctxt).unwrap_or(false);
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
        let cur = en.current_entry(b, &g.store, &mut g.ctxt).ok()
            .and_then(|e| e.expect_enum_entry(&g.store).ok().map(|x| x.symbolic().to_string()));
        let idx = cur.as_ref().and_then(|s| opts.iter().position(|o| o == s)).unwrap_or(0);
        let mut c = base(GevControlKind::Enumeration(opts), String::new(), writable);
        c.value = idx as i64;
        Some(c)
    } else if let Some(f) = nid.as_ifloat_kind(&g.store) {
        let readable = f.is_readable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        let writable = f.is_writable(b, &g.store, &mut g.ctxt).unwrap_or(false);
        if !readable && !writable { return None; }
        let unit = f.unit(&g.store).unwrap_or("").to_string();
        let value = f.value(b, &g.store, &mut g.ctxt).unwrap_or(0.0);
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
        let value = i.value(b, &g.store, &mut g.ctxt).unwrap_or(0);
        let min = i.min(b, &g.store, &mut g.ctxt).unwrap_or(0);
        let max = i.max(b, &g.store, &mut g.ctxt).unwrap_or(0);
        let mut c = base(GevControlKind::Integer, unit, writable);
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

// ── Capture loop ─────────────────────────────────────────────────────────--

#[allow(clippy::too_many_arguments)]
fn capture_loop(
    rt: Runtime,
    mut dev: GigeDevice,
    socket: UdpSocket,
    mut genapi: Option<GenApi>,
    packet_payload: usize,
    cam_name: &str,
    frame_tx: Sender<FrameData>,
    cmd_rx: Receiver<GevCmd>,
    controls_tx: Sender<Vec<GevControl>>,
    log_tx: Sender<LogEntry>,
) {
    // Kick off acquisition now that everything is wired. TLParamsLocked=1 arms
    // the stream transport — required by FLIR/Point Grey before frames flow.
    if let Some(g) = genapi.as_mut() {
        set_int_feature(g, &mut dev, &rt, "TLParamsLocked", 1, &log_tx);
        execute_command(g, &mut dev, &rt, "AcquisitionStart", &log_tx);
    }

    let mut last_heartbeat = Instant::now();
    let mut assembly: Option<FrameAssembly> = None;
    let mut geom: Option<FrameGeometry> = None;
    let mut buf = vec![0u8; 65536];

    loop {
        // 1. Service pending commands (sync GenICam access, not inside block_on).
        let mut stop = false;
        let mut changed = false;
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                GevCmd::Stop => { stop = true; break; }
                other => {
                    if let Some(g) = genapi.as_mut() {
                        apply_set(g, &mut dev, &rt, other, &log_tx);
                        changed = true;
                    }
                }
            }
        }
        // After any change, push a fresh control snapshot so the UI reflects new
        // values and writability (e.g. ExposureAuto=Off unlocks ExposureTime).
        if changed {
            if let Some(g) = genapi.as_mut() {
                let snap = build_controls(g, &mut dev, &rt);
                let _ = controls_tx.try_send(snap);
            }
        }
        if stop {
            if let Some(g) = genapi.as_mut() {
                execute_command(g, &mut dev, &rt, "AcquisitionStop", &log_tx);
                set_int_feature(g, &mut dev, &rt, "TLParamsLocked", 0, &log_tx);
            }
            let _ = rt.block_on(dev.release_control());
            return;
        }

        // 2. Heartbeat.
        if last_heartbeat.elapsed() >= HEARTBEAT_INTERVAL {
            if rt.block_on(dev.read_register(CCP_REGISTER)).is_err() {
                let _ = log_tx.try_send(LogEntry::error(format!("{cam_name}: camera disconnected")));
                return;
            }
            last_heartbeat = Instant::now();
        }

        // 3. Receive GVSP packets until a frame completes or the poll window expires.
        let completed = rt.block_on(receive_until_frame(
            &socket, &mut buf, &mut assembly, &mut geom, packet_payload,
        ));

        if let Some((payload, g)) = completed {
            if let Some((mono, w, h, bit_depth)) = decode_payload(&payload, &g) {
                let frame = FrameData::new(mono, w, h, bit_depth);
                if frame_tx.try_send(frame).is_err() && frame_tx.is_empty() {
                    // Receiver gone.
                    if let Some(g) = genapi.as_mut() {
                        execute_command(g, &mut dev, &rt, "AcquisitionStop", &log_tx);
                        set_int_feature(g, &mut dev, &rt, "TLParamsLocked", 0, &log_tx);
                    }
                    let _ = rt.block_on(dev.release_control());
                    return;
                }
            }
        }
    }
}

/// Per-frame geometry captured from the GVSP Leader packet.
#[derive(Clone, Copy)]
struct FrameGeometry {
    width: u32,
    height: u32,
    pixel_format: u32,
}

/// Drain GVSP packets, assembling the current frame. Returns the finished
/// payload + geometry when a Trailer completes a frame, or `None` if the poll
/// window elapses first.
async fn receive_until_frame(
    socket: &UdpSocket,
    buf: &mut [u8],
    assembly: &mut Option<FrameAssembly>,
    geom: &mut Option<FrameGeometry>,
    packet_payload: usize,
) -> Option<(bytes::Bytes, FrameGeometry)> {
    let deadline = tokio::time::Instant::now() + POLL_TIMEOUT;
    loop {
        let recv = tokio::time::timeout_at(deadline, socket.recv_from(buf)).await;
        let n = match recv {
            Ok(Ok((n, _src))) => n,
            Ok(Err(_)) => return None,
            Err(_) => return None, // poll window elapsed
        };
        let packet = match gvsp::parse_packet(&buf[..n]) {
            Ok(p) => p,
            Err(_) => continue,
        };
        match packet {
            GvspPacket::Leader { block_id, width, height, pixel_format, .. } => {
                *geom = Some(FrameGeometry { width, height, pixel_format });
                let total_bytes = frame_payload_bytes(pixel_format, width as usize * height as usize);
                let expected_packets = total_bytes.div_ceil(packet_payload).max(1);
                let pool = BytesMut::zeroed(expected_packets * packet_payload);
                let dl = Instant::now() + FRAME_DEADLINE;
                *assembly = Some(FrameAssembly::new(block_id, expected_packets, packet_payload, pool, dl));
            }
            GvspPacket::Payload { block_id, packet_id, data } => {
                if let Some(a) = assembly.as_mut() {
                    if a.block_id() == block_id {
                        // Leader/Trailer are packet 0 and N+1; payload ids are 1-based.
                        a.ingest(packet_id.saturating_sub(1) as usize, &data);
                    }
                }
            }
            GvspPacket::Trailer { block_id, .. } => {
                if let Some(a) = assembly.as_ref() {
                    if a.block_id() == block_id {
                        let a = assembly.take().unwrap();
                        if let (Some(payload), Some(g)) = (a.finish(), *geom) {
                            return Some((payload, g));
                        }
                    }
                }
            }
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

fn set_float_feature(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, name: &str, v: f64, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { rt, dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(f) = nid.as_ifloat_kind(&g.store) {
            if let Err(e) = f.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn set_int_feature(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, name: &str, v: i64, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { rt, dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(i) = nid.as_iinteger_kind(&g.store) {
            if let Err(e) = i.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn set_enum_feature(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, name: &str, sym: &str, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { rt, dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(en) = nid.as_ienumeration_kind(&g.store) {
            if let Err(e) = en.set_entry_by_symbolic(sym, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={sym}: {e}")));
            }
        }
    }
}

fn set_bool_feature(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, name: &str, v: bool, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { rt, dev };
    if let Some(nid) = g.store.id_by_name(name) {
        if let Some(bn) = nid.as_iboolean_kind(&g.store) {
            if let Err(e) = bn.set_value(v, &mut bridge, &g.store, &mut g.ctxt) {
                let _ = log_tx.try_send(LogEntry::error(format!("GigE set {name}={v}: {e}")));
            }
        }
    }
}

fn execute_command(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, name: &str, log_tx: &Sender<LogEntry>) {
    let mut bridge = DeviceBridge { rt, dev };
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
fn apply_set(g: &mut GenApi, dev: &mut GigeDevice, rt: &Runtime, cmd: GevCmd, log_tx: &Sender<LogEntry>) {
    let restart = match &cmd {
        GevCmd::SetInt(n, _) | GevCmd::SetFloat(n, _) | GevCmd::SetEnum(n, _) | GevCmd::SetBool(n, _) => needs_restart(n),
        _ => false,
    };
    if restart {
        execute_command(g, dev, rt, "AcquisitionStop", log_tx);
        set_int_feature(g, dev, rt, "TLParamsLocked", 0, log_tx);
    }
    match cmd {
        GevCmd::SetInt(n, v) => set_int_feature(g, dev, rt, &n, v, log_tx),
        GevCmd::SetFloat(n, v) => set_float_feature(g, dev, rt, &n, v, log_tx),
        GevCmd::SetEnum(n, s) => set_enum_feature(g, dev, rt, &n, &s, log_tx),
        GevCmd::SetBool(n, v) => set_bool_feature(g, dev, rt, &n, v, log_tx),
        GevCmd::Execute(n) => execute_command(g, dev, rt, &n, log_tx),
        GevCmd::Stop => {}
    }
    if restart {
        set_int_feature(g, dev, rt, "TLParamsLocked", 1, log_tx);
        execute_command(g, dev, rt, "AcquisitionStart", log_tx);
    }
}

// ── Misc ────────────────────────────────────────────────────────────────--

/// Best-effort: pick a local IPv4 on the same network as the camera by opening a
/// UDP socket "towards" it (no packets are sent). Falls back to 0.0.0.0.
fn local_ipv4_towards(target: Ipv4Addr) -> Ipv4Addr {
    use std::net::UdpSocket as StdUdp;
    if let Ok(sock) = StdUdp::bind((Ipv4Addr::UNSPECIFIED, 0)) {
        if sock.connect((target, gvcp::GVCP_PORT)).is_ok() {
            if let Ok(SocketAddr::V4(local)) = sock.local_addr() {
                return *local.ip();
            }
        }
    }
    Ipv4Addr::UNSPECIFIED
}

// Trait imports needed for the `*Kind` method calls above.
use cameleon_genapi::interface::{IBoolean, ICommand, IEnumeration, IFloat, IInteger};
