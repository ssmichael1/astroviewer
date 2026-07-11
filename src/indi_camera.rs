//! INDI client — connects to an indiserver, drives a CCD driver, and delivers
//! frames as `FrameData` over the same channel pattern as `camera` / `gev_camera`.
//!
//! Protocol notes (INDI 1.7, docs.indilib.org/protocol):
//! - Plain TCP (default port 7624) carrying a stream of flat XML elements with
//!   no document root. The client sends `<getProperties/>` and receives
//!   `def*Vector` / `set*Vector` / `delProperty` / `message` elements forever.
//! - Property values are changed by sending `new*Vector` elements.
//! - Images arrive as base64 BLOBs (FITS), only after `<enableBLOB>` opt-in.
//! - INDI CCDs are one-shot: write CCD_EXPOSURE, wait for the BLOB. "Live view"
//!   is implemented here by re-triggering the exposure when each BLOB lands.
//!
//! INDIGO (indigo-astronomy.org) servers speak a backward-compatible
//! extension of the same protocol. The client offers it in the handshake
//! (`<getProperties version='1.7' switch='2.0'/>`); a legacy INDI server
//! ignores the extra attribute, an INDIGO server replies
//! `<switchProtocol version='2.0'/>`. Once negotiated:
//! - BLOBs are requested in `URL` mode: `setBLOBVector` then carries a
//!   `url`/`path` attribute and the frame is fetched as *raw binary* over
//!   HTTP from the server — no base64 (skips the 33 % size overhead and the
//!   encode/decode passes on both ends, which is what makes INDIGO framing
//!   faster than classic INDI).
//! - Well-known item names switch to the INDIGO dialect
//!   (`CCD_EXPOSURE.EXPOSURE` instead of `CCD_EXPOSURE.CCD_EXPOSURE_VALUE`,
//!   `CONNECTION.CONNECTED` instead of `CONNECTION.CONNECT`).
//!
//! Threading (mirrors `gev_camera`):
//! - A *writer* thread owns the command channel and serializes `IndiCmd` → XML.
//! - A *reader* thread blocks on the socket, parses elements, maintains the
//!   property store, decodes BLOBs, and pushes property snapshots to the UI.
//! - `IndiCmd::Stop` shuts down the socket, which unblocks the reader.
//!
use anyhow::{anyhow, bail, Result};
use crossbeam_channel::{bounded, Receiver, Sender};
use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
use std::collections::HashMap;
use std::io::{BufReader, Write};
use std::net::{Shutdown, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

pub const DEFAULT_PORT: u16 = 7624;

/// Well-known property/element names (conventions, not schema).
pub const PROP_CONNECTION: &str = "CONNECTION";
pub const PROP_EXPOSURE: &str = "CCD_EXPOSURE";
pub const ELEM_EXPOSURE: &str = "CCD_EXPOSURE_VALUE";

/// The exposure item name in the negotiated dialect (INDIGO renames the
/// well-known items; property names are unchanged).
fn exposure_item(indigo: bool) -> &'static str {
    if indigo { "EXPOSURE" } else { ELEM_EXPOSURE }
}

/// `(connect, disconnect)` item names in the negotiated dialect.
fn connection_items(indigo: bool) -> (&'static str, &'static str) {
    if indigo { ("CONNECTED", "DISCONNECTED") } else { ("CONNECT", "DISCONNECT") }
}

// ── Protocol model ──────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PropState { Idle, Ok, Busy, Alert }

impl PropState {
    fn parse(s: &str) -> Self {
        match s {
            "Ok" => PropState::Ok,
            "Busy" => PropState::Busy,
            "Alert" => PropState::Alert,
            _ => PropState::Idle,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PropPerm { Ro, Wo, Rw }

impl PropPerm {
    fn parse(s: &str) -> Self {
        match s {
            "wo" => PropPerm::Wo,
            "rw" => PropPerm::Rw,
            _ => PropPerm::Ro,
        }
    }
}

/// How switches in a vector interact (radio group vs. checkboxes).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SwitchRule { OneOfMany, AtMostOne, AnyOfMany }

impl SwitchRule {
    fn parse(s: &str) -> Self {
        match s {
            "AtMostOne" => SwitchRule::AtMostOne,
            "AnyOfMany" => SwitchRule::AnyOfMany,
            _ => SwitchRule::OneOfMany,
        }
    }
}

/// One element (member) of a property vector.
#[derive(Clone, Debug)]
pub enum IndiValue {
    Number {
        value: f64,
        min: f64,
        max: f64,
        step: f64,
        /// printf-style display format from the driver (may be sexagesimal,
        /// e.g. "%10.6m"); kept for future formatted display.
        #[allow(dead_code)]
        format: String,
    },
    Switch(bool),
    Text(String),
    Light(PropState),
    /// BLOB metadata only — pixel data goes straight to the frame channel,
    /// never into the property store.
    Blob { format: String, size: usize },
}

#[derive(Clone, Debug)]
pub struct IndiElement {
    pub name: String,
    pub label: String,
    pub value: IndiValue,
}

/// A property vector as defined by the driver. This is the UI-facing unit —
/// render one widget group per property, like `GevControl`.
#[derive(Clone, Debug)]
pub struct IndiProperty {
    pub device: String,
    pub name: String,
    pub label: String,
    pub group: String,
    pub state: PropState,
    pub perm: PropPerm,
    pub rule: Option<SwitchRule>,
    pub elements: Vec<IndiElement>,
}

// ── Commands (UI thread → writer thread) ────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // Never/Only are part of the protocol even if the UI only uses Also
pub enum BlobMode { Never, Also, Only }

impl BlobMode {
    fn as_str(self) -> &'static str {
        match self {
            BlobMode::Never => "Never",
            BlobMode::Also => "Also",
            BlobMode::Only => "Only",
        }
    }
}

pub enum IndiCmd {
    /// Set number elements: (element name, value).
    SetNumber { device: String, property: String, values: Vec<(String, f64)> },
    /// Set switch elements: (element name, on).
    SetSwitch { device: String, property: String, values: Vec<(String, bool)> },
    /// Set text elements: (element name, text).
    SetText { device: String, property: String, values: Vec<(String, String)> },
    /// Opt in/out of BLOB delivery for a device.
    EnableBlob { device: String, mode: BlobMode },
    /// Convenience: CONNECTION.CONNECT = On.
    Connect { device: String },
    /// Trigger an exposure; if `live`, re-trigger on every received frame.
    StartExposure { device: String, seconds: f64, live: bool },
    /// Stop re-triggering (does not abort an in-flight exposure).
    StopLive,
    /// Shut down the connection and both threads.
    Stop,
}

// ── Handle ──────────────────────────────────────────────────────────────────

/// Handle to a running INDI client. Mirrors `GevHandle`.
pub struct IndiHandle {
    pub cmd_tx: Sender<IndiCmd>,
    /// Latest full property snapshot, replaced wholesale by the reader thread.
    /// A mailbox slot rather than a channel: a server's initial enumeration is
    /// a burst of hundreds of updates, and a bounded channel drops the later
    /// (complete) snapshots once full — the UI would be stuck on a stale
    /// early one. `take()` it each frame.
    pub props: Arc<Mutex<Option<Vec<IndiProperty>>>>,
    reader_jh: Option<JoinHandle<()>>,
    writer_jh: Option<JoinHandle<()>>,
}

impl IndiHandle {
    pub fn stop(&mut self) {
        let _ = self.cmd_tx.send(IndiCmd::Stop);
        for jh in [self.writer_jh.take(), self.reader_jh.take()].into_iter().flatten() {
            let _ = jh.join();
        }
    }
}

impl Drop for IndiHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

/// State shared between reader (blob → retrigger) and writer (exposure cmds).
struct SharedState {
    live: AtomicBool,
    exposure_s: Mutex<f64>,
    live_device: Mutex<String>,
    /// Server accepted the INDIGO 2.0 protocol extension (see module docs).
    indigo: AtomicBool,
}

// ── Client startup ──────────────────────────────────────────────────────────

/// Connect to an indiserver and spawn the reader/writer threads.
pub fn start_client(
    host: &str,
    port: u16,
    frame_tx: Sender<super::FrameData>,
    log_tx: Sender<super::LogEntry>,
) -> Result<IndiHandle> {
    let stream = TcpStream::connect((host, port))
        .map_err(|e| anyhow!("connect {host}:{port}: {e}"))?;
    stream.set_nodelay(true).ok();

    let mut write_stream = stream.try_clone()?;
    // Offer the INDIGO protocol extension; legacy INDI servers ignore the
    // extra attributes, INDIGO replies <switchProtocol version='2.0'/>.
    write_stream
        .write_all(b"<getProperties version=\"1.7\" client=\"AstroViewer\" switch=\"2.0\"/>\n")?;

    let (cmd_tx, cmd_rx) = bounded::<IndiCmd>(32);
    let props: Arc<Mutex<Option<Vec<IndiProperty>>>> = Arc::new(Mutex::new(None));
    let shared = Arc::new(SharedState {
        live: AtomicBool::new(false),
        exposure_s: Mutex::new(1.0),
        live_device: Mutex::new(String::new()),
        indigo: AtomicBool::new(false),
    });
    // Kept for resolving INDIGO's server-relative BLOB paths ("/blob/…").
    let server_addr = format!("{host}:{port}");

    let writer_jh = {
        let shared = shared.clone();
        let log_tx = log_tx.clone();
        std::thread::spawn(move || writer_loop(write_stream, cmd_rx, shared, log_tx))
    };
    let reader_jh = {
        let shared = shared.clone();
        let cmd_tx = cmd_tx.clone();
        let props = props.clone();
        std::thread::spawn(move || {
            reader_loop(stream, server_addr, frame_tx, props, cmd_tx, shared, log_tx)
        })
    };

    Ok(IndiHandle {
        cmd_tx,
        props,
        reader_jh: Some(reader_jh),
        writer_jh: Some(writer_jh),
    })
}

// ── Writer thread: IndiCmd → XML ────────────────────────────────────────────

fn writer_loop(
    mut stream: TcpStream,
    cmd_rx: Receiver<IndiCmd>,
    shared: Arc<SharedState>,
    log_tx: Sender<super::LogEntry>,
) {
    loop {
        let cmd = match cmd_rx.recv() {
            Ok(IndiCmd::Stop) | Err(_) => {
                shared.live.store(false, Ordering::Relaxed);
                let _ = stream.shutdown(Shutdown::Both);
                return;
            }
            Ok(cmd) => cmd,
        };
        let result = match cmd {
            IndiCmd::SetNumber { device, property, values } => {
                let items: Vec<(String, String)> =
                    values.into_iter().map(|(n, v)| (n, format!("{v}"))).collect();
                write_new_vector(&mut stream, "Number", &device, &property, &items)
            }
            IndiCmd::SetSwitch { device, property, values } => {
                let items: Vec<(String, String)> = values
                    .into_iter()
                    .map(|(n, on)| (n, if on { "On" } else { "Off" }.to_string()))
                    .collect();
                write_new_vector(&mut stream, "Switch", &device, &property, &items)
            }
            IndiCmd::SetText { device, property, values } => {
                write_new_vector(&mut stream, "Text", &device, &property, &values)
            }
            IndiCmd::EnableBlob { device, mode } => {
                // On INDIGO, upgrade any BLOB opt-in to URL mode: frames are
                // then fetched as raw binary over HTTP instead of inline
                // base64 (see module docs).
                let mode_text = if mode != BlobMode::Never
                    && shared.indigo.load(Ordering::Relaxed)
                {
                    "URL"
                } else {
                    mode.as_str()
                };
                stream.write_all(
                    format!(
                        "<enableBLOB device=\"{}\">{}</enableBLOB>\n",
                        xml_escape(&device),
                        mode_text
                    )
                    .as_bytes(),
                )
            }
            IndiCmd::Connect { device } => {
                let (connect, _) = connection_items(shared.indigo.load(Ordering::Relaxed));
                write_new_vector(
                    &mut stream,
                    "Switch",
                    &device,
                    PROP_CONNECTION,
                    &[(connect.to_string(), "On".to_string())],
                )
            }
            IndiCmd::StartExposure { device, seconds, live } => {
                *shared.exposure_s.lock().unwrap() = seconds;
                *shared.live_device.lock().unwrap() = device.clone();
                shared.live.store(live, Ordering::Relaxed);
                let item = exposure_item(shared.indigo.load(Ordering::Relaxed));
                write_new_vector(
                    &mut stream,
                    "Number",
                    &device,
                    PROP_EXPOSURE,
                    &[(item.to_string(), format!("{seconds}"))],
                )
            }
            IndiCmd::StopLive => {
                shared.live.store(false, Ordering::Relaxed);
                Ok(())
            }
            IndiCmd::Stop => unreachable!(),
        };
        if let Err(e) = result {
            let _ = log_tx.try_send(super::LogEntry::error(format!("INDI write: {e}")));
            let _ = stream.shutdown(Shutdown::Both);
            return;
        }
    }
}

/// Emit `<newNumberVector device=.. name=..><oneNumber name=..>v</oneNumber>…`
fn write_new_vector(
    stream: &mut TcpStream,
    kind: &str,
    device: &str,
    property: &str,
    items: &[(String, String)],
) -> std::io::Result<()> {
    let mut xml = format!(
        "<new{kind}Vector device=\"{}\" name=\"{}\">\n",
        xml_escape(device),
        xml_escape(property)
    );
    for (name, value) in items {
        xml.push_str(&format!(
            "  <one{kind} name=\"{}\">{}</one{kind}>\n",
            xml_escape(name),
            xml_escape(value)
        ));
    }
    xml.push_str(&format!("</new{kind}Vector>\n"));
    stream.write_all(xml.as_bytes())
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

// ── Reader thread: XML → property store / frames ────────────────────────────

/// One parsed child element of a def*/set* vector (`defNumber`, `oneBLOB`, …).
struct RawChild {
    tag: String,
    attrs: HashMap<String, String>,
    text: String,
}

fn reader_loop(
    stream: TcpStream,
    server_addr: String,
    frame_tx: Sender<super::FrameData>,
    props_slot: Arc<Mutex<Option<Vec<IndiProperty>>>>,
    cmd_tx: Sender<IndiCmd>,
    shared: Arc<SharedState>,
    log_tx: Sender<super::LogEntry>,
) {
    let mut reader = Reader::from_reader(BufReader::with_capacity(1 << 16, stream));
    reader.config_mut().trim_text(true);
    // The INDI stream is a sequence of top-level elements with no root, which
    // is fine for quick-xml's event reader but means we never see a "document".
    let mut store: HashMap<(String, String), IndiProperty> = HashMap::new();
    let mut buf = Vec::new();

    loop {
        buf.clear();
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attr_map(&e);
                let children = match read_children(&mut reader, &tag) {
                    Ok(c) => c,
                    Err(e) => {
                        let _ = log_tx.try_send(super::LogEntry::error(format!("INDI parse: {e}")));
                        return;
                    }
                };
                handle_element(
                    &tag, attrs, children, &mut store, &server_addr, &frame_tx, &props_slot,
                    &cmd_tx, &shared, &log_tx,
                );
            }
            Ok(Event::Empty(e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attr_map(&e);
                handle_element(
                    &tag, attrs, Vec::new(), &mut store, &server_addr, &frame_tx, &props_slot,
                    &cmd_tx, &shared, &log_tx,
                );
            }
            Ok(Event::Eof) => {
                let _ = log_tx.try_send(super::LogEntry::info("INDI server disconnected".into()));
                return;
            }
            Ok(_) => {}
            Err(e) => {
                // Also the normal exit path: Stop shuts the socket down under us.
                let _ = log_tx.try_send(super::LogEntry::info(format!("INDI reader exit: {e}")));
                return;
            }
        }
    }
}

/// Read the children of a vector element until its matching end tag.
fn read_children(
    reader: &mut Reader<BufReader<TcpStream>>,
    parent_tag: &str,
) -> Result<Vec<RawChild>> {
    let mut children = Vec::new();
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match reader.read_event_into(&mut buf)? {
            Event::Start(e) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attr_map(&e);
                // Collect text (may be huge for oneBLOB) until the child's end tag.
                let mut text = String::new();
                let mut tbuf = Vec::new();
                loop {
                    tbuf.clear();
                    match reader.read_event_into(&mut tbuf)? {
                        Event::Text(t) => {
                            text.push_str(&t.unescape().unwrap_or_default());
                        }
                        Event::End(_) => break,
                        Event::Eof => bail!("EOF inside <{tag}>"),
                        _ => {}
                    }
                }
                children.push(RawChild { tag, attrs, text });
            }
            Event::Empty(e) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let attrs = attr_map(&e);
                children.push(RawChild { tag, attrs, text: String::new() });
            }
            Event::End(_) => return Ok(children),
            Event::Eof => bail!("EOF inside <{parent_tag}>"),
            _ => {}
        }
    }
}

fn attr_map(e: &BytesStart) -> HashMap<String, String> {
    e.attributes()
        .flatten()
        .map(|a| {
            (
                String::from_utf8_lossy(a.key.as_ref()).into_owned(),
                a.unescape_value().unwrap_or_default().into_owned(),
            )
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn handle_element(
    tag: &str,
    attrs: HashMap<String, String>,
    children: Vec<RawChild>,
    store: &mut HashMap<(String, String), IndiProperty>,
    server_addr: &str,
    frame_tx: &Sender<super::FrameData>,
    props_slot: &Mutex<Option<Vec<IndiProperty>>>,
    cmd_tx: &Sender<IndiCmd>,
    shared: &SharedState,
    log_tx: &Sender<super::LogEntry>,
) {
    let get = |k: &str| attrs.get(k).cloned().unwrap_or_default();
    match tag {
        // INDIGO accepted the 2.0 extension offered in getProperties.
        "switchProtocol" => {
            shared.indigo.store(true, Ordering::Relaxed);
            let _ = log_tx.try_send(super::LogEntry::info(
                "INDIGO protocol 2.0 negotiated — raw (non-base64) BLOB transfer enabled".into(),
            ));
        }
        "defNumberVector" | "defSwitchVector" | "defTextVector" | "defLightVector"
        | "defBLOBVector" => {
            let prop = IndiProperty {
                device: get("device"),
                name: get("name"),
                label: {
                    let l = get("label");
                    if l.is_empty() { get("name") } else { l }
                },
                group: get("group"),
                state: PropState::parse(&get("state")),
                perm: PropPerm::parse(&get("perm")),
                rule: (tag == "defSwitchVector").then(|| SwitchRule::parse(&get("rule"))),
                elements: children.iter().map(parse_def_element).collect(),
            };
            store.insert((prop.device.clone(), prop.name.clone()), prop);
            push_snapshot(store, props_slot);
        }
        "setNumberVector" | "setSwitchVector" | "setTextVector" | "setLightVector"
        | "setBLOBVector" => {
            let key = (get("device"), get("name"));
            // BLOBs: decode straight to the frame channel; don't hold pixels
            // in the property store.
            for child in children.iter().filter(|c| c.tag == "oneBLOB") {
                handle_blob(child, server_addr, frame_tx, cmd_tx, shared, log_tx);
            }
            if let Some(prop) = store.get_mut(&key) {
                if let Some(s) = attrs.get("state") {
                    prop.state = PropState::parse(s);
                }
                for child in &children {
                    let name = child.attrs.get("name").cloned().unwrap_or_default();
                    if let Some(el) = prop.elements.iter_mut().find(|el| el.name == name) {
                        update_element(el, child);
                    }
                }
                push_snapshot(store, props_slot);
            }
        }
        "delProperty" => {
            let device = get("device");
            let name = get("name");
            if name.is_empty() {
                store.retain(|(d, _), _| *d != device);
            } else {
                store.remove(&(device, name));
            }
            push_snapshot(store, props_slot);
        }
        "message" => {
            let msg = get("message");
            if !msg.is_empty() {
                let device = get("device");
                let text = if device.is_empty() { msg } else { format!("{device}: {msg}") };
                let _ = log_tx.try_send(super::LogEntry::info(text));
            }
        }
        _ => {}
    }
}

fn parse_def_element(child: &RawChild) -> IndiElement {
    let get = |k: &str| child.attrs.get(k).cloned().unwrap_or_default();
    let name = get("name");
    let label = {
        let l = get("label");
        if l.is_empty() { name.clone() } else { l }
    };
    let num = |k: &str| child.attrs.get(k).and_then(|v| parse_indi_number(v)).unwrap_or(0.0);
    let value = match child.tag.as_str() {
        "defNumber" => IndiValue::Number {
            value: parse_indi_number(&child.text).unwrap_or(0.0),
            min: num("min"),
            max: num("max"),
            step: num("step"),
            format: get("format"),
        },
        "defSwitch" => IndiValue::Switch(child.text.trim() == "On"),
        "defLight" => IndiValue::Light(PropState::parse(child.text.trim())),
        "defBLOB" => IndiValue::Blob { format: get("format"), size: 0 },
        _ => IndiValue::Text(child.text.clone()),
    };
    IndiElement { name, label, value }
}

fn update_element(el: &mut IndiElement, child: &RawChild) {
    match &mut el.value {
        IndiValue::Number { value, .. } => {
            if let Some(v) = parse_indi_number(&child.text) {
                *value = v;
            }
        }
        IndiValue::Switch(on) => *on = child.text.trim() == "On",
        IndiValue::Text(t) => *t = child.text.clone(),
        IndiValue::Light(s) => *s = PropState::parse(child.text.trim()),
        IndiValue::Blob { format, size } => {
            if let Some(f) = child.attrs.get("format") {
                *format = f.clone();
            }
            *size = child.attrs.get("size").and_then(|s| s.parse().ok()).unwrap_or(0);
        }
    }
}

/// Publish the current property snapshot into the UI's mailbox slot,
/// replacing any unconsumed older one.
fn push_snapshot(
    store: &HashMap<(String, String), IndiProperty>,
    props_slot: &Mutex<Option<Vec<IndiProperty>>>,
) {
    let mut snapshot: Vec<IndiProperty> = store.values().cloned().collect();
    snapshot.sort_by(|a, b| (&a.device, &a.group, &a.name).cmp(&(&b.device, &b.group, &b.name)));
    *props_slot.lock().unwrap() = Some(snapshot);
}

// ── BLOB → FrameData ────────────────────────────────────────────────────────

fn handle_blob(
    child: &RawChild,
    server_addr: &str,
    frame_tx: &Sender<super::FrameData>,
    cmd_tx: &Sender<IndiCmd>,
    shared: &SharedState,
    log_tx: &Sender<super::LogEntry>,
) {
    // INDIGO URL mode: the element carries a `url` (absolute) or `path`
    // (server-relative) attribute and no inline data — fetch raw binary.
    let by_ref = child.attrs.get("url").or_else(|| child.attrs.get("path"));
    let result = if let Some(loc) = by_ref {
        http_fetch(server_addr, loc).and_then(|bytes| decode_fits_bytes(&bytes))
    } else {
        let format = child.attrs.get("format").map(String::as_str).unwrap_or("");
        if !format.contains("fits") {
            let _ = log_tx.try_send(super::LogEntry::error(format!(
                "INDI: unsupported BLOB format {format:?} (only FITS is handled)"
            )));
            return;
        }
        decode_fits_blob(&child.text)
    };
    match result {
        Ok(frame) => {
            let _ = frame_tx.try_send(frame);
        }
        Err(e) => {
            let _ = log_tx.try_send(super::LogEntry::error(format!("INDI BLOB decode: {e}")));
        }
    }
    // Live view: an INDI exposure is one-shot, so trigger the next one now.
    if shared.live.load(Ordering::Relaxed) {
        let device = shared.live_device.lock().unwrap().clone();
        let seconds = *shared.exposure_s.lock().unwrap();
        let item = exposure_item(shared.indigo.load(Ordering::Relaxed));
        let _ = cmd_tx.try_send(IndiCmd::SetNumber {
            device,
            property: PROP_EXPOSURE.to_string(),
            values: vec![(item.to_string(), seconds)],
        });
    }
}

/// Minimal HTTP/1.1 GET returning the response body — used only to pull
/// INDIGO BLOBs, which the server exposes at `http://host:port/blob/…`.
/// `loc` is either an absolute `http://…` URL or a server-relative path.
fn http_fetch(server_addr: &str, loc: &str) -> Result<Vec<u8>> {
    use std::io::Read;

    let (host_port, path) = if let Some(rest) = loc.strip_prefix("http://") {
        match rest.split_once('/') {
            Some((hp, p)) => (hp.to_string(), format!("/{p}")),
            None => (rest.to_string(), "/".to_string()),
        }
    } else {
        (server_addr.to_string(), loc.to_string())
    };
    let host_port = if host_port.contains(':') { host_port } else { format!("{host_port}:80") };

    let mut stream = TcpStream::connect(&host_port)
        .map_err(|e| anyhow!("BLOB fetch connect {host_port}: {e}"))?;
    stream.set_read_timeout(Some(std::time::Duration::from_secs(15))).ok();
    stream.write_all(
        format!("GET {path} HTTP/1.1\r\nHost: {host_port}\r\nConnection: close\r\n\r\n")
            .as_bytes(),
    )?;

    let mut response = Vec::new();
    stream.read_to_end(&mut response)?;
    let header_end = response
        .windows(4)
        .position(|w| w == b"\r\n\r\n")
        .ok_or_else(|| anyhow!("BLOB fetch: malformed HTTP response"))?;
    let header = String::from_utf8_lossy(&response[..header_end]).into_owned();
    let status_ok = header.lines().next().is_some_and(|l| l.contains(" 200 "));
    if !status_ok {
        bail!("BLOB fetch {path}: {}", header.lines().next().unwrap_or("no status"));
    }
    let mut body = response.split_off(header_end + 4);
    // Trust Content-Length when present (Connection: close bounds it anyway).
    let content_length = header.lines().find_map(|l| {
        l.to_ascii_lowercase()
            .strip_prefix("content-length:")
            .and_then(|v| v.trim().parse::<usize>().ok())
    });
    if let Some(len) = content_length {
        body.truncate(len);
    }
    Ok(body)
}

/// Base64 text → FITS → mono `FrameData` (first image HDU).
fn decode_fits_blob(b64: &str) -> Result<super::FrameData> {
    use base64::Engine;
    // Servers wrap base64 in newlines; strip all whitespace before decoding.
    let cleaned: Vec<u8> = b64.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    let data = base64::engine::general_purpose::STANDARD.decode(&cleaned)?;
    decode_fits_bytes(&data)
}

/// Raw FITS bytes → mono `FrameData` (first image HDU).
fn decode_fits_bytes(data: &[u8]) -> Result<super::FrameData> {
    let fits = fitskit::FitsFile::from_bytes(data)?;
    for hdu in fits.iter() {
        let img = match &hdu.data {
            fitskit::HduData::Image(im) if im.axes.len() >= 2 => im,
            _ => continue,
        };
        let width = img.axes[0] as u32;
        let height = img.axes[1] as u32;
        let bscale = hdu.header.get_float("BSCALE").unwrap_or(1.0);
        let bzero = hdu.header.get_float("BZERO").unwrap_or(0.0);
        let pixels = img.scaled_values(bscale, bzero);
        let npix = (width as usize) * (height as usize);
        if pixels.len() < npix {
            continue;
        }
        let mono: Vec<f32> = pixels[..npix].iter().map(|&v| v as f32).collect();
        let max_val = mono.iter().copied().fold(0.0_f32, f32::max);
        let bit_depth = if max_val <= 255.0 { 8 }
            else if max_val <= 4095.0 { 12 }
            else if max_val <= 16383.0 { 14 }
            else { 16 };
        return Ok(super::FrameData::new(mono, width, height, bit_depth));
    }
    bail!("no image HDU in BLOB")
}

// ── Number parsing ──────────────────────────────────────────────────────────

/// Parse an INDI number: plain float or sexagesimal ("12:30:45", "12 30 45").
fn parse_indi_number(s: &str) -> Option<f64> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    if let Ok(v) = s.parse::<f64>() {
        return Some(v);
    }
    let neg = s.starts_with('-');
    let parts: Option<Vec<f64>> = s
        .split([':', ' '])
        .filter(|p| !p.is_empty())
        .map(|p| p.parse::<f64>().ok())
        .collect();
    let parts = parts?;
    if parts.is_empty() || parts.len() > 3 {
        return None;
    }
    let mut val = 0.0;
    for (i, p) in parts.iter().enumerate() {
        val += p.abs() / 60f64.powi(i as i32);
    }
    Some(if neg { -val } else { val })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::time::Duration;

    #[test]
    fn numbers() {
        assert_eq!(parse_indi_number("1.5"), Some(1.5));
        assert_eq!(parse_indi_number("-12:30"), Some(-12.5));
        assert_eq!(parse_indi_number("12 30 36"), Some(12.51));
        assert_eq!(parse_indi_number(""), None);
    }

    /// 80-char FITS header card.
    fn card(s: &str) -> Vec<u8> {
        let mut c = s.as_bytes().to_vec();
        c.resize(80, b' ');
        c
    }

    /// Minimal valid FITS: 4x4 image, 16-bit, values 100..=115.
    fn tiny_fits() -> Vec<u8> {
        let mut fits = Vec::new();
        for s in [
            "SIMPLE  =                    T",
            "BITPIX  =                   16",
            "NAXIS   =                    2",
            "NAXIS1  =                    4",
            "NAXIS2  =                    4",
            "END",
        ] {
            fits.extend(card(s));
        }
        fits.resize(2880, b' ');
        for i in 0..16i16 {
            fits.extend((100 + i).to_be_bytes());
        }
        fits.resize(2880 * 2, 0);
        fits
    }

    /// End-to-end against a fake in-process INDI server: property definitions
    /// land in the snapshot channel, and a FITS BLOB decodes to a FrameData.
    #[test]
    fn client_receives_props_and_frame() {
        use base64::Engine;

        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();

        let server = std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            // Consume the client's getProperties before pushing.
            let mut buf = [0u8; 1024];
            let _ = sock.read(&mut buf);

            let blob = base64::engine::general_purpose::STANDARD.encode(tiny_fits());
            let xml = format!(
                concat!(
                    "<defSwitchVector device=\"Test CCD\" name=\"CONNECTION\" ",
                    "label=\"Connection\" group=\"Main Control\" state=\"Idle\" ",
                    "perm=\"rw\" rule=\"OneOfMany\">\n",
                    "  <defSwitch name=\"CONNECT\" label=\"Connect\">Off</defSwitch>\n",
                    "  <defSwitch name=\"DISCONNECT\" label=\"Disconnect\">On</defSwitch>\n",
                    "</defSwitchVector>\n",
                    "<defNumberVector device=\"Test CCD\" name=\"CCD_EXPOSURE\" ",
                    "label=\"Expose\" group=\"Main Control\" state=\"Idle\" perm=\"rw\">\n",
                    "  <defNumber name=\"CCD_EXPOSURE_VALUE\" label=\"Duration (s)\" ",
                    "format=\"%5.2f\" min=\"0\" max=\"3600\" step=\"0.1\">1.0</defNumber>\n",
                    "</defNumberVector>\n",
                    "<setBLOBVector device=\"Test CCD\" name=\"CCD1\" state=\"Ok\">\n",
                    "  <oneBLOB name=\"CCD1\" size=\"{}\" format=\".fits\">{}</oneBLOB>\n",
                    "</setBLOBVector>\n",
                ),
                blob.len(),
                blob
            );
            sock.write_all(xml.as_bytes()).unwrap();
            // Hold the connection open until the client shuts it down.
            while sock.read(&mut buf).map(|n| n > 0).unwrap_or(false) {}
        });

        let (frame_tx, frame_rx) = bounded(4);
        let (log_tx, _log_rx) = bounded(64);
        let mut handle = start_client("127.0.0.1", port, frame_tx, log_tx).unwrap();

        // Property snapshots: wait until both definitions have arrived.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let mut props = Vec::new();
        while props.len() < 2 && std::time::Instant::now() < deadline {
            if let Some(snap) = handle.props.lock().unwrap().take() {
                props = snap;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let conn = props.iter().find(|p| p.name == "CONNECTION").unwrap();
        assert_eq!(conn.device, "Test CCD");
        assert_eq!(conn.rule, Some(SwitchRule::OneOfMany));
        assert!(conn.elements.iter().any(|el| {
            el.name == "DISCONNECT" && matches!(el.value, IndiValue::Switch(true))
        }));
        let exp = props.iter().find(|p| p.name == "CCD_EXPOSURE").unwrap();
        assert!(matches!(exp.elements[0].value,
            IndiValue::Number { value, max, .. } if value == 1.0 && max == 3600.0));

        // BLOB → FrameData.
        let frame = frame_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!((frame.width, frame.height), (4, 4));
        assert_eq!(frame.mono[0], 100.0);
        assert_eq!(frame.mono[15], 115.0);

        handle.stop();
        server.join().unwrap();
    }

    /// INDIGO variant: the server accepts the 2.0 handshake, the client
    /// upgrades enableBLOB to URL mode, and the frame is fetched raw over
    /// HTTP — no base64 anywhere.
    #[test]
    fn indigo_url_blob_roundtrip() {
        // Mini HTTP server: one GET, replies with the FITS bytes.
        let http = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let http_port = http.local_addr().unwrap().port();
        let http_thread = std::thread::spawn(move || {
            let (mut sock, _) = http.accept().unwrap();
            let mut buf = [0u8; 2048];
            let _ = sock.read(&mut buf);
            let body = tiny_fits();
            let head = format!(
                "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                body.len()
            );
            sock.write_all(head.as_bytes()).unwrap();
            sock.write_all(&body).unwrap();
        });

        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let server = std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            let mut buf = [0u8; 2048];
            let n = sock.read(&mut buf).unwrap();
            let hello = String::from_utf8_lossy(&buf[..n]).into_owned();
            assert!(hello.contains("switch=\"2.0\""), "client must offer INDIGO: {hello}");
            // Accept the INDIGO protocol.
            sock.write_all(b"<switchProtocol version='2.0'/>\n").unwrap();

            // Wait for the client's enableBLOB — must be upgraded to URL mode.
            let mut req = String::new();
            while !req.contains("</enableBLOB>") && !req.contains("enableBLOB") {
                let n = sock.read(&mut buf).unwrap();
                if n == 0 { panic!("client closed before enableBLOB"); }
                req.push_str(&String::from_utf8_lossy(&buf[..n]));
            }
            assert!(req.contains(">URL<"), "expected URL BLOB mode, got: {req}");

            // Announce the frame by URL (absolute form, INDIGO style).
            let xml = format!(
                concat!(
                    "<setBLOBVector device=\"Test CCD\" name=\"CCD_IMAGE\" state=\"Ok\">\n",
                    "  <oneBLOB name=\"IMAGE\" url=\"http://127.0.0.1:{}/blob/0x1.fits?1\"/>\n",
                    "</setBLOBVector>\n",
                ),
                http_port
            );
            sock.write_all(xml.as_bytes()).unwrap();
            while sock.read(&mut buf).map(|n| n > 0).unwrap_or(false) {}
        });

        let (frame_tx, frame_rx) = bounded(4);
        let (log_tx, _log_rx) = bounded(64);
        let mut handle = start_client("127.0.0.1", port, frame_tx, log_tx).unwrap();

        // Give the reader a moment to process switchProtocol, then opt in.
        std::thread::sleep(Duration::from_millis(200));
        handle
            .cmd_tx
            .send(IndiCmd::EnableBlob { device: "Test CCD".into(), mode: BlobMode::Also })
            .unwrap();

        let frame = frame_rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert_eq!((frame.width, frame.height), (4, 4));
        assert_eq!(frame.mono[0], 100.0);
        assert_eq!(frame.mono[15], 115.0);

        handle.stop();
        server.join().unwrap();
        http_thread.join().unwrap();
    }

    /// End-to-end against a *real* INDI or INDIGO server — ignored by default
    /// since it needs one running. Start e.g.:
    ///   indigo_server indigo_ccd_simulator
    /// then:
    ///   cargo test --features indi -- --ignored live_server
    /// Env: INDI_TEST_ADDR (default 127.0.0.1:7624), INDI_TEST_DEVICE
    /// (default: first device defining CCD_EXPOSURE after connect).
    #[test]
    #[ignore]
    fn live_server_frames() {
        let addr = std::env::var("INDI_TEST_ADDR").unwrap_or_else(|_| "127.0.0.1:7624".into());
        let (host, port) = match addr.rsplit_once(':') {
            Some((h, p)) if p.parse::<u16>().is_ok() => (h.to_string(), p.parse().unwrap()),
            _ => (addr.clone(), DEFAULT_PORT),
        };

        let (frame_tx, frame_rx) = bounded(8);
        let (log_tx, log_rx) = bounded(1024);
        let mut handle = start_client(&host, port, frame_tx, log_tx).expect("connect");

        let dump_logs = |log_rx: &Receiver<crate::LogEntry>| {
            while let Ok(e) = log_rx.try_recv() {
                eprintln!("[log] {}", e.message);
            }
        };

        // Collect devices as definitions stream in; connect the CCD.
        let want_device = std::env::var("INDI_TEST_DEVICE").ok();
        let deadline = std::time::Instant::now() + Duration::from_secs(20);
        let mut props: Vec<IndiProperty> = Vec::new();
        let mut device: Option<String> = None;
        let mut connect_sent = false;
        let mut exposure_started = false;
        let mut frames = 0u32;
        let mut dims = (0u32, 0u32);

        while std::time::Instant::now() < deadline && frames < 3 {
            if let Some(snap) = handle.props.lock().unwrap().take() {
                props = snap;
            }
            dump_logs(&log_rx);

            if device.is_none() {
                // Prefer the requested device, else anything with a CONNECTION
                // property whose name suggests a camera, else the first device.
                let candidates: Vec<&IndiProperty> =
                    props.iter().filter(|p| p.name == PROP_CONNECTION).collect();
                device = match &want_device {
                    Some(w) => candidates.iter().find(|p| p.device == *w).map(|p| p.device.clone()),
                    None => candidates
                        .iter()
                        .find(|p| {
                            let d = p.device.to_ascii_lowercase();
                            d.contains("ccd") || d.contains("imager") || d.contains("camera")
                        })
                        .or(candidates.first())
                        .map(|p| p.device.clone()),
                };
            }
            if let Some(dev) = &device {
                if !connect_sent {
                    eprintln!("[test] connecting device {dev:?}");
                    handle.cmd_tx.send(IndiCmd::Connect { device: dev.clone() }).unwrap();
                    handle
                        .cmd_tx
                        .send(IndiCmd::EnableBlob { device: dev.clone(), mode: BlobMode::Also })
                        .unwrap();
                    connect_sent = true;
                }
                // CCD_EXPOSURE only appears once the driver is connected.
                let has_exposure =
                    props.iter().any(|p| p.device == *dev && p.name == PROP_EXPOSURE);
                if has_exposure && !exposure_started {
                    eprintln!("[test] starting 0.2 s live exposures");
                    handle
                        .cmd_tx
                        .send(IndiCmd::StartExposure { device: dev.clone(), seconds: 0.2, live: true })
                        .unwrap();
                    exposure_started = true;
                }
            }

            if let Ok(frame) = frame_rx.recv_timeout(Duration::from_millis(200)) {
                frames += 1;
                dims = (frame.width, frame.height);
                eprintln!(
                    "[test] frame {}: {}x{} mean {:.1}",
                    frames, frame.width, frame.height, frame.mean
                );
            }
        }

        let _ = handle.cmd_tx.send(IndiCmd::StopLive);
        dump_logs(&log_rx);
        handle.stop();

        assert!(connect_sent, "no INDI device discovered within the deadline");
        assert!(exposure_started, "CCD_EXPOSURE never appeared (device did not connect?)");
        assert!(frames >= 2, "expected repeated live frames, got {frames}");
        assert!(dims.0 > 0 && dims.1 > 0);
    }
}
