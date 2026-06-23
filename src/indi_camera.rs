//! Blocking INDI-protocol camera backend.
//!
//! INDI (Instrument-Neutral Distributed Interface) is an XML protocol spoken to
//! an `indiserver` over TCP (default port 7624). Unlike a USB SDK, the server
//! already abstracts the hardware, so a minimal CCD client is small and — unlike
//! the `indi` crate — can be fully *blocking*, which fits this app's
//! thread-plus-channel model with no async runtime.
//!
//! Flow: connect → `getProperties` → set `CONNECTION.CONNECT=On` → wait for the
//! CCD properties → `enableBLOB Also` → trigger an exposure → on each `CCD1`
//! BLOB, decode the FITS bytes into a [`FrameData`] and re-trigger the next
//! exposure. Stop is delivered by shutting the socket down from another thread,
//! which unblocks the capture read.
//!
//! NOTE: the BLOB transfer details (base64; `format=".fits.z"` ⇒ zlib) and the
//! exposure-as-trigger semantics are implemented to spec but have only been
//! validated against the protocol docs — they want a pass against a real
//! `indiserver` / `indi_simulator_ccd`.

use std::collections::BTreeMap;
use std::io::{BufReader, Read, Write};
use std::net::{Shutdown, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use base64::Engine;
use crossbeam_channel::{Receiver, Sender};
use quick_xml::events::{BytesStart, Event};
use quick_xml::reader::Reader;

use super::{FrameData, LogEntry};

/// Default INDI server port.
pub const DEFAULT_PORT: u16 = 7624;

// ── Public types ────────────────────────────────────────────────────────────

/// A discovered INDI CCD device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IndiDeviceInfo {
    pub device: String,
}

impl IndiDeviceInfo {
    pub fn display_name(&self) -> String {
        self.device.clone()
    }
}

/// One adjustable camera control, flattened to a single property element.
#[derive(Clone, Debug)]
pub struct IndiControl {
    pub property: String, // INDI vector name, e.g. "CCD_GAIN"
    pub element: String,  // element name, e.g. "GAIN"
    pub label: String,
    pub group: String,
    pub kind: IndiControlKind,
    pub writable: bool,
}

#[derive(Clone, Debug)]
pub enum IndiControlKind {
    /// A numeric value with an inclusive range. `step` is 0 when unspecified.
    Number { value: f64, min: f64, max: f64, step: f64 },
    /// A single on/off switch element.
    Switch { on: bool },
}

/// A request to change a control, routed through the running handle.
#[derive(Clone, Debug)]
pub enum IndiCmd {
    SetNumber(String, String, f64),
    SetSwitch(String, String, bool),
}

/// A live INDI camera. The capture thread owns the read side; control writes go
/// directly to the socket under `writer`.
pub struct IndiHandle {
    pub device: String,
    pub controls: Vec<IndiControl>,
    /// Fresh control snapshots pushed by the capture thread (value/state changes).
    pub controls_rx: Receiver<Vec<IndiControl>>,
    writer: Arc<Mutex<TcpStream>>,
    /// Desired exposure duration (seconds). Setting `CCD_EXPOSURE_VALUE` *starts*
    /// an exposure, so the duration is a setting and the capture loop owns the
    /// actual re-triggering.
    exposure_s: Arc<Mutex<f64>>,
    stop: Arc<AtomicBool>,
    shutdown_handle: TcpStream,
    join_handle: Option<std::thread::JoinHandle<()>>,
}

impl IndiHandle {
    /// Apply a control change. `CCD_EXPOSURE` updates the desired duration only
    /// (the loop re-triggers); everything else is written to the server now.
    pub fn set_control(&self, cmd: IndiCmd) {
        match cmd {
            IndiCmd::SetNumber(prop, _elem, v) if prop == "CCD_EXPOSURE" => {
                if let Ok(mut e) = self.exposure_s.lock() {
                    *e = v.max(0.0);
                }
            }
            IndiCmd::SetNumber(prop, elem, v) => {
                self.write_new_number(&prop, &elem, v);
            }
            IndiCmd::SetSwitch(prop, elem, on) => {
                self.write_new_switch(&prop, &elem, on);
            }
        }
    }

    fn write_new_number(&self, property: &str, element: &str, value: f64) {
        let xml = format!(
            "<newNumberVector device=\"{}\" name=\"{}\"><oneNumber name=\"{}\">{}</oneNumber></newNumberVector>",
            xml_escape(&self.device),
            xml_escape(property),
            xml_escape(element),
            value
        );
        self.send(&xml);
    }

    fn write_new_switch(&self, property: &str, element: &str, on: bool) {
        let xml = format!(
            "<newSwitchVector device=\"{}\" name=\"{}\"><oneSwitch name=\"{}\">{}</oneSwitch></newSwitchVector>",
            xml_escape(&self.device),
            xml_escape(property),
            xml_escape(element),
            if on { "On" } else { "Off" }
        );
        self.send(&xml);
    }

    fn send(&self, xml: &str) {
        if let Ok(mut w) = self.writer.lock() {
            let _ = w.write_all(xml.as_bytes());
            let _ = w.flush();
        }
    }

    /// Stop acquisition and join the capture thread. Shutting the socket down
    /// unblocks the capture read.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        let _ = self.shutdown_handle.shutdown(Shutdown::Both);
        if let Some(jh) = self.join_handle.take() {
            let _ = jh.join();
        }
    }
}

impl Drop for IndiHandle {
    fn drop(&mut self) {
        if self.join_handle.is_some() {
            self.stop();
        }
    }
}

// ── Discovery ───────────────────────────────────────────────────────────────

/// Connect briefly and list devices that expose a CCD (a `CCD_EXPOSURE` number
/// vector or a BLOB vector). Reads the initial property burst, then disconnects.
pub fn enumerate(host: &str, port: u16) -> Result<Vec<IndiDeviceInfo>> {
    let stream = TcpStream::connect((host, port))
        .with_context(|| format!("connect to INDI server {host}:{port}"))?;
    stream.set_read_timeout(Some(Duration::from_millis(400)))?;
    {
        let mut w = stream.try_clone()?;
        w.write_all(b"<getProperties version=\"1.7\"/>")?;
        w.flush()?;
    }

    // Drain the burst into a buffer (raw reads with a short timeout), then parse.
    // Parsing a complete buffer avoids quick-xml desyncing on a mid-element read
    // timeout.
    let mut data = Vec::new();
    let mut tmp = [0u8; 16384];
    let deadline = Instant::now() + Duration::from_millis(2000);
    let mut reader = BufReader::new(stream);
    loop {
        match reader.read(&mut tmp) {
            Ok(0) => break,
            Ok(n) => data.extend_from_slice(&tmp[..n]),
            Err(e) if is_timeout(&e) => break, // burst delivered
            Err(_) => break,
        }
        if Instant::now() > deadline {
            break;
        }
    }

    let mut ccds: BTreeMap<String, IndiDeviceInfo> = BTreeMap::new();
    let mut xr = Reader::from_reader(&data[..]);
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match xr.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) | Ok(Event::Empty(e)) => {
                let tag = e.name();
                let tag = tag.as_ref();
                if tag == b"defBLOBVector" || tag == b"defNumberVector" {
                    let device = attr(&e, b"device");
                    let name = attr(&e, b"name");
                    let is_ccd = tag == b"defBLOBVector" || name.as_deref() == Some("CCD_EXPOSURE");
                    if is_ccd {
                        if let Some(d) = device {
                            ccds.entry(d.clone()).or_insert(IndiDeviceInfo { device: d });
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
    Ok(ccds.into_values().collect())
}

// ── Start / capture ─────────────────────────────────────────────────────────

/// Open `device` on the given server and begin streaming frames.
pub fn start_camera(
    host: &str,
    port: u16,
    device: &str,
    frame_tx: Sender<FrameData>,
    log_tx: Sender<LogEntry>,
) -> Result<IndiHandle> {
    let stream = TcpStream::connect((host, port))
        .with_context(|| format!("connect to INDI server {host}:{port}"))?;
    let writer = Arc::new(Mutex::new(stream.try_clone()?));
    let shutdown_handle = stream.try_clone()?;

    // Handshake uses a generous read timeout so a non-responsive server fails
    // instead of hanging; steady-state capture clears it (exposures can be long).
    stream.set_read_timeout(Some(Duration::from_secs(15)))?;
    let mut reader = Reader::from_reader(BufReader::new(stream.try_clone()?));

    // Ask for this device's properties.
    send(&writer, &format!(
        "<getProperties version=\"1.7\" device=\"{}\"/>",
        xml_escape(device)
    ));

    let mut controls: Vec<IndiControl> = Vec::new();
    let mut connect_sent = false;
    let mut have_exposure = false;
    let mut buf = Vec::new();

    // Handshake: connect the driver, then collect its number controls until the
    // exposure property appears.
    let hs_deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < hs_deadline && !have_exposure {
        buf.clear();
        let ev = reader
            .read_event_into(&mut buf)
            .map_err(|e| anyhow!("INDI read during handshake: {e}"))?;
        match ev {
            Event::Eof => return Err(anyhow!("INDI server closed during handshake")),
            Event::Start(e) => {
                let tag = e.name().as_ref().to_vec();
                let dev = attr(&e, b"device");
                let name = attr(&e, b"name");
                let group = attr(&e, b"group").unwrap_or_default();
                let children = read_children(&mut reader, &mut buf, &tag)?;
                if dev.as_deref() != Some(device) {
                    continue;
                }
                match tag.as_slice() {
                    b"defSwitchVector" if name.as_deref() == Some("CONNECTION") => {
                        let connected = children.iter().any(|c| {
                            c.attrs.get("name").map(String::as_str) == Some("CONNECT")
                                && c.text.trim().eq_ignore_ascii_case("On")
                        });
                        if !connected && !connect_sent {
                            send(&writer, &format!(
                                "<newSwitchVector device=\"{}\" name=\"CONNECTION\"><oneSwitch name=\"CONNECT\">On</oneSwitch></newSwitchVector>",
                                xml_escape(device)
                            ));
                            connect_sent = true;
                        }
                    }
                    b"defNumberVector" => {
                        let prop = name.clone().unwrap_or_default();
                        for c in &children {
                            if c.tag != b"defNumber" {
                                continue;
                            }
                            if let Some(ctrl) = number_control(&prop, &group, c) {
                                if ctrl.property == "CCD_EXPOSURE" {
                                    have_exposure = true;
                                }
                                upsert(&mut controls, ctrl);
                            }
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }

    if !have_exposure {
        return Err(anyhow!(
            "device '{device}' did not expose CCD_EXPOSURE (is it a CCD, and did it connect?)"
        ));
    }

    let _ = log_tx.send(LogEntry::info(format!("INDI: connected to {device}")));

    // Enable BLOBs for this device and clear the handshake timeout.
    send(&writer, &format!(
        "<enableBLOB device=\"{}\">Also</enableBLOB>",
        xml_escape(device)
    ));
    stream.set_read_timeout(None)?;

    // Initial exposure duration: the current CCD_EXPOSURE_VALUE if sane, else 1s.
    let exposure0 = controls
        .iter()
        .find(|c| c.property == "CCD_EXPOSURE")
        .and_then(|c| match c.kind {
            IndiControlKind::Number { value, .. } if value > 0.0 => Some(value),
            _ => None,
        })
        .unwrap_or(1.0);
    let exposure_s = Arc::new(Mutex::new(exposure0));

    // Trigger the first exposure.
    write_exposure(&writer, device, exposure0);

    let (controls_tx, controls_rx) = crossbeam_channel::bounded(4);
    let stop = Arc::new(AtomicBool::new(false));

    let thread_writer = Arc::clone(&writer);
    let thread_exposure = Arc::clone(&exposure_s);
    let thread_stop = Arc::clone(&stop);
    let device_owned = device.to_string();
    let init_controls = controls.clone();
    let join_handle = std::thread::Builder::new()
        .name("indi-capture".into())
        .spawn(move || {
            let res = capture_loop(
                reader,
                thread_writer,
                device_owned,
                thread_exposure,
                init_controls,
                frame_tx,
                controls_tx,
                &log_tx,
                thread_stop.clone(),
            );
            if let Err(e) = res {
                if !thread_stop.load(Ordering::SeqCst) {
                    let _ = log_tx.send(LogEntry::error(format!("INDI capture stopped: {e}")));
                }
            }
        })?;

    Ok(IndiHandle {
        device: device.to_string(),
        controls,
        controls_rx,
        writer,
        exposure_s,
        stop,
        shutdown_handle,
        join_handle: Some(join_handle),
    })
}

#[allow(clippy::too_many_arguments)]
fn capture_loop(
    mut reader: Reader<BufReader<TcpStream>>,
    writer: Arc<Mutex<TcpStream>>,
    device: String,
    exposure_s: Arc<Mutex<f64>>,
    mut controls: Vec<IndiControl>,
    frame_tx: Sender<FrameData>,
    controls_tx: Sender<Vec<IndiControl>>,
    log_tx: &Sender<LogEntry>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let mut buf = Vec::new();
    loop {
        if stop.load(Ordering::SeqCst) {
            return Ok(());
        }
        buf.clear();
        let ev = match reader.read_event_into(&mut buf) {
            Ok(ev) => ev,
            Err(e) => {
                if stop.load(Ordering::SeqCst) {
                    return Ok(());
                }
                return Err(anyhow!("INDI read: {e}"));
            }
        };
        match ev {
            Event::Eof => return Ok(()),
            Event::Start(e) => {
                let tag = e.name().as_ref().to_vec();
                let dev = attr(&e, b"device");
                let name = attr(&e, b"name");
                let group = attr(&e, b"group").unwrap_or_default();
                // `e` borrows `buf`; extract everything above before read_children
                // takes `buf` mutably.
                let children = read_children(&mut reader, &mut buf, &tag)?;
                if dev.as_deref() != Some(device.as_str()) {
                    continue;
                }
                match tag.as_slice() {
                    b"setBLOBVector" => {
                        for c in &children {
                            if c.tag != b"oneBLOB" {
                                continue;
                            }
                            let format = c.attrs.get("format").map(String::as_str).unwrap_or("");
                            match decode_blob(&c.text, format) {
                                Ok(frame) => {
                                    if frame_tx.try_send(frame).is_err() {
                                        return Ok(()); // UI gone
                                    }
                                }
                                Err(e) => {
                                    let _ = log_tx
                                        .send(LogEntry::error(format!("INDI BLOB decode: {e}")));
                                }
                            }
                        }
                        // Re-trigger the next exposure at the current duration.
                        let dur = exposure_s.lock().map(|g| *g).unwrap_or(1.0);
                        write_exposure(&writer, &device, dur);
                    }
                    b"setNumberVector" => {
                        let prop = name.unwrap_or_default();
                        let mut changed = false;
                        for c in &children {
                            if c.tag != b"oneNumber" {
                                continue;
                            }
                            if let (Some(elem), Ok(v)) =
                                (c.attrs.get("name"), c.text.trim().parse::<f64>())
                            {
                                for ctrl in controls.iter_mut() {
                                    if ctrl.property == prop && &ctrl.element == elem {
                                        if let IndiControlKind::Number { value, .. } = &mut ctrl.kind {
                                            *value = v;
                                            changed = true;
                                        }
                                    }
                                }
                            }
                        }
                        if changed {
                            let _ = controls_tx.try_send(controls.clone());
                        }
                    }
                    b"defNumberVector" => {
                        // A property defined after the handshake — fold it in.
                        let prop = name.unwrap_or_default();
                        let mut changed = false;
                        for c in &children {
                            if c.tag == b"defNumber" {
                                if let Some(ctrl) = number_control(&prop, &group, c) {
                                    upsert(&mut controls, ctrl);
                                    changed = true;
                                }
                            }
                        }
                        if changed {
                            let _ = controls_tx.try_send(controls.clone());
                        }
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
}

/// Set `CCD_EXPOSURE_VALUE`, which starts an exposure of `seconds`.
fn write_exposure(writer: &Arc<Mutex<TcpStream>>, device: &str, seconds: f64) {
    let xml = format!(
        "<newNumberVector device=\"{}\" name=\"CCD_EXPOSURE\"><oneNumber name=\"CCD_EXPOSURE_VALUE\">{}</oneNumber></newNumberVector>",
        xml_escape(device),
        seconds
    );
    send(writer, &xml);
}

// ── BLOB / FITS decoding ────────────────────────────────────────────────────

/// Decode a base64 `<oneBLOB>` payload into a frame. A `format` ending in `.z`
/// is zlib-compressed (INDI uses zlib's `compress()`); the inner bytes are FITS.
fn decode_blob(b64: &str, format: &str) -> Result<FrameData> {
    // Strip the whitespace INDI servers insert into the base64 stream.
    let cleaned: String = b64.chars().filter(|c| !c.is_ascii_whitespace()).collect();
    let raw = base64::engine::general_purpose::STANDARD
        .decode(cleaned.as_bytes())
        .context("base64 decode")?;

    let fits = if format.trim_end().ends_with(".z") {
        decompress(&raw)?
    } else {
        raw
    };

    fits_bytes_to_frame(&fits)
}

/// zlib first (INDI's convention), gzip as a fallback for lenient servers.
fn decompress(data: &[u8]) -> Result<Vec<u8>> {
    use flate2::read::{GzDecoder, ZlibDecoder};
    let mut out = Vec::new();
    if ZlibDecoder::new(data).read_to_end(&mut out).is_ok() && !out.is_empty() {
        return Ok(out);
    }
    out.clear();
    GzDecoder::new(data)
        .read_to_end(&mut out)
        .context("decompress .z BLOB")?;
    Ok(out)
}

/// Convert in-memory FITS bytes (first 2-D image HDU) into a [`FrameData`],
/// mirroring `fits_source`'s BSCALE/BZERO handling and bit-depth inference.
fn fits_bytes_to_frame(bytes: &[u8]) -> Result<FrameData> {
    use fitskit::{FitsFile, HduData};
    let fits = FitsFile::from_bytes(bytes).context("parse FITS BLOB")?;
    for hdu in fits.iter() {
        let img = match &hdu.data {
            HduData::Image(im) if im.axes.len() >= 2 => im,
            _ => continue,
        };
        let w = img.axes[0] as u32;
        let h = img.axes[1] as u32;
        let bscale = hdu.header.get_float("BSCALE").unwrap_or(1.0);
        let bzero = hdu.header.get_float("BZERO").unwrap_or(0.0);
        let scaled = img.scaled_values(bscale, bzero);
        let n = (w as usize) * (h as usize);
        if scaled.len() < n {
            continue;
        }
        let mono: Vec<f32> = scaled[..n].iter().map(|&v| v as f32).collect();
        let max_val = mono.iter().copied().fold(0.0_f32, f32::max);
        let bit_depth = if max_val <= 255.0 {
            8
        } else if max_val <= 4095.0 {
            12
        } else if max_val <= 16383.0 {
            14
        } else if max_val <= 65535.0 {
            16
        } else {
            32
        };
        return Ok(FrameData::new(mono, w, h, bit_depth));
    }
    Err(anyhow!("no 2-D image HDU in FITS BLOB"))
}

// ── XML helpers ─────────────────────────────────────────────────────────────

/// A child element captured while reading a vector: its tag, attributes, and
/// inner text.
struct Child {
    tag: Vec<u8>,
    attrs: BTreeMap<String, String>,
    text: String,
}

/// Read every child element of the currently-open element up to its matching
/// `</end>`, returning each child's tag/attrs/text. Handles the one level of
/// nesting INDI vectors use (vector → element → text).
fn read_children(
    reader: &mut Reader<BufReader<TcpStream>>,
    buf: &mut Vec<u8>,
    end: &[u8],
) -> Result<Vec<Child>> {
    let mut out = Vec::new();
    let mut cur: Option<Child> = None;
    loop {
        buf.clear();
        match reader.read_event_into(buf)? {
            Event::Start(e) => {
                cur = Some(Child {
                    tag: e.name().as_ref().to_vec(),
                    attrs: attrs_map(&e),
                    text: String::new(),
                });
            }
            Event::Text(t) => {
                if let Some(c) = cur.as_mut() {
                    c.text.push_str(&String::from_utf8_lossy(t.as_ref()));
                }
            }
            Event::Empty(e) => {
                out.push(Child {
                    tag: e.name().as_ref().to_vec(),
                    attrs: attrs_map(&e),
                    text: String::new(),
                });
            }
            Event::End(e) => {
                if e.name().as_ref() == end {
                    break;
                }
                if let Some(c) = cur.take() {
                    out.push(c);
                }
            }
            Event::Eof => return Err(anyhow!("EOF inside <{}>", String::from_utf8_lossy(end))),
            _ => {}
        }
    }
    Ok(out)
}

fn attrs_map(e: &BytesStart) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    for a in e.attributes().flatten() {
        let k = String::from_utf8_lossy(a.key.as_ref()).into_owned();
        if let Ok(v) = a.unescape_value() {
            m.insert(k, v.into_owned());
        }
    }
    m
}

fn attr(e: &BytesStart, key: &[u8]) -> Option<String> {
    e.attributes()
        .flatten()
        .find(|a| a.key.as_ref() == key)
        .and_then(|a| a.unescape_value().ok().map(|c| c.into_owned()))
}

/// Build a numeric control from a `<defNumber>` child.
fn number_control(property: &str, group: &str, c: &Child) -> Option<IndiControl> {
    let element = c.attrs.get("name")?.clone();
    let label = c.attrs.get("label").cloned().unwrap_or_else(|| element.clone());
    let value = c.text.trim().parse::<f64>().unwrap_or(0.0);
    let min = c.attrs.get("min").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let max = c.attrs.get("max").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let step = c.attrs.get("step").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    Some(IndiControl {
        property: property.to_string(),
        element,
        label,
        group: group.to_string(),
        kind: IndiControlKind::Number { value, min, max, step },
        writable: true,
    })
}

/// Insert or replace a control identified by (property, element).
fn upsert(controls: &mut Vec<IndiControl>, ctrl: IndiControl) {
    if let Some(existing) = controls
        .iter_mut()
        .find(|c| c.property == ctrl.property && c.element == ctrl.element)
    {
        *existing = ctrl;
    } else {
        controls.push(ctrl);
    }
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn send(writer: &Arc<Mutex<TcpStream>>, xml: &str) {
    if let Ok(mut w) = writer.lock() {
        let _ = w.write_all(xml.as_bytes());
        let _ = w.flush();
    }
}

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}
