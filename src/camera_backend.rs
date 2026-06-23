//! A backend-neutral camera abstraction.
//!
//! svbony (USB SDK), GigE Vision / GenICam, and INDI describe their controls in
//! very different vocabularies. This trait unifies the three things the app
//! actually needs from a live camera — its adjustable controls, applying a
//! change, and stopping — so the UI renders one generic controls panel and a
//! new backend is a drop-in. Frames are delivered out of band on the shared
//! `FrameData` channel each backend is given when it starts.

/// One adjustable (or read-only) control, in a vocabulary common to all
/// backends. `id` is the backend-native key used to address it in [`ControlCmd`].
#[derive(Clone, Debug)]
pub struct ControlDesc {
    pub id: String,
    pub label: String,
    pub group: String,
    pub unit: String,
    pub kind: ControlKind,
    pub writable: bool,
    /// Changing this control pauses/restarts the stream (GenICam ROI/format).
    pub needs_restart: bool,
    /// `Some(_)` when the control has a companion auto mode (svbony); the bool
    /// is the current auto state. INDI/GenICam expose auto as its own control.
    pub auto: Option<bool>,
}

#[derive(Clone, Debug)]
pub enum ControlKind {
    Int { value: i64, min: i64, max: i64, step: i64 },
    Float { value: f64, min: f64, max: f64 },
    Bool(bool),
    /// `value` indexes `options`.
    Enum { value: usize, options: Vec<String> },
    /// A button/trigger; carries no value.
    Command,
    /// Telemetry text (read-only).
    ReadOnly(String),
}

/// A request to change a control, addressed by [`ControlDesc::id`].
#[derive(Clone, Debug)]
pub enum ControlCmd {
    SetInt(String, i64),
    SetFloat(String, f64),
    SetBool(String, bool),
    SetEnum(String, String),
    SetAuto(String, bool),
    Execute(String),
}

/// A live camera backend. Construction (opening/connecting + spawning the
/// capture thread) is backend-specific; the app drives it through this trait.
pub trait CameraBackend: Send {
    /// Current control snapshot for the Controls panel.
    fn controls(&self) -> Vec<ControlDesc>;
    /// Drain any control snapshot the capture thread pushed (dynamic
    /// writability / live telemetry). Returns true if anything changed.
    fn poll_controls(&mut self) -> bool;
    /// Apply a control change (forwarded to the capture thread).
    fn set_control(&mut self, cmd: ControlCmd);
    /// Best estimate of the current exposure time in microseconds, for FITS
    /// recording metadata. `None` if the backend can't report it.
    fn exposure_us(&self) -> Option<f64> {
        None
    }
    /// Stop acquisition and join the capture thread.
    fn stop(self: Box<Self>);
}

// ── GigE Vision / GenICam ───────────────────────────────────────────────────

#[cfg(feature = "gev")]
impl CameraBackend for crate::gev_camera::GevHandle {
    fn controls(&self) -> Vec<ControlDesc> {
        use crate::gev_camera::GevControlKind as K;
        self.controls
            .iter()
            .map(|c| {
                let kind = match &c.kind {
                    K::Integer => ControlKind::Int { value: c.value, min: c.min, max: c.max, step: 0 },
                    K::Float => ControlKind::Float { value: c.fvalue, min: c.fmin, max: c.fmax },
                    K::Boolean => ControlKind::Bool(c.value != 0),
                    K::Enumeration(opts) => ControlKind::Enum {
                        value: c.value.max(0) as usize,
                        options: opts.clone(),
                    },
                    K::Command => ControlKind::Command,
                    K::ReadOnly => ControlKind::ReadOnly(format!("{:.3}", c.fvalue)),
                    K::IpV4 => ControlKind::ReadOnly(fmt_ipv4(c.value, c.ip_swapped)),
                    K::MacAddr => ControlKind::ReadOnly(fmt_mac(c.value, c.ip_swapped)),
                };
                ControlDesc {
                    id: c.name.clone(),
                    label: c.display.clone(),
                    group: c.category.clone(),
                    unit: c.unit.clone(),
                    kind,
                    writable: c.writable,
                    needs_restart: c.needs_restart,
                    auto: None,
                }
            })
            .collect()
    }

    fn poll_controls(&mut self) -> bool {
        let mut changed = false;
        while let Ok(snap) = self.controls_rx.try_recv() {
            self.controls = snap;
            changed = true;
        }
        changed
    }

    fn set_control(&mut self, cmd: ControlCmd) {
        use crate::gev_camera::GevCmd;
        let g = match cmd {
            ControlCmd::SetInt(id, v) => GevCmd::SetInt(id, v),
            ControlCmd::SetFloat(id, v) => GevCmd::SetFloat(id, v),
            ControlCmd::SetBool(id, on) => GevCmd::SetBool(id, on),
            ControlCmd::SetEnum(id, v) => GevCmd::SetEnum(id, v),
            ControlCmd::Execute(id) => GevCmd::Execute(id),
            ControlCmd::SetAuto(_, _) => return, // GenICam auto is its own control
        };
        let _ = self.cmd_tx.send(g);
    }

    fn exposure_us(&self) -> Option<f64> {
        // GenICam ExposureTime is microseconds.
        self.controls.iter().find(|c| c.name == "ExposureTime").map(|c| c.fvalue)
    }

    fn stop(mut self: Box<Self>) {
        crate::gev_camera::GevHandle::stop(&mut self);
    }
}

#[cfg(feature = "gev")]
fn fmt_ipv4(value: i64, swapped: bool) -> String {
    let v = value as u32;
    let b = v.to_be_bytes();
    let o = if swapped { [b[3], b[2], b[1], b[0]] } else { b };
    format!("{}.{}.{}.{}", o[0], o[1], o[2], o[3])
}

#[cfg(feature = "gev")]
fn fmt_mac(value: i64, swapped: bool) -> String {
    let v = (value as u64) & 0x0000_FFFF_FFFF_FFFF;
    let b = v.to_be_bytes(); // 8 bytes; low 6 are the MAC
    let mac = if swapped {
        [b[7], b[6], b[5], b[4], b[3], b[2]]
    } else {
        [b[2], b[3], b[4], b[5], b[6], b[7]]
    };
    mac.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(":")
}

// ── INDI ────────────────────────────────────────────────────────────────────

#[cfg(feature = "indi")]
impl CameraBackend for crate::indi_camera::IndiHandle {
    fn controls(&self) -> Vec<ControlDesc> {
        use crate::indi_camera::IndiControlKind as K;
        self.controls
            .iter()
            .map(|c| {
                let kind = match &c.kind {
                    K::Number { value, min, max, .. } => ControlKind::Float {
                        value: *value,
                        min: *min,
                        max: *max,
                    },
                    K::Switch { on } => ControlKind::Bool(*on),
                };
                ControlDesc {
                    id: format!("{}/{}", c.property, c.element),
                    label: c.label.clone(),
                    group: c.group.clone(),
                    unit: String::new(),
                    kind,
                    writable: c.writable,
                    needs_restart: false,
                    auto: None,
                }
            })
            .collect()
    }

    fn poll_controls(&mut self) -> bool {
        let mut changed = false;
        while let Ok(snap) = self.controls_rx.try_recv() {
            self.controls = snap;
            changed = true;
        }
        changed
    }

    fn set_control(&mut self, cmd: ControlCmd) {
        use crate::indi_camera::IndiCmd;
        let split = |id: &str| -> Option<(String, String)> {
            id.split_once('/').map(|(p, e)| (p.to_string(), e.to_string()))
        };
        let native = match cmd {
            ControlCmd::SetFloat(id, v) => split(&id).map(|(p, e)| IndiCmd::SetNumber(p, e, v)),
            ControlCmd::SetInt(id, v) => split(&id).map(|(p, e)| IndiCmd::SetNumber(p, e, v as f64)),
            ControlCmd::SetBool(id, on) => split(&id).map(|(p, e)| IndiCmd::SetSwitch(p, e, on)),
            _ => None,
        };
        if let Some(c) = native {
            crate::indi_camera::IndiHandle::set_control(self, c);
        }
    }

    fn exposure_us(&self) -> Option<f64> {
        // INDI CCD_EXPOSURE_VALUE is in seconds.
        self.controls.iter().find_map(|c| {
            if c.property == "CCD_EXPOSURE" {
                if let crate::indi_camera::IndiControlKind::Number { value, .. } = c.kind {
                    return Some(value * 1_000_000.0);
                }
            }
            None
        })
    }

    fn stop(mut self: Box<Self>) {
        crate::indi_camera::IndiHandle::stop(&mut self);
    }
}

// ── svbony (USB SDK) ────────────────────────────────────────────────────────
//
// svbony needs a wrapper: unlike gev/indi its handle has no live control
// snapshot, so we mirror (value, auto) here and translate the generic command
// vocabulary into the SDK's combined `SetControl(type, value, auto)`.

#[cfg(feature = "svbony")]
pub struct SvbonyBackend {
    handle: crate::camera::CameraHandle,
    /// (control_type, value, is_auto), parallel to `handle.controls`.
    values: Vec<(svbony::ControlType, i64, bool)>,
}

#[cfg(feature = "svbony")]
impl SvbonyBackend {
    pub fn new(handle: crate::camera::CameraHandle) -> Self {
        let values = handle
            .controls
            .iter()
            .zip(handle.initial_values.iter())
            .map(|(c, iv)| (c.control_type, iv.0, iv.1))
            .collect();
        Self { handle, values }
    }

    fn index_of(&self, id: &str) -> Option<usize> {
        self.handle.controls.iter().position(|c| c.name == id)
    }
}

#[cfg(feature = "svbony")]
impl CameraBackend for SvbonyBackend {
    fn controls(&self) -> Vec<ControlDesc> {
        self.handle
            .controls
            .iter()
            .zip(self.values.iter())
            .map(|(caps, v)| {
                let is_bool = caps.max_value - caps.min_value <= 1;
                let kind = if is_bool {
                    ControlKind::Bool(v.1 != 0)
                } else {
                    ControlKind::Int {
                        value: v.1,
                        min: caps.min_value,
                        max: caps.max_value,
                        step: 1,
                    }
                };
                ControlDesc {
                    id: caps.name.clone(),
                    label: caps.name.clone(),
                    group: "Camera".into(),
                    unit: String::new(),
                    kind,
                    writable: caps.is_writable,
                    needs_restart: false,
                    auto: if caps.is_auto_supported { Some(v.2) } else { None },
                }
            })
            .collect()
    }

    fn poll_controls(&mut self) -> bool {
        false // svbony pushes no live control updates
    }

    fn set_control(&mut self, cmd: ControlCmd) {
        let (id, new_val, new_auto) = match cmd {
            ControlCmd::SetInt(id, v) => (id, Some(v), None),
            ControlCmd::SetBool(id, on) => (id, Some(on as i64), None),
            ControlCmd::SetAuto(id, a) => (id, None, Some(a)),
            _ => return,
        };
        if let Some(i) = self.index_of(&id) {
            if let Some(v) = new_val {
                self.values[i].1 = v;
            }
            if let Some(a) = new_auto {
                self.values[i].2 = a;
            }
            let (ty, val, auto) = self.values[i];
            let _ = self.handle.cmd_tx.send(crate::camera::CameraCmd::SetControl(ty, val, auto));
        }
    }

    fn exposure_us(&self) -> Option<f64> {
        // svbony Exposure control is in microseconds.
        self.values
            .iter()
            .find(|(t, _, _)| *t == svbony::ControlType::Exposure)
            .map(|(_, v, _)| *v as f64)
    }

    fn stop(mut self: Box<Self>) {
        let _ = self.handle.cmd_tx.send(crate::camera::CameraCmd::Stop);
        if let Some(jh) = self.handle.join_handle.take() {
            let _ = jh.join();
        }
    }
}
