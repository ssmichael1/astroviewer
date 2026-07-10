//! ToupTek (toupcam) camera source: open, configure for astro capture, and
//! stream frames from a background thread.
//!
//! Unlike the SVBony SDK (runtime `control_caps()` discovery) or GigE
//! (GenICam XML), the toupcam SDK's options are a flat numeric space with no
//! runtime metadata — so the UI exposes a curated control set: exposure, gain,
//! speed, cooling, corrections (FFC/DFC/FPNC), filter wheel, focuser, plus the
//! capability-gated [`build_advanced`] table. Everything else stays at SDK
//! defaults. Adding a curated option is one entry in [`build_advanced`]; the
//! capture thread applies any option through the generic [`ToupCmd::SetOption`].
//!
//! All sensors — mono *and* color — stream RAW at native bit depth. The SDK's
//! processed RGB output black-subtracts, white-balances, tone-maps, and
//! quantizes to 8 bits, which destroys the noise floor and photometric
//! linearity; RAW preserves true ADUs (a capped color sensor shows its read
//! noise, as it should). The cost is that color sensors display their Bayer
//! mosaic as a faint checkerboard at 1:1 zoom.

use crossbeam_channel::{bounded, Receiver, Sender};
use std::time::{Duration, Instant};
use toupcam::{sys, AutoExposure, BitDepth, Camera, DeviceInfo, Event, Opt};

/// Commands sent from the UI thread to the camera capture thread.
pub enum ToupCmd {
    /// Exposure time, microseconds.
    SetExposure(u32),
    SetAutoExposure(bool),
    /// Auto-exposure target brightness, `[16, 220]`.
    SetAutoExpoTarget(u16),
    /// Analog gain, percent (100 = 1×).
    SetGain(u16),
    /// Frame-speed level, `0..=max_speed`.
    SetSpeed(u16),
    /// Any `TOUPCAM_OPTION_*` value (TEC, black level, and the advanced table).
    SetOption(Opt, i32),
    Correction(Correction, CorrectionAction),
    /// Move the filter wheel to a 0-based slot (shortest direction).
    SetFilterPosition(u32),
    /// Re-home the filter wheel.
    ResetFilterWheel,
    /// Move the focuser to an absolute step position.
    SetFocuserPosition(i32),
    FocuserHalt,
    Stop,
}

/// On-camera correction pipelines. Support is probed per-camera by reading the
/// correction's enable option.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Correction {
    FlatField,
    DarkField,
    FixedPatternNoise,
}

impl Correction {
    pub fn label(self) -> &'static str {
        match self {
            Correction::FlatField => "Flat Field",
            Correction::DarkField => "Dark Field",
            Correction::FixedPatternNoise => "Fixed Pattern",
        }
    }
    /// What the camera needs to see when capturing this correction.
    pub fn capture_hint(self) -> &'static str {
        match self {
            Correction::FlatField => "Point at an evenly illuminated field, then capture",
            Correction::DarkField => "Cover the sensor (cap on), then capture",
            Correction::FixedPatternNoise => "Cover the sensor (cap on), then capture",
        }
    }
    /// The `TOUPCAM_OPTION_*` that enables/disables this correction.
    fn opt(self) -> Opt {
        match self {
            Correction::FlatField => Opt::new(sys::TOUPCAM_OPTION_FFC),
            Correction::DarkField => Opt::new(sys::TOUPCAM_OPTION_DFC),
            Correction::FixedPatternNoise => Opt::new(sys::TOUPCAM_OPTION_FPNC),
        }
    }
}

pub enum CorrectionAction {
    /// One-shot capture of the correction frame (camera must be streaming).
    Capture,
    Enable(bool),
    Import(String),
    Export(String),
}

/// Periodic readings pushed by the capture thread (~every 2 s). Fields are
/// `None` when the camera lacks the subsystem or the read failed.
#[derive(Clone, Copy, Default)]
pub struct ToupTelemetry {
    pub temperature_c: Option<f32>,
    /// Power consumption, milliwatts.
    pub power_mw: Option<i32>,
    /// Filter-wheel slot, `-1` while the wheel is in motion.
    pub filter_position: Option<i32>,
    pub focuser_position: Option<i32>,
    pub focuser_moving: Option<bool>,
}

/// Which telemetry sources the capture thread should poll, decided at open.
#[derive(Clone, Copy, Default)]
struct TelemetryCfg {
    temp: bool,
    power: bool,
    wheel: bool,
    focuser: bool,
}

/// Enable state of one supported correction.
#[derive(Clone)]
pub struct CorrectionState {
    pub kind: Correction,
    pub enabled: bool,
}

#[derive(Clone)]
pub struct FilterWheelState {
    pub slots: u32,
    /// Current slot, `None` while the wheel is in motion.
    pub position: Option<u32>,
}

#[derive(Clone)]
pub struct FocuserState {
    pub position: i32,
    /// UI-side target position (sent on "Move").
    pub target: i32,
    pub max_step: i32,
    pub moving: bool,
}

/// UI-side mirror of the curated camera controls, populated at open time and
/// edited by the controls panel (each edit also sends a [`ToupCmd`]).
#[derive(Clone)]
pub struct ToupControls {
    pub exposure_us: u32,
    pub exposure_min: u32,
    pub exposure_max: u32,
    pub auto_exposure: bool,
    /// Auto-exposure target brightness, `[16, 220]`.
    pub auto_expo_target: u16,
    pub gain: u16,
    pub gain_min: u16,
    pub gain_max: u16,
    /// Frame-speed level; row hidden when `max_speed == 0`.
    pub speed: u16,
    pub max_speed: u16,
    /// Model has a thermoelectric cooler; gates the Cooling group in the UI.
    pub has_tec: bool,
    pub tec_on: bool,
    pub tec_target_c: f32,
    /// Valid TEC target range, °C.
    pub tec_range: (f32, f32),
    /// Latest sensor temperature reading, if the camera reports one.
    pub temperature_c: Option<f32>,
    /// Latest power draw, milliwatts, if the camera reports it.
    pub power_mw: Option<i32>,
    /// Corrections this camera supports, with their enable state.
    pub corrections: Vec<CorrectionState>,
    pub filter_wheel: Option<FilterWheelState>,
    pub focuser: Option<FocuserState>,
    /// Capability-gated advanced options this model supports, in display order.
    pub advanced: Vec<AdvControl>,
}

/// Widget shape for an advanced option row.
#[derive(Clone, Copy)]
pub enum AdvKind {
    /// On/off (0/1) checkbox.
    Bool,
    /// Integer slider over an inclusive range.
    Int { min: i32, max: i32 },
    /// One-of combo; `(value, label)` pairs.
    Enum(&'static [(i32, &'static str)]),
}

/// An advanced option instantiated for the open camera, holding its current
/// value. Edits send [`ToupCmd::SetOption`] with [`AdvControl::opt`].
#[derive(Clone)]
pub struct AdvControl {
    pub label: &'static str,
    pub opt: Opt,
    pub kind: AdvKind,
    pub value: i32,
}

/// Builds the curated advanced-option list for one camera: each candidate is
/// gated on the model's capability flags, then kept only if its current value
/// can actually be read back. To expose another `TOUPCAM_OPTION_*`, add an
/// entry here — the capture thread and UI need no changes.
fn build_advanced(cam: &Camera, model: &toupcam::ModelInfo, bit_depth: u8) -> Vec<AdvControl> {
    const CG2: &[(i32, &str)] = &[(0, "LCG"), (1, "HCG")];
    const CG3: &[(i32, &str)] = &[(0, "LCG"), (1, "HCG"), (2, "HDR")];
    /// RAW=1 applies the SDK's black balance and flat/dark-field corrections;
    /// RAW=-1 is the untouched sensor readout (full bias pedestal preserved).
    const RAW_MODES: &[(i32, &str)] = &[(1, "Calibrated"), (-1, "Unprocessed")];
    /// Averaging modes (`0x80 | n`) keep the bit depth unchanged, so they are
    /// safe with the viewer's fixed-depth display pipeline.
    const BINNING: &[(i32, &str)] =
        &[(1, "1×1"), (0x82, "2×2"), (0x83, "3×3"), (0x84, "4×4")];

    let mut defs: Vec<(&'static str, Opt, AdvKind)> = Vec::new();

    // Streaming is always RAW (see module docs), so this only toggles the
    // SDK's on-host calibration, not the data layout — safe to switch live.
    defs.push(("Raw Processing", Opt::RAW, AdvKind::Enum(RAW_MODES)));
    if model.has_flag(sys::TOUPCAM_FLAG_CG) || model.has_flag(sys::TOUPCAM_FLAG_CGHDR) {
        let variants = if model.has_flag(sys::TOUPCAM_FLAG_CGHDR) { CG3 } else { CG2 };
        defs.push(("Conversion Gain", Opt::CONVERSION_GAIN, AdvKind::Enum(variants)));
    }
    defs.push(("Binning", Opt::BINNING, AdvKind::Enum(BINNING)));
    if model.has_flag(sys::TOUPCAM_FLAG_LOW_NOISE) {
        defs.push(("Low Noise Mode", Opt::LOW_NOISE, AdvKind::Bool));
    }
    if model.has_flag(sys::TOUPCAM_FLAG_HIGH_FULLWELL) {
        defs.push(("High Full-Well", Opt::HIGH_FULLWELL, AdvKind::Bool));
    }
    if model.has_flag(sys::TOUPCAM_FLAG_BLACKLEVEL) {
        // SDK range: 0..=31 at 8-bit, scaling with bit depth (31 << (depth-8)).
        let max = 31i32 << bit_depth.saturating_sub(8);
        defs.push((
            "Black Level",
            Opt::new(sys::TOUPCAM_OPTION_BLACKLEVEL),
            AdvKind::Int { min: 0, max },
        ));
    }
    if model.has_flag(sys::TOUPCAM_FLAG_FAN) && model.max_fan_speed > 0 {
        defs.push((
            "Fan Speed",
            Opt::FAN,
            AdvKind::Int { min: 0, max: model.max_fan_speed as i32 },
        ));
    }
    // 0 = unlimited; the SDK caps the limit value at 63 fps.
    defs.push(("Frame Rate Limit", Opt::FRAMERATE, AdvKind::Int { min: 0, max: 63 }));

    defs.into_iter()
        .filter_map(|(label, opt, kind)| {
            cam.get_option(opt)
                .ok()
                .map(|value| AdvControl { label, opt, kind, value })
        })
        .collect()
}

/// Probes which corrections this camera supports by reading their enable
/// options.
fn probe_corrections(cam: &Camera) -> Vec<CorrectionState> {
    [
        Correction::FlatField,
        Correction::DarkField,
        Correction::FixedPatternNoise,
    ]
    .into_iter()
    .filter_map(|kind| {
        cam.get_option(kind.opt()).ok().map(|v| CorrectionState {
            kind,
            enabled: v > 0,
        })
    })
    .collect()
}

pub struct ToupHandle {
    #[allow(dead_code)]
    pub info: DeviceInfo,
    pub cmd_tx: Sender<ToupCmd>,
    /// Periodic telemetry snapshots pushed by the capture thread.
    pub telemetry_rx: Receiver<ToupTelemetry>,
    pub join_handle: Option<std::thread::JoinHandle<()>>,
}

/// Enumerate connected ToupTek cameras (USB and, once per process, GigE).
/// Returns an empty vec if none found.
pub fn enumerate() -> Vec<DeviceInfo> {
    // ToupTek GigE models only appear in enumeration after GigE support is
    // initialised; do it once, lazily. Harmless when no GigE camera exists.
    static GIGE: std::sync::Once = std::sync::Once::new();
    GIGE.call_once(|| {
        let _ = toupcam::gige_enable::<fn()>(None);
    });
    toupcam::enumerate()
}

/// Open a camera, configure it for astro capture (RAW + native bit depth),
/// start a capture thread, and return a handle plus the initial control values.
pub fn start_camera(
    info: &DeviceInfo,
    frame_tx: Sender<super::FrameData>,
    log_tx: Sender<super::LogEntry>,
) -> anyhow::Result<(ToupHandle, ToupControls)> {
    let cam = Camera::open(&info.id)
        .map_err(|e| anyhow::anyhow!("{}: {}", info.display_name, e))?;

    if info.model.is_usb3_over_usb2() {
        let _ = log_tx.try_send(super::LogEntry::warn(format!(
            "{}: USB3 camera on a USB2 link — bandwidth limited",
            info.display_name
        )));
    }

    let mono = cam.is_mono().unwrap_or(info.model.is_mono());
    let max_depth = cam.max_bit_depth().unwrap_or(8).min(16) as u8;

    // Pixel pipeline, set before streaming starts: RAW at native bit depth
    // for every sensor (see module docs for why RAW, even on color models).
    cam.put_option(Opt::RAW, 1)
        .map_err(|e| anyhow::anyhow!("set RAW mode: {}", e))?;
    let (pull_bits, bit_depth) = if max_depth > 8 {
        cam.put_option(Opt::BITDEPTH, 1)
            .map_err(|e| anyhow::anyhow!("set 16-bit mode: {}", e))?;
        (BitDepth::Bpp16, max_depth)
    } else {
        (BitDepth::Bpp8, 8)
    };
    let _ = log_tx.try_send(super::LogEntry::info(format!(
        "{}: RAW {} capture, {}-bit{}",
        info.display_name,
        if mono { "mono" } else { "color (Bayer)" },
        bit_depth,
        if mono { "" } else { " — mosaic visible at 1:1 zoom" },
    )));

    // Deterministic manual exposure to start; the Auto checkbox re-enables it.
    let _ = cam.set_auto_exposure(AutoExposure::Off);

    let (exp_min, exp_max, exp_def) = cam.exposure_range().unwrap_or((1, 10_000_000, 100_000));
    let exposure_us = cam.exposure().unwrap_or(exp_def).clamp(exp_min, exp_max);
    let auto_expo_target = cam.auto_expo_target().unwrap_or(120);
    let (gain_min, gain_max, gain_def) = cam.gain_range().unwrap_or((100, 100, 100));
    let gain = cam.gain().unwrap_or(gain_def).clamp(gain_min, gain_max);

    let max_speed = info.model.max_speed as u16;
    let speed = cam.speed().unwrap_or(0).min(max_speed);

    let has_tec = info.model.has_tec();
    let tec_on = has_tec && cam.get_option(Opt::TEC).map(|v| v != 0).unwrap_or(false);
    let tec_range = cam.tec_target_range().unwrap_or((-50.0, 30.0));
    let tec_target_c = cam
        .get_option(Opt::TEC_TARGET)
        .map(|v| v as f32 / 10.0)
        .unwrap_or(0.0)
        .clamp(tec_range.0, tec_range.1);
    let temperature_c = cam.temperature().ok();
    let power_mw = cam.power_consumption().ok();

    let corrections = probe_corrections(&cam);

    let filter_wheel = if info.model.has_flag(sys::TOUPCAM_FLAG_FILTERWHEEL) {
        let slots = cam.filter_wheel_slots().unwrap_or(0).max(0) as u32;
        (slots > 0).then(|| FilterWheelState {
            slots,
            position: cam.filter_position().ok().flatten(),
        })
    } else {
        None
    };

    let focuser = if info.model.has_flag(sys::TOUPCAM_FLAG_AUTOFOCUSER) {
        cam.focuser_position().ok().map(|position| FocuserState {
            position,
            target: position,
            max_step: cam.aaf(sys::TOUPCAM_AAF_GETMAXSTEP, 0).unwrap_or(100_000).max(1),
            moving: false,
        })
    } else {
        None
    };

    let advanced = build_advanced(&cam, &info.model, bit_depth);

    let telemetry_cfg = TelemetryCfg {
        temp: info.model.can_get_temperature() || temperature_c.is_some(),
        power: power_mw.is_some(),
        wheel: filter_wheel.is_some(),
        focuser: focuser.is_some(),
    };

    let controls = ToupControls {
        exposure_us,
        exposure_min: exp_min,
        exposure_max: exp_max,
        auto_exposure: false,
        auto_expo_target,
        gain,
        gain_min,
        gain_max,
        speed,
        max_speed,
        has_tec,
        tec_on,
        tec_target_c,
        tec_range,
        temperature_c,
        power_mw,
        corrections,
        filter_wheel,
        focuser,
        advanced,
    };

    cam.start_pull_mode()
        .map_err(|e| anyhow::anyhow!("start streaming: {}", e))?;

    let (cmd_tx, cmd_rx) = bounded::<ToupCmd>(16);
    let (telemetry_tx, telemetry_rx) = bounded::<ToupTelemetry>(4);

    let ctx = CaptureCtx {
        cam,
        cam_name: info.display_name.clone(),
        frame_tx,
        cmd_rx,
        log_tx,
        telemetry_tx,
        pull_bits,
        bit_depth,
        telemetry_cfg,
    };
    let join_handle = std::thread::spawn(move || capture_loop(ctx));

    Ok((
        ToupHandle {
            info: info.clone(),
            cmd_tx,
            telemetry_rx,
            join_handle: Some(join_handle),
        },
        controls,
    ))
}

struct CaptureCtx {
    cam: Camera,
    cam_name: String,
    frame_tx: Sender<super::FrameData>,
    cmd_rx: Receiver<ToupCmd>,
    log_tx: Sender<super::LogEntry>,
    telemetry_tx: Sender<ToupTelemetry>,
    pull_bits: BitDepth,
    bit_depth: u8,
    telemetry_cfg: TelemetryCfg,
}

fn apply_cmd(cam: &Camera, cmd: ToupCmd) -> toupcam::Result<()> {
    match cmd {
        ToupCmd::Stop => unreachable!("handled by caller"),
        ToupCmd::SetExposure(us) => cam.set_exposure(us),
        ToupCmd::SetAutoExposure(on) => cam.set_auto_exposure(if on {
            AutoExposure::Continuous
        } else {
            AutoExposure::Off
        }),
        ToupCmd::SetAutoExpoTarget(t) => cam.set_auto_expo_target(t),
        ToupCmd::SetGain(g) => cam.set_gain(g),
        ToupCmd::SetSpeed(s) => cam.set_speed(s),
        ToupCmd::SetOption(opt, v) => cam.put_option(opt, v),
        ToupCmd::Correction(kind, action) => match action {
            CorrectionAction::Capture => match kind {
                Correction::FlatField => cam.ffc_once(),
                Correction::DarkField => cam.dfc_once(),
                Correction::FixedPatternNoise => cam.fpnc_once(),
            },
            CorrectionAction::Enable(on) => cam.put_option(kind.opt(), on as i32),
            CorrectionAction::Import(path) => match kind {
                Correction::FlatField => cam.ffc_import(&path),
                Correction::DarkField => cam.dfc_import(&path),
                Correction::FixedPatternNoise => cam.fpnc_import(&path),
            },
            CorrectionAction::Export(path) => match kind {
                Correction::FlatField => cam.ffc_export(&path),
                Correction::DarkField => cam.dfc_export(&path),
                Correction::FixedPatternNoise => cam.fpnc_export(&path),
            },
        },
        ToupCmd::SetFilterPosition(p) => cam.set_filter_position(p, true),
        ToupCmd::ResetFilterWheel => cam.reset_filter_wheel(),
        ToupCmd::SetFocuserPosition(p) => cam.set_focuser_position(p),
        ToupCmd::FocuserHalt => cam.focuser_halt(),
    }
}

fn poll_telemetry(cam: &Camera, cfg: TelemetryCfg) -> ToupTelemetry {
    ToupTelemetry {
        temperature_c: cfg.temp.then(|| cam.temperature().ok()).flatten(),
        power_mw: cfg.power.then(|| cam.power_consumption().ok()).flatten(),
        filter_position: cfg
            .wheel
            .then(|| {
                cam.filter_position()
                    .ok()
                    .map(|p| p.map_or(-1, |v| v as i32))
            })
            .flatten(),
        focuser_position: cfg.focuser.then(|| cam.focuser_position().ok()).flatten(),
        focuser_moving: cfg.focuser.then(|| cam.focuser_is_moving().ok()).flatten(),
    }
}

fn capture_loop(ctx: CaptureCtx) {
    let CaptureCtx {
        cam,
        cam_name,
        frame_tx,
        cmd_rx,
        log_tx,
        telemetry_tx,
        pull_bits,
        bit_depth,
        telemetry_cfg,
    } = ctx;

    let native_max: u16 = if bit_depth >= 16 { u16::MAX } else { (1u16 << bit_depth) - 1 };
    // Whether 16-bit samples arrive left-justified (e.g. 12-bit data in the
    // top bits). The SDK doesn't document this and it varies by model, so
    // detect it: any sample above the native range proves left-justification,
    // after which every frame is shifted down to native ADU values.
    let mut shift_bits: u8 = 0;
    let mut last_telemetry = Instant::now();

    loop {
        // Apply pending UI commands.
        while let Ok(cmd) = cmd_rx.try_recv() {
            if matches!(cmd, ToupCmd::Stop) {
                let _ = cam.stop();
                return;
            }
            if let Err(e) = apply_cmd(&cam, cmd) {
                let _ = log_tx.try_send(super::LogEntry::error(format!(
                    "{}: set control failed: {}",
                    cam_name, e
                )));
            }
        }

        if last_telemetry.elapsed() > Duration::from_secs(2) {
            last_telemetry = Instant::now();
            let _ = telemetry_tx.try_send(poll_telemetry(&cam, telemetry_cfg));
        }

        // Events arrive on a channel fed by the SDK's internal thread. The
        // timeout keeps the command/telemetry polling above responsive during
        // long exposures.
        match cam.events().recv_timeout(Duration::from_millis(200)) {
            Ok(Event::Image) => {
                let frame = match cam.pull_image(pull_bits) {
                    Ok(f) => f,
                    Err(e) => {
                        let _ = log_tx.try_send(super::LogEntry::error(format!(
                            "{}: pull image failed: {}",
                            cam_name, e
                        )));
                        continue;
                    }
                };
                let Some(mut img) = frame.to_image() else { continue };
                if pull_bits == BitDepth::Bpp16 && bit_depth < 16 {
                    if shift_bits == 0 && image_exceeds(&img, native_max) {
                        shift_bits = 16 - bit_depth;
                        let _ = log_tx.try_send(super::LogEntry::info(format!(
                            "{}: 16-bit samples are left-justified, shifting right by {}",
                            cam_name, shift_bits
                        )));
                    }
                    if shift_bits > 0 {
                        img = shift_image_right(img, shift_bits);
                    }
                }
                let frame_data = super::process_image(img, bit_depth);
                if frame_tx.try_send(frame_data).is_err() && frame_tx.is_empty() {
                    // Receiver dropped — the UI is gone.
                    let _ = cam.stop();
                    return;
                }
            }
            Ok(Event::Disconnected) => {
                let _ = log_tx.try_send(super::LogEntry::error(format!(
                    "{}: camera disconnected",
                    cam_name
                )));
                return;
            }
            Ok(Event::Error) => {
                let _ = log_tx.try_send(super::LogEntry::error(format!(
                    "{}: camera reported an error, stopping capture",
                    cam_name
                )));
                let _ = cam.stop();
                return;
            }
            Ok(_) => {} // exposure/white-balance/still notifications — not used
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => return,
        }
    }
}

/// `true` if any 16-bit sample exceeds the sensor's native range — proof the
/// SDK delivers left-justified data.
fn image_exceeds(img: &image::DynamicImage, native_max: u16) -> bool {
    match img {
        image::DynamicImage::ImageLuma16(buf) => buf.as_raw().iter().any(|&v| v > native_max),
        _ => false,
    }
}

/// Right-shift all pixel values in a 16-bit image to recover native ADU values
/// when the SDK left-justifies data (e.g., 12-bit data in the upper bits).
fn shift_image_right(img: image::DynamicImage, shift: u8) -> image::DynamicImage {
    use image::DynamicImage;
    match img {
        DynamicImage::ImageLuma16(buf) => {
            let shifted: Vec<u16> = buf.as_raw().iter().map(|&v| v >> shift).collect();
            let out = image::ImageBuffer::from_raw(buf.width(), buf.height(), shifted).unwrap();
            DynamicImage::ImageLuma16(out)
        }
        other => other,
    }
}
