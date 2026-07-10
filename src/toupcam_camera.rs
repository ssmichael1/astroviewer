//! ToupTek (toupcam) camera source: open, configure for astro capture, and
//! stream frames from a background thread.
//!
//! Unlike the SVBony SDK (runtime `control_caps()` discovery) or GigE
//! (GenICam XML), the toupcam SDK's options are a flat numeric space with no
//! runtime metadata — so the UI exposes a small curated control set (exposure,
//! gain, cooling) and leaves everything else at SDK defaults. The generic
//! [`ToupCmd::SetOption`] command means adding another curated option later
//! only touches the UI table, not this module.

use crossbeam_channel::{bounded, Receiver, Sender};
use std::time::{Duration, Instant};
use toupcam::{AutoExposure, BitDepth, Camera, DeviceInfo, Event, Opt};

/// Commands sent from the UI thread to the camera capture thread.
pub enum ToupCmd {
    /// Exposure time, microseconds.
    SetExposure(u32),
    SetAutoExposure(bool),
    /// Analog gain, percent (100 = 1×).
    SetGain(u16),
    /// Any `TOUPCAM_OPTION_*` value (TEC, TEC target, and future curated options).
    SetOption(Opt, i32),
    Stop,
}

/// UI-side mirror of the curated camera controls, populated at open time and
/// edited by the controls panel (each edit also sends a [`ToupCmd`]).
#[derive(Clone)]
pub struct ToupControls {
    pub exposure_us: u32,
    pub exposure_min: u32,
    pub exposure_max: u32,
    pub auto_exposure: bool,
    pub gain: u16,
    pub gain_min: u16,
    pub gain_max: u16,
    /// Model has a thermoelectric cooler; gates the Cooling group in the UI.
    pub has_tec: bool,
    pub tec_on: bool,
    pub tec_target_c: f32,
    /// Latest sensor temperature reading, if the camera reports one.
    pub temperature_c: Option<f32>,
}

pub struct ToupHandle {
    #[allow(dead_code)]
    pub info: DeviceInfo,
    pub cmd_tx: Sender<ToupCmd>,
    /// Periodic sensor-temperature readings pushed by the capture thread.
    pub temp_rx: Receiver<f32>,
    pub join_handle: Option<std::thread::JoinHandle<()>>,
}

/// Enumerate connected ToupTek cameras. Returns an empty vec if none found.
pub fn enumerate() -> Vec<DeviceInfo> {
    toupcam::enumerate()
}

/// Open a camera, configure it for astro capture (RAW + native bit depth on
/// mono sensors), start a capture thread, and return a handle plus the initial
/// control values.
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

    // Pixel pipeline, set before streaming starts.
    // Mono sensors: RAW mode + native bit depth — unprocessed ADU values with
    // no tone mapping (the SDK applies a logarithmic curve by default, which is
    // wrong under our own scaling/gamma). Color sensors: keep the SDK's RGB24
    // debayer and convert to luminance on receive (RAW would show the Bayer
    // mosaic as a checkerboard).
    let (pull_bits, bit_depth) = if mono {
        cam.put_option(Opt::RAW, 1)
            .map_err(|e| anyhow::anyhow!("set RAW mode: {}", e))?;
        if max_depth > 8 {
            cam.put_option(Opt::BITDEPTH, 1)
                .map_err(|e| anyhow::anyhow!("set 16-bit mode: {}", e))?;
            (BitDepth::Bpp16, max_depth)
        } else {
            (BitDepth::Bpp8, 8)
        }
    } else {
        (BitDepth::Bpp24, 8)
    };
    let _ = log_tx.try_send(super::LogEntry::info(format!(
        "{}: {} capture, {}-bit",
        info.display_name,
        if mono { "RAW mono" } else { "RGB24 color" },
        bit_depth,
    )));

    // Deterministic manual exposure to start; the Auto checkbox re-enables it.
    let _ = cam.set_auto_exposure(AutoExposure::Off);

    let (exp_min, exp_max, exp_def) = cam.exposure_range().unwrap_or((1, 10_000_000, 100_000));
    let exposure_us = cam.exposure().unwrap_or(exp_def).clamp(exp_min, exp_max);
    let (gain_min, gain_max, gain_def) = cam.gain_range().unwrap_or((100, 100, 100));
    let gain = cam.gain().unwrap_or(gain_def).clamp(gain_min, gain_max);

    let has_tec = info.model.has_tec();
    let tec_on = has_tec && cam.get_option(Opt::TEC).map(|v| v != 0).unwrap_or(false);
    let tec_target_c = cam
        .get_option(Opt::TEC_TARGET)
        .map(|v| v as f32 / 10.0)
        .unwrap_or(0.0);
    let temperature_c = cam.temperature().ok();

    let controls = ToupControls {
        exposure_us,
        exposure_min: exp_min,
        exposure_max: exp_max,
        auto_exposure: false,
        gain,
        gain_min,
        gain_max,
        has_tec,
        tec_on,
        tec_target_c,
        temperature_c,
    };

    cam.start_pull_mode()
        .map_err(|e| anyhow::anyhow!("start streaming: {}", e))?;

    let (cmd_tx, cmd_rx) = bounded::<ToupCmd>(16);
    let (temp_tx, temp_rx) = bounded::<f32>(4);

    let cam_name = info.display_name.clone();
    let can_read_temp = info.model.can_get_temperature() || temperature_c.is_some();
    let join_handle = std::thread::spawn(move || {
        capture_loop(
            cam, &cam_name, frame_tx, cmd_rx, log_tx, temp_tx, pull_bits, bit_depth, can_read_temp,
        );
    });

    Ok((
        ToupHandle {
            info: info.clone(),
            cmd_tx,
            temp_rx,
            join_handle: Some(join_handle),
        },
        controls,
    ))
}

#[allow(clippy::too_many_arguments)]
fn capture_loop(
    cam: Camera,
    cam_name: &str,
    frame_tx: Sender<super::FrameData>,
    cmd_rx: Receiver<ToupCmd>,
    log_tx: Sender<super::LogEntry>,
    temp_tx: Sender<f32>,
    pull_bits: BitDepth,
    bit_depth: u8,
    can_read_temp: bool,
) {
    let native_max: u16 = if bit_depth >= 16 { u16::MAX } else { (1u16 << bit_depth) - 1 };
    // Whether 16-bit samples arrive left-justified (e.g. 12-bit data in the
    // top bits). The SDK doesn't document this and it varies by model, so
    // detect it: any sample above the native range proves left-justification,
    // after which every frame is shifted down to native ADU values.
    let mut shift_bits: u8 = 0;
    let mut last_temp_poll = Instant::now();

    loop {
        // Apply pending UI commands.
        while let Ok(cmd) = cmd_rx.try_recv() {
            let result = match cmd {
                ToupCmd::Stop => {
                    let _ = cam.stop();
                    return;
                }
                ToupCmd::SetExposure(us) => cam.set_exposure(us),
                ToupCmd::SetAutoExposure(on) => cam.set_auto_exposure(if on {
                    AutoExposure::Continuous
                } else {
                    AutoExposure::Off
                }),
                ToupCmd::SetGain(g) => cam.set_gain(g),
                ToupCmd::SetOption(opt, v) => cam.put_option(opt, v),
            };
            if let Err(e) = result {
                let _ = log_tx.try_send(super::LogEntry::error(format!(
                    "{}: set control failed: {}",
                    cam_name, e
                )));
            }
        }

        if can_read_temp && last_temp_poll.elapsed() > Duration::from_secs(2) {
            last_temp_poll = Instant::now();
            if let Ok(t) = cam.temperature() {
                let _ = temp_tx.try_send(t);
            }
        }

        // Events arrive on a channel fed by the SDK's internal thread. The
        // timeout keeps the command/temperature polling above responsive
        // during long exposures.
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
