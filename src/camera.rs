use crossbeam_channel::{bounded, Receiver, Sender};
use svbony::{Camera, CameraInfo, ControlCaps, ControlType, ImageType, RoiFormat};

/// Commands sent from the UI thread to the camera capture thread.
pub enum CameraCmd {
    SetControl(ControlType, i64, bool),
    Stop,
}

/// Information about a discovered camera, plus its available controls.
pub struct CameraHandle {
    pub info: CameraInfo,
    pub property: svbony::CameraProperty,
    pub controls: Vec<ControlCaps>,
    pub cmd_tx: Sender<CameraCmd>,
}

/// Enumerate connected SVBony cameras. Returns an empty vec if none found.
pub fn enumerate() -> Vec<CameraInfo> {
    svbony::connected_cameras().unwrap_or_default()
}

/// Open a camera, configure for full-sensor mono capture, start a capture thread,
/// and return a handle with control metadata and command channel.
pub fn start_camera(
    info: &CameraInfo,
    frame_tx: Sender<super::FrameData>,
    log_tx: Sender<super::LogEntry>,
) -> anyhow::Result<CameraHandle> {
    let cam = Camera::open(info.camera_id)?;

    let prop = cam.property()?;
    let bit_depth = prop.max_bit_depth as u8;

    // Query all controls
    let num_controls = cam.num_controls()?;
    let mut controls = Vec::with_capacity(num_controls);
    for i in 0..num_controls {
        if let Ok(caps) = cam.control_caps(i) {
            controls.push(caps);
        }
    }

    // Full-sensor ROI
    cam.set_roi(&RoiFormat {
        start_x: 0,
        start_y: 0,
        width: prop.max_width as i32,
        height: prop.max_height as i32,
        bin: 1,
    })?;

    // Mono output format — try to match the sensor's native bit depth.
    // Fall back to Y16 if the preferred format isn't supported.
    let preferred = match bit_depth {
        0..=8 => ImageType::Y8,
        9..=10 => ImageType::Y10,
        11..=12 => ImageType::Y12,
        13..=14 => ImageType::Y14,
        _ => ImageType::Y16,
    };
    let img_type = if prop.supported_formats.contains(&preferred) {
        preferred
    } else {
        // Fallback to Y16; we'll right-shift in the capture loop
        ImageType::Y16
    };
    let needs_shift = img_type == ImageType::Y16 && bit_depth < 16;
    if img_type == preferred {
        let _ = log_tx.try_send(super::LogEntry::info(
            format!("Using native format {:?} for {}-bit sensor", img_type, bit_depth),
        ));
    } else {
        let _ = log_tx.try_send(super::LogEntry::info(
            format!("Format {:?} not supported, using {:?} with >>{}  shift", preferred, img_type, 16 - bit_depth),
        ));
    }
    cam.set_output_image_type(img_type)?;

    cam.start_capture()?;

    let (cmd_tx, cmd_rx) = bounded::<CameraCmd>(16);

    // Compute initial timeout from current exposure
    let exposure_us = cam
        .get_control(ControlType::Exposure)
        .map(|(v, _)| v)
        .unwrap_or(100_000);
    let base_timeout = ((exposure_us / 1000) * 2 + 500).max(2000) as i32;

    let handle = CameraHandle {
        info: info.clone(),
        property: prop,
        controls,
        cmd_tx: cmd_tx.clone(),
    };

    let shift_bits = if needs_shift { 16 - bit_depth } else { 0 };
    let cam_name = info.name.clone();
    std::thread::spawn(move || {
        capture_loop(cam, &cam_name, frame_tx, cmd_rx, log_tx, bit_depth, base_timeout, shift_bits);
    });

    Ok(handle)
}

fn capture_loop(
    cam: Camera,
    cam_name: &str,
    frame_tx: Sender<super::FrameData>,
    cmd_rx: Receiver<CameraCmd>,
    log_tx: Sender<super::LogEntry>,
    bit_depth: u8,
    mut timeout_ms: i32,
    shift_bits: u8,
) {
    loop {
        // Process pending commands — batch all pending changes, then apply
        let mut controls_to_set: Vec<(ControlType, i64, bool)> = Vec::new();
        let mut should_stop = false;
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                CameraCmd::SetControl(ctrl, val, auto) => {
                    // Deduplicate: keep only the latest value per control type
                    if let Some(existing) = controls_to_set.iter_mut().find(|(c, _, _)| *c == ctrl) {
                        existing.1 = val;
                        existing.2 = auto;
                    } else {
                        controls_to_set.push((ctrl, val, auto));
                    }
                }
                CameraCmd::Stop => { should_stop = true; break; }
            }
        }
        if should_stop {
            let _ = cam.stop_capture();
            return;
        }
        // Apply batched control changes with capture stopped to avoid SDK lockups
        if !controls_to_set.is_empty() {
            let _ = cam.stop_capture();
            for (ctrl, val, auto) in &controls_to_set {
                if let Err(e) = cam.set_control(*ctrl, *val, *auto) {
                    let msg = format!("Set {:?} = {} (auto={}): {:?}", ctrl, val, auto, e);
                    let _ = log_tx.try_send(super::LogEntry::error(msg));
                }
                if *ctrl == ControlType::Exposure {
                    timeout_ms = ((val / 1000) * 2 + 500).max(2000) as i32;
                }
            }
            let _ = cam.start_capture();
        }

        match cam.get_image(timeout_ms) {
            Ok(img) => {
                let img = if shift_bits > 0 {
                    shift_image_right(img, shift_bits)
                } else {
                    img
                };
                let frame_data = super::process_image(img, bit_depth);
                if frame_tx.try_send(frame_data).is_err() && frame_tx.is_empty() {
                    let _ = cam.stop_capture();
                    return;
                }
            }
            Err(svbony::Error::Timeout) => continue,
            Err(svbony::Error::CameraRemoved) => {
                let _ = log_tx.try_send(super::LogEntry::error(
                    format!("{}: camera disconnected", cam_name),
                ));
                return;
            }
            Err(e) => {
                let _ = log_tx.try_send(super::LogEntry::error(
                    format!("{}: capture error: {:?}", cam_name, e),
                ));
                let _ = cam.stop_capture();
                return;
            }
        }
    }
}

/// Right-shift all pixel values in a 16-bit image to recover the true bit depth
/// when the SDK left-justifies data (e.g., 12-bit data stored in upper bits of u16).
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
