mod colormaps;
mod fits_source;
mod histogram;
mod imageview;
mod sim;
mod widgets;

#[cfg(feature = "svbony")]
mod camera;

use anyhow::Result;
use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use eframe::egui;
use image::DynamicImage;
use std::thread;
use std::time::Instant;

use colormaps::{Colormap, ColormapKind};
use histogram::{compute_histogram, compute_stats};
use imageview::{DisplayParams, ImageViewer};
use sim::SimCamera;

// ── Data types ──────────────────────────────────────────────────────────────

struct FrameData {
    mono: Vec<f64>,
    width: u32,
    height: u32,
    hist: histogram::Histogram,
    mean: f64,
    stddev: f64,
    bit_depth: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ScaleMode { Full, Auto, ZScale, Manual }

impl ScaleMode {
    const ALL: &'static [(ScaleMode, &'static str)] = &[
        (ScaleMode::Full, "Full Range"),
        (ScaleMode::Auto, "Auto (Min/Max)"),
        (ScaleMode::ZScale, "ZScale"),
        (ScaleMode::Manual, "Manual"),
    ];
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum HistDrag { Min, Max }

#[derive(Clone, PartialEq, Eq)]
enum CameraSource {
    Simulated,
    FitsFile(std::path::PathBuf),
    #[cfg(feature = "svbony")]
    SVBony(i32),
}

enum CaptureState {
    Sim { _stop_tx: Sender<()> },
    Fits { _stop_tx: Sender<()> },
    #[cfg(feature = "svbony")]
    SVBony {
        handle: camera::CameraHandle,
        control_values: Vec<(svbony::ControlType, i64, bool)>,
    },
    Stopped,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BottomTab { Histogram, Controls, Log }

// ── Log ─────────────────────────────────────────────────────────────────────

#[derive(Clone)]
enum LogLevel { Info, Warn, Error }

#[derive(Clone)]
struct LogEntry {
    timestamp: String,
    level: LogLevel,
    message: String,
}

impl LogEntry {
    fn now(level: LogLevel, message: String) -> Self {
        let t = chrono::Local::now().format("%H:%M:%S").to_string();
        Self { timestamp: t, level, message }
    }
    fn info(msg: String) -> Self { Self::now(LogLevel::Info, msg) }
    #[allow(dead_code)]
    fn warn(msg: String) -> Self { Self::now(LogLevel::Warn, msg) }
    fn error(msg: String) -> Self { Self::now(LogLevel::Error, msg) }
}

// ── App ─────────────────────────────────────────────────────────────────────

struct ViewerApp {
    frame_tx: Sender<FrameData>,
    frame_rx: Receiver<FrameData>,
    current_frame: Option<FrameData>,

    display_params: DisplayParams,
    colormap: Colormap,
    scale_mode: ScaleMode,
    image_viewer: ImageViewer,
    zoom_texture: Option<egui::TextureHandle>,
    zoom_rgba: Vec<u8>,

    cursor_pixel: Option<(u32, u32)>,
    cursor_value: Option<f64>,
    hist_drag: Option<HistDrag>,
    hist_log_y: bool,

    frame_times: Vec<Instant>,
    fps: f64,

    camera_source: CameraSource,
    capture_state: CaptureState,
    capture_running: bool,
    recording: bool,

    sim_width: u32,
    sim_height: u32,
    sim_bit_depth: u8,
    sim_fps: u32,

    bottom_tab: BottomTab,

    // Log
    log: Vec<LogEntry>,
    log_rx: Receiver<LogEntry>,
    log_tx: Sender<LogEntry>,

    // Async file dialog result
    pending_fits_path: Option<Receiver<Option<std::path::PathBuf>>>,
    // Async FITS loading result
    pending_fits_load: Option<Receiver<Result<(std::path::PathBuf, fits_source::FitsSource), String>>>,

    #[cfg(feature = "svbony")]
    discovered_cameras: Vec<svbony::CameraInfo>,
    #[cfg(feature = "svbony")]
    camera_error: Option<String>,
}

impl ViewerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Load system fonts
        let mut fonts = egui::FontDefinitions::default();
        if let Ok(sf_data) = std::fs::read("/System/Library/Fonts/SFNS.ttf") {
            let mut font_data = egui::FontData::from_owned(sf_data);
            font_data.tweak.y_offset_factor = -0.02;
            fonts.font_data.insert("sf_pro".to_owned(), font_data.into());
            fonts.families.entry(egui::FontFamily::Proportional).or_default().insert(0, "sf_pro".to_owned());
        }
        if let Ok(sf_mono) = std::fs::read("/System/Library/Fonts/SFNSMono.ttf") {
            let mut font_data = egui::FontData::from_owned(sf_mono);
            font_data.tweak.scale = 0.95;
            font_data.tweak.y_offset_factor = -0.02;
            fonts.font_data.insert("sf_mono".to_owned(), font_data.into());
            fonts.families.entry(egui::FontFamily::Monospace).or_default().insert(0, "sf_mono".to_owned());
        }
        cc.egui_ctx.set_fonts(fonts);

        // Theme
        let mut style = (*cc.egui_ctx.style()).clone();
        style.text_styles.insert(egui::TextStyle::Body, egui::FontId::new(13.0, egui::FontFamily::Proportional));
        style.text_styles.insert(egui::TextStyle::Heading, egui::FontId::new(15.0, egui::FontFamily::Proportional));
        style.text_styles.insert(egui::TextStyle::Button, egui::FontId::new(13.0, egui::FontFamily::Proportional));
        style.text_styles.insert(egui::TextStyle::Monospace, egui::FontId::new(12.5, egui::FontFamily::Monospace));
        style.text_styles.insert(egui::TextStyle::Small, egui::FontId::new(11.0, egui::FontFamily::Proportional));

        style.spacing.item_spacing = egui::vec2(6.0, 4.0);
        style.spacing.button_padding = egui::vec2(12.0, 6.0);
        style.spacing.slider_width = 140.0;
        style.spacing.icon_width = 16.0;
        style.spacing.icon_spacing = 6.0;
        style.spacing.combo_width = 110.0;

        use crate::widgets::*;
        let r = egui::CornerRadius::same(6);
        style.visuals.widgets.noninteractive.corner_radius = r;
        style.visuals.widgets.inactive.corner_radius = r;
        style.visuals.widgets.hovered.corner_radius = r;
        style.visuals.widgets.active.corner_radius = r;
        style.visuals.panel_fill = BG_SURFACE;
        style.visuals.window_fill = egui::Color32::WHITE;
        style.visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(229, 231, 235));
        style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, TEXT_PRIMARY);
        style.visuals.widgets.inactive.bg_fill = egui::Color32::WHITE;
        style.visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, BORDER);
        style.visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.5, TEXT_SECONDARY);
        style.visuals.widgets.hovered.bg_fill = BG_HOVER;
        style.visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.5, ACCENT_LIGHT);
        style.visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.5, ACCENT);
        style.visuals.widgets.active.bg_fill = egui::Color32::from_rgb(238, 239, 244);
        style.visuals.widgets.active.bg_stroke = egui::Stroke::new(1.5, ACCENT);
        style.visuals.widgets.active.fg_stroke = egui::Stroke::new(2.0, ACCENT);
        style.visuals.selection.bg_fill = ACCENT;
        style.visuals.selection.stroke = egui::Stroke::new(1.5, egui::Color32::WHITE);
        style.visuals.hyperlink_color = ACCENT;
        style.visuals.window_shadow = egui::Shadow {
            offset: [0, 4], blur: 12, spread: 0,
            color: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 15),
        };
        cc.egui_ctx.set_style(style);

        let (frame_tx, frame_rx) = bounded(2);
        let (log_tx, log_rx) = bounded(64);

        let sim_width = 1280u32;
        let sim_height = 960u32;
        let sim_bit_depth = 12u8;
        let sim_fps = 30u32;

        let mut log = Vec::new();
        log.push(LogEntry::info("Viewer started".to_string()));

        // Try to start with an SVBony camera if available, else fall back to simulated
        #[cfg(feature = "svbony")]
        let discovered_cameras = camera::enumerate();

        #[cfg(feature = "svbony")]
        let mut camera_error: Option<String> = None;

        let (camera_source, capture_state, capture_running);

        #[cfg(feature = "svbony")]
        {
            if let Some(info) = discovered_cameras.first() {
                match camera::start_camera(info, frame_tx.clone(), log_tx.clone()) {
                    Ok(handle) => {
                        let mut control_values = Vec::new();
                        for caps in &handle.controls {
                            control_values.push((caps.control_type, caps.default_value, false));
                        }
                        log.push(LogEntry::info(format!("Camera opened: {}", info.name)));
                        camera_source = CameraSource::SVBony(info.camera_id);
                        capture_state = CaptureState::SVBony { handle, control_values };
                        capture_running = true;
                    }
                    Err(e) => {
                        let msg = format!("Failed to open camera: {}", e);
                        log.push(LogEntry::error(msg.clone()));
                        camera_error = Some(msg);
                        // Fall back to simulated
                        let (stop_tx, stop_rx) = bounded(1);
                        start_sim_capture(frame_tx.clone(), stop_rx, sim_width, sim_height, sim_bit_depth, sim_fps);
                        camera_source = CameraSource::Simulated;
                        capture_state = CaptureState::Sim { _stop_tx: stop_tx };
                        capture_running = true;
                    }
                }
            } else {
                let (stop_tx, stop_rx) = bounded(1);
                start_sim_capture(frame_tx.clone(), stop_rx, sim_width, sim_height, sim_bit_depth, sim_fps);
                camera_source = CameraSource::Simulated;
                capture_state = CaptureState::Sim { _stop_tx: stop_tx };
                capture_running = true;
            }
        }

        #[cfg(not(feature = "svbony"))]
        {
            let (stop_tx, stop_rx) = bounded(1);
            start_sim_capture(frame_tx.clone(), stop_rx, sim_width, sim_height, sim_bit_depth, sim_fps);
            camera_source = CameraSource::Simulated;
            capture_state = CaptureState::Sim { _stop_tx: stop_tx };
            capture_running = true;
        }

        Self {
            frame_tx, frame_rx,
            current_frame: None,
            display_params: DisplayParams { scale_min: 0.0, scale_max: 4095.0, ..Default::default() },
            colormap: Colormap::new(ColormapKind::Grayscale),
            scale_mode: ScaleMode::Auto,
            image_viewer: ImageViewer::new(),
            zoom_texture: None,
            zoom_rgba: Vec::new(),
            cursor_pixel: None, cursor_value: None,
            hist_drag: None,
            hist_log_y: false,
            frame_times: Vec::new(), fps: 0.0,
            camera_source, capture_state, capture_running,
            recording: false,
            sim_width, sim_height, sim_bit_depth, sim_fps,
            bottom_tab: BottomTab::Histogram,
            log, log_rx, log_tx,
            pending_fits_path: None,
            pending_fits_load: None,
            #[cfg(feature = "svbony")]
            discovered_cameras,
            #[cfg(feature = "svbony")]
            camera_error,
        }
    }

    fn add_log(&mut self, entry: LogEntry) {
        self.log.push(entry);
        if self.log.len() > 500 { self.log.remove(0); }
    }

    fn poll_log(&mut self) {
        while let Ok(entry) = self.log_rx.try_recv() {
            self.add_log(entry);
        }
    }

    fn stop_capture(&mut self) {
        match std::mem::replace(&mut self.capture_state, CaptureState::Stopped) {
            CaptureState::Sim { _stop_tx } => {}
            CaptureState::Fits { _stop_tx } => {}
            #[cfg(feature = "svbony")]
            CaptureState::SVBony { handle, .. } => {
                let _ = handle.cmd_tx.send(camera::CameraCmd::Stop);
            }
            CaptureState::Stopped => {}
        }
        self.capture_running = false;
        self.frame_times.clear();
        self.fps = 0.0;
        while self.frame_rx.try_recv().is_ok() {}
    }

    fn start_sim(&mut self) {
        self.stop_capture();
        let (stop_tx, stop_rx) = bounded(1);
        start_sim_capture(self.frame_tx.clone(), stop_rx, self.sim_width, self.sim_height, self.sim_bit_depth, self.sim_fps);
        self.capture_state = CaptureState::Sim { _stop_tx: stop_tx };
        self.camera_source = CameraSource::Simulated;
        self.capture_running = true;
        self.add_log(LogEntry::info("Simulated camera started".to_string()));
    }

    fn start_fits(&mut self, path: std::path::PathBuf) {
        self.stop_capture();
        self.add_log(LogEntry::info(format!(
            "Loading FITS: {}...",
            path.file_name().unwrap_or_default().to_string_lossy()
        )));
        self.camera_source = CameraSource::FitsFile(path.clone());

        // Load in background thread to avoid freezing the UI
        let (tx, rx) = bounded(1);
        self.pending_fits_load = Some(rx);
        std::thread::spawn(move || {
            let path_str = path.to_str().unwrap_or("").to_string();
            match fits_source::FitsSource::from_file(&path_str) {
                Ok(source) => { let _ = tx.send(Ok((path, source))); }
                Err(e) => { let _ = tx.send(Err(format!("{}", e))); }
            }
        });
    }

    fn poll_fits_load(&mut self) {
        if let Some(rx) = &self.pending_fits_load {
            if let Ok(result) = rx.try_recv() {
                self.pending_fits_load = None;
                match result {
                    Ok((path, source)) => {
                        let nframes = source.num_frames();
                        let w = source.width;
                        let h = source.height;
                        let bd = source.bit_depth;
                        let (stop_tx, stop_rx) = bounded(1);
                        start_fits_capture(self.frame_tx.clone(), stop_rx, source, self.sim_fps);
                        self.capture_state = CaptureState::Fits { _stop_tx: stop_tx };
                        self.capture_running = true;
                        self.add_log(LogEntry::info(format!(
                            "FITS: {} ({}x{}, {}-bit, {} frames)",
                            path.file_name().unwrap_or_default().to_string_lossy(), w, h, bd, nframes
                        )));
                    }
                    Err(e) => {
                        self.add_log(LogEntry::error(format!("Failed to open FITS: {}", e)));
                    }
                }
            }
        }
    }

    #[cfg(feature = "svbony")]
    fn start_svbony(&mut self, info: &svbony::CameraInfo) {
        self.stop_capture();
        self.camera_error = None;

        match camera::start_camera(info, self.frame_tx.clone(), self.log_tx.clone()) {
            Ok(handle) => {
                let mut control_values = Vec::new();
                for caps in &handle.controls {
                    control_values.push((caps.control_type, caps.default_value, false));
                }
                self.add_log(LogEntry::info(format!("Camera opened: {}", info.name)));
                let camera_id = info.camera_id;
                self.capture_state = CaptureState::SVBony { handle, control_values };
                self.camera_source = CameraSource::SVBony(camera_id);
                self.capture_running = true;
            }
            Err(e) => {
                let msg = format!("Failed to open camera: {}", e);
                self.camera_error = Some(msg.clone());
                self.add_log(LogEntry::error(msg));
            }
        }
    }

    fn poll_frame(&mut self) {
        let mut latest = None;
        loop {
            match self.frame_rx.try_recv() {
                Ok(frame) => latest = Some(frame),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => { self.capture_running = false; break; }
            }
        }
        if let Some(frame) = latest {
            match self.scale_mode {
                ScaleMode::Auto => {
                    let new_min = snap_floor(frame.hist.data_min, 100.0);
                    let new_max = snap_ceil(frame.hist.data_max, 100.0);
                    if new_min != self.display_params.scale_min || new_max != self.display_params.scale_max {
                        self.display_params.scale_min = new_min;
                        self.display_params.scale_max = new_max;
                    }
                }
                ScaleMode::ZScale => {
                    let (zmin, zmax) = zscale(&frame.mono);
                    self.display_params.scale_min = zmin;
                    self.display_params.scale_max = zmax;
                }
                ScaleMode::Full => {
                    self.display_params.scale_min = 0.0;
                    self.display_params.scale_max = ((1u64 << frame.bit_depth) - 1) as f64;
                }
                ScaleMode::Manual => {}
            }
            let now = Instant::now();
            self.frame_times.push(now);
            while self.frame_times.len() > 30 { self.frame_times.remove(0); }
            if self.frame_times.len() >= 2 {
                let dt = self.frame_times.last().unwrap().duration_since(self.frame_times[0]);
                self.fps = (self.frame_times.len() - 1) as f64 / dt.as_secs_f64();
            }
            self.current_frame = Some(frame);
        }
    }

    // ── Side panel ──────────────────────────────────────────────────────────

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        // Poll pending FITS file dialog result (outside section closure)
        if let Some(rx) = &self.pending_fits_path {
            if let Ok(result) = rx.try_recv() {
                if let Some(path) = result {
                    self.start_fits(path);
                }
                self.pending_fits_path = None;
            }
        }

        section(ui, "Camera", |ui| {
            let mut new_source = self.camera_source.clone();

            // Only show simulated option if no real cameras are available
            #[cfg(feature = "svbony")]
            let has_cameras = !self.discovered_cameras.is_empty();
            #[cfg(not(feature = "svbony"))]
            let has_cameras = false;

            if !has_cameras {
                ui.radio_value(&mut new_source, CameraSource::Simulated, "Simulated");
            }

            // FITS file source
            if let CameraSource::FitsFile(path) = &self.camera_source {
                let name = path.file_name().unwrap_or_default().to_string_lossy().to_string();
                ui.radio_value(&mut new_source, self.camera_source.clone(), format!("FITS: {}", name));
            }
            let dialog_pending = self.pending_fits_path.is_some();
            if !dialog_pending && widgets::styled_button(ui, "Open FITS...") {
                let (tx, rx) = bounded(1);
                self.pending_fits_path = Some(rx);
                std::thread::spawn(move || {
                    let result = rfd::FileDialog::new()
                        .add_filter("FITS", &["fits", "fit", "fts"])
                        .pick_file();
                    let _ = tx.send(result);
                });
            }

            #[cfg(feature = "svbony")]
            {
                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    ui.label("SVBony Cameras:");
                    if widgets::styled_button(ui, "Refresh") {
                        self.discovered_cameras = camera::enumerate();
                    }
                });
                if self.discovered_cameras.is_empty() {
                    ui.label(egui::RichText::new("No cameras found").color(widgets::TEXT_SECONDARY).italics());
                } else {
                    for cam_info in &self.discovered_cameras.clone() {
                        let source = CameraSource::SVBony(cam_info.camera_id);
                        let label = format!("{} ({})", cam_info.name, cam_info.serial);
                        ui.radio_value(&mut new_source, source, label);
                    }
                }
                if let Some(err) = &self.camera_error {
                    ui.label(egui::RichText::new(err).color(egui::Color32::from_rgb(220, 38, 38)).small());
                }
            }

            if new_source != self.camera_source {
                match &new_source {
                    CameraSource::Simulated => self.start_sim(),
                    CameraSource::FitsFile(path) => {
                        let path = path.clone();
                        self.start_fits(path);
                    }
                    #[cfg(feature = "svbony")]
                    CameraSource::SVBony(cam_id) => {
                        let cam_id = *cam_id;
                        if let Some(info) = self.discovered_cameras.iter().find(|c| c.camera_id == cam_id).cloned() {
                            self.start_svbony(&info);
                        }
                    }
                }
            }

            if self.camera_source == CameraSource::Simulated {
                ui.add_space(4.0);
                ui.label("Resolution:");
                ui.horizontal(|ui| {
                    ui.add(egui::DragValue::new(&mut self.sim_width).range(64..=4096).speed(8).prefix("W: "));
                    ui.add(egui::DragValue::new(&mut self.sim_height).range(64..=4096).speed(8).prefix("H: "));
                });
                widgets::combo_box(ui, "bit_depth", "Bit depth:", &mut self.sim_bit_depth, &[
                    (8, "8"), (12, "12"), (14, "14"), (16, "16"),
                ]);
                widgets::styled_slider_u32(ui, &mut self.sim_fps, 1..=60, "FPS");
            }
        });

        ui.add_space(4.0);

        section(ui, "Display", |ui| {
            let cmap_options: Vec<(ColormapKind, &str)> = ColormapKind::ALL.iter().map(|&k| (k, k.name())).collect();
            let label_w = 65.0;
            egui::Grid::new("display_grid").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Colormap"); });
                if widgets::combo_box(ui, "colormap", "", &mut self.colormap.kind, &cmap_options) {
                    self.colormap = Colormap::new(self.colormap.kind);
                }
                ui.end_row();

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Scale"); });
                widgets::combo_box(ui, "scale_mode", "", &mut self.scale_mode, ScaleMode::ALL);
                ui.end_row();

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Transfer"); });
                widgets::combo_box(ui, "transfer_fn", "", &mut self.display_params.transfer, imageview::TransferFn::ALL);
                ui.end_row();

                let gamma_label = match self.display_params.transfer {
                    imageview::TransferFn::Linear => "Gamma",
                    imageview::TransferFn::Asinh => "Alpha",
                };
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label(gamma_label); });
                ui.horizontal(|ui| { widgets::styled_slider(ui, &mut self.display_params.gamma, 0.1..=10.0, ""); });
                ui.end_row();
            });
            ui.horizontal(|ui| {
                ui.add_space(label_w + 8.0);
                let reset_label = match self.display_params.transfer {
                    imageview::TransferFn::Linear => "Reset Gamma",
                    imageview::TransferFn::Asinh => "Reset Alpha",
                };
                if widgets::styled_button(ui, reset_label) { self.display_params.gamma = 1.0; }
            });
            if self.scale_mode == ScaleMode::Manual {
                ui.add_space(4.0);
                let max_range = self.current_frame.as_ref().map(|f| ((1u64 << f.bit_depth) - 1) as f64).unwrap_or(65535.0);
                widgets::styled_slider_f64(ui, &mut self.display_params.scale_min, 0.0..=max_range, "Min");
                widgets::styled_slider_f64(ui, &mut self.display_params.scale_max, 0.0..=max_range, "Max");
            } else {
                ui.label(format!("Range: {:.0} – {:.0}", self.display_params.scale_min, self.display_params.scale_max));
            }
            ui.add_space(6.0);
            widgets::styled_checkbox(ui, &mut self.display_params.show_axes, "Show Axes");
            widgets::styled_checkbox(ui, &mut self.display_params.show_colorbar, "Show Colorbar");
        });

        ui.add_space(4.0);

        section(ui, "Statistics", |ui| {
            let lw = 65.0;
            egui::Grid::new("stats_grid").num_columns(2).spacing([8.0, 3.0]).show(ui, |ui| {
                stat_row(ui, lw, "FPS", &format!("{:.1}", self.fps));
                if let Some(frame) = &self.current_frame {
                    stat_row(ui, lw, "Size", &format!("{} x {}", frame.width, frame.height));
                    stat_row(ui, lw, "Bit depth", &format!("{}", frame.bit_depth));
                    stat_row(ui, lw, "Mean", &format!("{:.1}", frame.mean));
                    stat_row(ui, lw, "Std Dev", &format!("{:.1}", frame.stddev));
                }
            });
            ui.add_space(4.0);
            if let (Some((px, py)), Some(val)) = (self.cursor_pixel, self.cursor_value) {
                ui.label(egui::RichText::new(format!("({}, {}) = {:.0}", px, py, val)).monospace());
            } else {
                ui.label(egui::RichText::new("---").monospace().weak());
            }
        });
    }

    // ── Bottom panel tabs ───────────────────────────────────────────────────

    fn bottom_panel_tabs(&mut self, ui: &mut egui::Ui) {
        let tab_bar_color = egui::Color32::from_rgb(37, 37, 38);
        let active_bg = egui::Color32::from_rgb(30, 30, 30);
        let inactive_text = egui::Color32::from_rgb(150, 150, 150);
        let active_text = egui::Color32::WHITE;
        let accent_line = widgets::ACCENT;

        let avail = ui.available_rect_before_wrap();
        let bar_rect = egui::Rect::from_min_size(avail.min, egui::vec2(avail.width(), 28.0));
        ui.painter().rect_filled(bar_rect, egui::CornerRadius::ZERO, tab_bar_color);

        #[allow(deprecated)]
        ui.allocate_ui_at_rect(bar_rect, |ui| {
            ui.horizontal_centered(|ui| {
                ui.spacing_mut().item_spacing.x = 0.0;

                let tabs = [
                    (BottomTab::Histogram, "Histogram"),
                    (BottomTab::Controls, "Controls"),
                    (BottomTab::Log, "Log"),
                ];

                for (tab, label) in tabs {
                    let is_active = self.bottom_tab == tab;
                    let font = egui::FontId::new(12.0, egui::FontFamily::Proportional);
                    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), active_text);
                    let tab_w = galley.size().x + 24.0;
                    let tab_rect = egui::Rect::from_min_size(
                        ui.cursor().min,
                        egui::vec2(tab_w, 28.0),
                    );
                    let resp = ui.allocate_rect(tab_rect, egui::Sense::click());

                    // Background
                    if is_active {
                        ui.painter().rect_filled(tab_rect, egui::CornerRadius::ZERO, active_bg);
                        // Active indicator line at top
                        ui.painter().hline(
                            tab_rect.x_range(),
                            tab_rect.min.y,
                            egui::Stroke::new(2.0, accent_line),
                        );
                    } else if resp.hovered() {
                        ui.painter().rect_filled(tab_rect, egui::CornerRadius::ZERO,
                            egui::Color32::from_rgb(45, 45, 46));
                    }

                    // Label
                    let text_color = if is_active { active_text } else { inactive_text };
                    ui.painter().text(
                        tab_rect.center(),
                        egui::Align2::CENTER_CENTER,
                        label,
                        font,
                        text_color,
                    );

                    if resp.clicked() {
                        self.bottom_tab = tab;
                    }
                }

                // Show unread log count badge on Log tab
                let unread = self.log.iter().filter(|e| matches!(e.level, LogLevel::Error | LogLevel::Warn)).count();
                if unread > 0 && self.bottom_tab != BottomTab::Log {
                    // Already rendered tabs, no need for extra badge
                }
            });
        });

        ui.allocate_space(egui::vec2(0.0, 28.0));
    }

    fn histogram_content(&mut self, ui: &mut egui::Ui) {
        if let Some(frame) = &self.current_frame {
            let hist = &frame.hist;
            let centers = hist.centers();
            let bin_width = if centers.len() > 1 { centers[1] - centers[0] } else { 1.0 };

            let mut line_vec: Vec<[f64; 2]> = Vec::with_capacity(centers.len() * 2);
            for (&cx, &cy) in centers.iter().zip(hist.counts.iter()) {
                let y = if self.hist_log_y {
                    (cy as f64 + 1.0).log10()
                } else {
                    cy as f64
                };
                line_vec.push([cx - bin_width * 0.5, y]);
                line_vec.push([cx + bin_width * 0.5, y]);
            }

            // Log Y toggle — placed in a horizontal bar above the plot
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 4.0;
                widgets::styled_checkbox(ui, &mut self.hist_log_y, "Log Y");
            });

            let plot_height = ui.available_height().max(80.0);
            let y_label = if self.hist_log_y { "log₁₀(count+1)" } else { "" };

            // Fix x-axis range to the bit depth so it doesn't bounce
            let x_max = self.current_frame.as_ref()
                .map(|f| ((1u64 << f.bit_depth) - 1) as f64)
                .unwrap_or(65535.0);

            let plot_resp = egui_plot::Plot::new("histogram")
                .height(plot_height)
                .y_axis_label(y_label)
                .show_axes([true, false])
                .allow_drag(false).allow_zoom(false).allow_scroll(false).allow_boxed_zoom(false)
                .show_grid([true, false])
                .x_axis_label("Pixel Value")
                .include_x(0.0)
                .include_x(x_max)
                .include_y(0.0)
                .set_margin_fraction(egui::vec2(0.01, 0.0))
                .show(ui, |plot_ui| {
                    let line_points: egui_plot::PlotPoints = line_vec.into();
                    plot_ui.line(
                        egui_plot::Line::new(line_points)
                            .color(egui::Color32::from_rgb(79, 70, 229))
                            .width(1.5)
                            .fill(0.0)
                            .fill_alpha(0.35),
                    );
                    if self.scale_mode == ScaleMode::Manual {
                        let smin = self.display_params.scale_min;
                        let smax = self.display_params.scale_max;
                        let grab_radius_data = {
                            let bounds = plot_ui.plot_bounds();
                            (bounds.max()[0] - bounds.min()[0]) * 0.015
                        };
                        let dragging_min = matches!(self.hist_drag, Some(HistDrag::Min));
                        let dragging_max = matches!(self.hist_drag, Some(HistDrag::Max));
                        let mut near_min = dragging_min;
                        let mut near_max = dragging_max;
                        if !dragging_min && !dragging_max {
                            if let Some(ptr) = plot_ui.pointer_coordinate() {
                                let dist_min = (ptr.x - smin).abs();
                                let dist_max = (ptr.x - smax).abs();
                                if dist_min < grab_radius_data && dist_min <= dist_max { near_min = true; }
                                else if dist_max < grab_radius_data { near_max = true; }
                            }
                        }
                        // Min line
                        let (min_c, min_w) = if near_min {
                            (egui::Color32::from_rgb(252, 85, 85), 4.0)
                        } else {
                            (egui::Color32::from_rgba_unmultiplied(220, 50, 50, 200), 3.0)
                        };
                        plot_ui.vline(egui_plot::VLine::new(smin).color(min_c).width(min_w).style(egui_plot::LineStyle::Solid));

                        // Max line
                        let (max_c, max_w) = if near_max {
                            (egui::Color32::from_rgb(96, 165, 250), 4.0)
                        } else {
                            (egui::Color32::from_rgba_unmultiplied(59, 130, 246, 200), 3.0)
                        };
                        plot_ui.vline(egui_plot::VLine::new(smax).color(max_c).width(max_w).style(egui_plot::LineStyle::Solid));
                    }
                });

            // Drag handling
            if plot_resp.response.dragged_by(egui::PointerButton::Primary) {
                if let Some(ptr) = plot_resp.response.hover_pos() {
                    let transform = plot_resp.transform;
                    let plot_x = transform.value_from_position(ptr).x;
                    let grab_radius_data = {
                        let bounds = transform.bounds();
                        (bounds.max()[0] - bounds.min()[0]) * 0.02
                    };
                    if plot_resp.response.drag_started() {
                        let dist_min = (plot_x - self.display_params.scale_min).abs();
                        let dist_max = (plot_x - self.display_params.scale_max).abs();
                        if dist_min < grab_radius_data && dist_min <= dist_max { self.hist_drag = Some(HistDrag::Min); }
                        else if dist_max < grab_radius_data { self.hist_drag = Some(HistDrag::Max); }
                    }
                    let bit_max = self.current_frame.as_ref().map(|f| ((1u64 << f.bit_depth) - 1) as f64).unwrap_or(65535.0);
                    match self.hist_drag {
                        Some(HistDrag::Min) => {
                            self.scale_mode = ScaleMode::Manual;
                            self.display_params.scale_min = plot_x.max(0.0).min(self.display_params.scale_max - 1.0);
                        }
                        Some(HistDrag::Max) => {
                            self.scale_mode = ScaleMode::Manual;
                            self.display_params.scale_max = plot_x.max(self.display_params.scale_min + 1.0).min(bit_max);
                        }
                        None => {}
                    }
                }
            }
            if plot_resp.response.drag_stopped() { self.hist_drag = None; }
        }
    }

    #[cfg(feature = "svbony")]
    fn controls_content(&mut self, ui: &mut egui::Ui) {
        if let CaptureState::SVBony { ref handle, ref mut control_values } = self.capture_state {
            let label_w = 130.0;
            ui.style_mut().spacing.item_spacing.y = 6.0;
            egui::Grid::new("camera_controls_grid")
                .num_columns(3)
                .spacing([12.0, 7.0])
                .show(ui, |ui| {
                    for (caps, cv) in handle.controls.iter().zip(control_values.iter_mut()) {
                        let old_val = cv.1;
                        let old_auto = cv.2;

                        if !caps.is_writable {
                            ctrl_label(ui, label_w, &caps.name);
                            ui.label(""); // empty slider column
                            ui.label(egui::RichText::new(format_control_value(caps.control_type, cv.1)).monospace().size(12.0));
                            ui.end_row();
                            continue;
                        }

                        if caps.max_value - caps.min_value <= 1 {
                            ctrl_label(ui, label_w, "");
                            let mut on = cv.1 != 0;
                            if widgets::styled_checkbox(ui, &mut on, &caps.name) {
                                cv.1 = if on { 1 } else { 0 };
                            }
                            ui.label(""); // empty value column
                            ui.end_row();
                        } else {
                            ctrl_label(ui, label_w, &caps.name);
                            let mut v = cv.1 as f64;
                            let is_exposure = caps.control_type == svbony::ControlType::Exposure;
                            let changed = if is_exposure {
                                slider_log(ui, &mut v, caps.min_value.max(1) as f64, caps.max_value as f64)
                            } else {
                                slider_i64(ui, &mut v, caps.min_value as f64, caps.max_value as f64)
                            };
                            if changed {
                                cv.1 = v as i64;
                            }
                            ui.label(egui::RichText::new(format_control_value(caps.control_type, cv.1)).monospace().size(12.0));
                            ui.end_row();

                            if caps.is_auto_supported {
                                ctrl_label(ui, label_w, "");
                                let mut auto = cv.2;
                                if widgets::styled_checkbox(ui, &mut auto, "Auto") { cv.2 = auto; }
                                ui.label(""); // empty value column
                                ui.end_row();
                            }
                        }

                        if cv.1 != old_val || cv.2 != old_auto {
                            let _ = handle.cmd_tx.send(camera::CameraCmd::SetControl(caps.control_type, cv.1, cv.2));
                        }
                    }
                });
        } else {
            ui.vertical_centered(|ui| {
                ui.add_space(20.0);
                ui.label(egui::RichText::new("No camera connected").color(widgets::TEXT_SECONDARY));
            });
        }
    }

    fn log_content(&mut self, ui: &mut egui::Ui) {
        if widgets::styled_button(ui, "Clear") {
            self.log.clear();
        }
        ui.add_space(4.0);

        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for entry in self.log.iter().rev() {
                    let color = match entry.level {
                        LogLevel::Info => widgets::TEXT_SECONDARY,
                        LogLevel::Warn => egui::Color32::from_rgb(217, 119, 6),
                        LogLevel::Error => egui::Color32::from_rgb(220, 38, 38),
                    };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&entry.timestamp).monospace().size(11.0).color(widgets::TEXT_SECONDARY));
                        ui.label(egui::RichText::new(&entry.message).size(12.0).color(color));
                    });
                }
            });
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

#[cfg(feature = "svbony")]
fn format_control_value(ctrl: svbony::ControlType, value: i64) -> String {
    match ctrl {
        svbony::ControlType::CurrentTemperature | svbony::ControlType::TargetTemperature => {
            format!("{:.1} °C", value as f64 / 10.0)
        }
        svbony::ControlType::Exposure => {
            if value >= 1_000_000 { format!("{:.1} s", value as f64 / 1_000_000.0) }
            else if value >= 1_000 { format!("{:.1} ms", value as f64 / 1_000.0) }
            else { format!("{} µs", value) }
        }
        _ => format!("{}", value),
    }
}

#[cfg(feature = "svbony")]
fn slider_i64(ui: &mut egui::Ui, value: &mut f64, min: f64, max: f64) -> bool {
    let old = *value;
    let handle_r = 6.0;
    let desired_width = ui.available_width().max(500.0);
    let height = 20.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(desired_width, height), egui::Sense::click_and_drag());
    let track_left = rect.min.x + handle_r;
    let track_right = rect.max.x - handle_r;
    let track_width = track_right - track_left;
    if response.dragged() || response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            let t = ((pos.x - track_left) / track_width).clamp(0.0, 1.0) as f64;
            *value = (min + t * (max - min)).round();
        }
    }
    let t = ((*value - min) / (max - min)).clamp(0.0, 1.0) as f32;
    let painter = ui.painter_at(rect);
    let track_y = rect.center().y;
    let track_rect = egui::Rect::from_min_max(egui::pos2(track_left, track_y - 2.0), egui::pos2(track_right, track_y + 2.0));
    painter.rect_filled(track_rect, egui::CornerRadius::same(2), egui::Color32::from_rgb(215, 216, 222));
    let filled = egui::Rect::from_min_max(track_rect.min, egui::pos2(track_left + t * track_width, track_rect.max.y));
    if filled.width() > 1.0 { painter.rect_filled(filled, egui::CornerRadius::same(2), widgets::ACCENT); }
    let hx = track_left + t * track_width;
    let hc = egui::pos2(hx, track_y);
    let hovered = response.hovered() || response.dragged();
    let hbg = if hovered { egui::Color32::from_rgb(252, 252, 254) } else { egui::Color32::WHITE };
    painter.circle_filled(hc, handle_r, hbg);
    let hborder = if response.dragged() { widgets::ACCENT_DARK } else if hovered { widgets::ACCENT } else { widgets::BORDER };
    painter.circle_stroke(hc, handle_r, egui::Stroke::new(1.5, hborder));
    *value != old
}

/// Logarithmic slider — equal slider distance per order of magnitude.
/// Ideal for exposure (1µs to 2000s spans ~10 decades).
#[cfg(feature = "svbony")]
fn slider_log(ui: &mut egui::Ui, value: &mut f64, min: f64, max: f64) -> bool {
    let old = *value;
    let handle_r = 6.0;
    let desired_width = ui.available_width().max(500.0);
    let height = 20.0;
    let (rect, response) = ui.allocate_exact_size(egui::vec2(desired_width, height), egui::Sense::click_and_drag());
    let track_left = rect.min.x + handle_r;
    let track_right = rect.max.x - handle_r;
    let track_width = track_right - track_left;

    let log_min = min.max(1.0).ln();
    let log_max = max.ln();

    if response.dragged() || response.clicked() {
        if let Some(pos) = response.interact_pointer_pos() {
            let t = ((pos.x - track_left) / track_width).clamp(0.0, 1.0) as f64;
            *value = (log_min + t * (log_max - log_min)).exp().round();
            *value = value.clamp(min, max);
        }
    }

    let t = ((*value).max(1.0).ln() - log_min) / (log_max - log_min);
    let t = t.clamp(0.0, 1.0) as f32;

    let painter = ui.painter_at(rect);
    let track_y = rect.center().y;
    let track_rect = egui::Rect::from_min_max(egui::pos2(track_left, track_y - 2.0), egui::pos2(track_right, track_y + 2.0));
    painter.rect_filled(track_rect, egui::CornerRadius::same(2), egui::Color32::from_rgb(215, 216, 222));
    let filled = egui::Rect::from_min_max(track_rect.min, egui::pos2(track_left + t * track_width, track_rect.max.y));
    if filled.width() > 1.0 { painter.rect_filled(filled, egui::CornerRadius::same(2), widgets::ACCENT); }
    let hx = track_left + t * track_width;
    let hc = egui::pos2(hx, track_y);
    let hovered = response.hovered() || response.dragged();
    let hbg = if hovered { egui::Color32::from_rgb(252, 252, 254) } else { egui::Color32::WHITE };
    painter.circle_filled(hc, handle_r, hbg);
    let hborder = if response.dragged() { widgets::ACCENT_DARK } else if hovered { widgets::ACCENT } else { widgets::BORDER };
    painter.circle_stroke(hc, handle_r, egui::Stroke::new(1.5, hborder));
    *value != old
}

fn section(ui: &mut egui::Ui, title: &str, content: impl FnOnce(&mut egui::Ui)) {
    let border = egui::Color32::from_rgb(229, 231, 235);
    egui::Frame::new()
        .fill(egui::Color32::WHITE)
        .stroke(egui::Stroke::new(1.0, border))
        .corner_radius(egui::CornerRadius::same(8))
        .inner_margin(egui::Margin { left: 1, right: 1, top: 1, bottom: 1 })
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            let header_h = 24.0;
            let header_rect = {
                let avail = ui.available_rect_before_wrap();
                let rect = egui::Rect::from_min_size(avail.min, egui::vec2(avail.width(), header_h));
                ui.painter().rect_filled(rect, egui::CornerRadius { nw: 7, ne: 7, sw: 0, se: 0 }, egui::Color32::from_rgb(232, 234, 246));
                ui.painter().hline(rect.x_range(), rect.max.y, egui::Stroke::new(1.0, border));
                rect
            };
            ui.painter().text(
                egui::pos2(header_rect.min.x + 10.0, header_rect.center().y),
                egui::Align2::LEFT_CENTER,
                &title.to_uppercase(),
                egui::FontId::new(11.0, egui::FontFamily::Proportional),
                egui::Color32::from_rgb(75, 70, 110),
            );
            ui.allocate_space(egui::vec2(0.0, header_h));
            egui::Frame::new()
                .inner_margin(egui::Margin { left: 9, right: 9, top: 8, bottom: 8 })
                .show(ui, |ui| { ui.spacing_mut().item_spacing.y = 5.0; content(ui); });
        });
}

#[cfg(feature = "svbony")]
fn ctrl_label(ui: &mut egui::Ui, width: f32, text: &str) {
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        ui.set_width(width);
        ui.label(egui::RichText::new(text).size(13.0));
    });
}

fn stat_row(ui: &mut egui::Ui, label_width: f32, label: &str, value: &str) {
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_width); ui.label(label); });
    ui.label(egui::RichText::new(value).monospace());
    ui.end_row();
}

// ── Main update loop ────────────────────────────────────────────────────────

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_frame();
        self.poll_log();
        self.poll_fits_load();

        if self.capture_running || self.pending_fits_load.is_some() { ctx.request_repaint(); }

        // Top toolbar
        egui::TopBottomPanel::top("toolbar")
            .exact_height(38.0)
            .frame(egui::Frame::new()
                .fill(egui::Color32::from_rgb(243, 244, 246))
                .inner_margin(egui::Margin { left: 10, right: 10, top: 4, bottom: 6 })
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(229, 231, 235)))
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 6.0;
                    if self.capture_running {
                        if ui.button(egui::RichText::new("\u{23F9}  Stop").size(14.0)).clicked() {
                            self.stop_capture();
                        }
                    } else if ui.button(egui::RichText::new("\u{25B6}  Play").size(14.0)).clicked() {
                        match &self.camera_source.clone() {
                            CameraSource::Simulated => self.start_sim(),
                            CameraSource::FitsFile(path) => {
                                let path = path.clone();
                                self.start_fits(path);
                            }
                            #[cfg(feature = "svbony")]
                            CameraSource::SVBony(cam_id) => {
                                let cam_id = *cam_id;
                                if let Some(info) = self.discovered_cameras.iter().find(|c| c.camera_id == cam_id).cloned() {
                                    self.start_svbony(&info);
                                }
                            }
                        }
                    }
                    ui.separator();
                    if self.recording {
                        let btn = ui.button(egui::RichText::new("Stop Rec").size(14.0).color(egui::Color32::from_rgb(239, 68, 68)));
                        if btn.clicked() { self.recording = false; }
                        let t = ui.input(|i| i.time);
                        let alpha = ((t * 3.0).sin() * 0.5 + 0.5) as u8 * 200 + 55;
                        ui.painter().circle_filled(
                            egui::pos2(btn.rect.min.x + 8.0, btn.rect.center().y),
                            4.0, egui::Color32::from_rgba_unmultiplied(220, 40, 40, alpha),
                        );
                    } else if ui.button(egui::RichText::new("Record").size(14.0)).clicked() {
                        self.recording = true;
                    }
                    ui.separator();
                    let cmap_options: Vec<(ColormapKind, &str)> = ColormapKind::ALL.iter().map(|&k| (k, k.name())).collect();
                    if widgets::combo_box(ui, "toolbar_cmap", "", &mut self.colormap.kind, &cmap_options) {
                        self.colormap = Colormap::new(self.colormap.kind);
                    }
                    ui.separator();
                    ui.label(egui::RichText::new("Scale:").size(13.0));
                    widgets::combo_box(ui, "toolbar_scale", "", &mut self.scale_mode, ScaleMode::ALL);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 12.0;
                        ui.label(egui::RichText::new(format!("{:.1} fps", self.fps)).monospace().size(13.0).color(egui::Color32::from_rgb(79, 70, 229)));
                        if let (Some((px, py)), Some(val)) = (self.cursor_pixel, self.cursor_value) {
                            ui.label(egui::RichText::new(format!("({}, {}) = {:.0}", px, py, val)).monospace().size(13.0));
                        }
                    });
                });
            });

        // Side panel
        egui::SidePanel::left("controls")
            .resizable(true).default_width(220.0)
            .frame(egui::Frame::new()
                .fill(egui::Color32::from_rgb(249, 250, 251))
                .inner_margin(egui::Margin { left: 6, right: 6, top: 8, bottom: 6 })
            )
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| { self.side_panel(ui); });
            });

        // Bottom tabbed panel
        egui::TopBottomPanel::bottom("bottom_panel")
            .resizable(true)
            .default_height(300.0)
            .height_range(150.0..=600.0)
            .frame(egui::Frame::new()
                .fill(egui::Color32::from_rgb(249, 250, 251))
                .inner_margin(egui::Margin::ZERO)
            )
            .show(ctx, |ui| {
                ui.set_min_height(260.0);
                self.bottom_panel_tabs(ui);

                egui::Frame::new()
                    .inner_margin(egui::Margin { left: 4, right: 4, top: 0, bottom: 4 })
                    .show(ui, |ui| {
                        match self.bottom_tab {
                            BottomTab::Histogram => self.histogram_content(ui),
                            BottomTab::Controls => {
                                #[cfg(feature = "svbony")]
                                {
                                    egui::ScrollArea::vertical().show(ui, |ui| {
                                        self.controls_content(ui);
                                    });
                                }
                                #[cfg(not(feature = "svbony"))]
                                ui.label("Camera support not compiled in");
                            }
                            BottomTab::Log => self.log_content(ui),
                        }
                    });
            });

        // Central panel
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(frame) = &self.current_frame {
                let resp = self.image_viewer.show(ui, &frame.mono, frame.width, frame.height, &self.display_params, &self.colormap);
                self.cursor_pixel = resp.hovered_pixel;
                self.cursor_value = resp.hovered_value;
            } else {
                ui.vertical_centered(|ui| { ui.label("Waiting for frames..."); });
            }
        });

        // Zoom popup window
        self.show_zoom_window(ctx);
    }
}

impl ViewerApp {
    fn show_zoom_window(&mut self, ctx: &egui::Context) {
        let roi = match self.image_viewer.roi_rect {
            Some(r) => r,
            None => return,
        };
        let frame = match &self.current_frame {
            Some(f) => f,
            None => return,
        };

        let [x0, y0, x1, y1] = roi;
        let roi_w = (x1 - x0 + 1) as usize;
        let roi_h = (y1 - y0 + 1) as usize;
        if roi_w < 2 || roi_h < 2 { return; }

        // Build zoomed RGBA from the ROI sub-region
        let npix = roi_w * roi_h;
        self.zoom_rgba.resize(npix * 4, 255);

        let range = self.display_params.scale_max - self.display_params.scale_min;
        let inv_range = if range > 0.0 { 1.0 / range } else { 1.0 };
        let inv_gamma = if self.display_params.gamma != 0.0 { 1.0 / self.display_params.gamma } else { 1.0 };
        let apply_gamma = (self.display_params.gamma - 1.0).abs() > 1e-4;
        let asinh_alpha = self.display_params.gamma as f64;
        let asinh_norm = if matches!(self.display_params.transfer, imageview::TransferFn::Asinh) {
            let v = asinh_alpha.asinh();
            if v > 0.0 { 1.0 / v } else { 1.0 }
        } else { 1.0 };

        for ry in 0..roi_h {
            for rx in 0..roi_w {
                let src_idx = ((y0 as usize + ry) * frame.width as usize) + (x0 as usize + rx);
                let val = if src_idx < frame.mono.len() { frame.mono[src_idx] } else { 0.0 };
                let mut t = ((val - self.display_params.scale_min) * inv_range).clamp(0.0, 1.0) as f32;
                match self.display_params.transfer {
                    imageview::TransferFn::Linear => { if apply_gamma { t = t.powf(inv_gamma); } }
                    imageview::TransferFn::Asinh => { t = ((asinh_alpha * t as f64).asinh() * asinh_norm).clamp(0.0, 1.0) as f32; }
                }
                let rgb = self.colormap.lookup(t);
                let off = (ry * roi_w + rx) * 4;
                self.zoom_rgba[off] = rgb[0];
                self.zoom_rgba[off + 1] = rgb[1];
                self.zoom_rgba[off + 2] = rgb[2];
                self.zoom_rgba[off + 3] = 255;
            }
        }

        let color_image = egui::ColorImage::from_rgba_unmultiplied(
            [roi_w, roi_h],
            &self.zoom_rgba,
        );
        match &mut self.zoom_texture {
            Some(tex) => tex.set(color_image, egui::TextureOptions::NEAREST),
            None => {
                self.zoom_texture = Some(ctx.load_texture(
                    "zoom_image",
                    color_image,
                    egui::TextureOptions::NEAREST,
                ));
            }
        }

        // Close on Escape or X key
        let close_key = ctx.input(|i| {
            i.key_pressed(egui::Key::Escape) || i.key_pressed(egui::Key::X)
        });
        if close_key {
            self.image_viewer.roi_rect = None;
            self.zoom_texture = None;
            return;
        }

        let title = format!("Zoom  ({},{})–({},{})  {}×{}", x0, y0, x1, y1, roi_w, roi_h);
        let mut open = true;
        egui::Window::new(title)
            .open(&mut open)
            .default_size([400.0, 400.0])
            .resizable(true)
            .show(ctx, |ui| {
                if let Some(tex) = &self.zoom_texture {
                    let avail = ui.available_size();
                    let aspect = roi_w as f32 / roi_h as f32;
                    let (w, h) = if avail.x / avail.y > aspect {
                        (avail.y * aspect, avail.y)
                    } else {
                        (avail.x, avail.x / aspect)
                    };
                    let image = egui::Image::new(tex)
                        .fit_to_exact_size(egui::vec2(w, h))
                        .texture_options(egui::TextureOptions::NEAREST);
                    ui.add(image);
                }
            });

        if !open {
            self.image_viewer.roi_rect = None;
            self.zoom_texture = None;
        }
    }
}

// ── Sim capture ─────────────────────────────────────────────────────────────

fn start_fits_capture(tx: Sender<FrameData>, stop_rx: Receiver<()>, mut source: fits_source::FitsSource, target_fps: u32) {
    let bit_depth = source.bit_depth;
    thread::spawn(move || {
        let frame_interval = std::time::Duration::from_secs_f64(1.0 / target_fps as f64);
        loop {
            let t0 = Instant::now();
            match stop_rx.try_recv() {
                Ok(()) | Err(crossbeam_channel::TryRecvError::Disconnected) => break,
                Err(crossbeam_channel::TryRecvError::Empty) => {}
            }
            let img = source.next_frame();
            let frame_data = process_image(img, bit_depth);
            if tx.try_send(frame_data).is_err() && tx.is_empty() { break; }
            let elapsed = t0.elapsed();
            if elapsed < frame_interval { thread::sleep(frame_interval - elapsed); }
        }
    });
}

fn start_sim_capture(tx: Sender<FrameData>, stop_rx: Receiver<()>, width: u32, height: u32, bit_depth: u8, target_fps: u32) {
    thread::spawn(move || {
        let mut cam = SimCamera::new(width, height, bit_depth);
        let frame_interval = std::time::Duration::from_secs_f64(1.0 / target_fps as f64);
        loop {
            let t0 = Instant::now();
            match stop_rx.try_recv() {
                Ok(()) | Err(crossbeam_channel::TryRecvError::Disconnected) => break,
                Err(crossbeam_channel::TryRecvError::Empty) => {}
            }
            let img = cam.next_frame();
            let frame_data = process_image(img, bit_depth);
            if tx.try_send(frame_data).is_err() && tx.is_empty() { break; }
            let elapsed = t0.elapsed();
            if elapsed < frame_interval { thread::sleep(frame_interval - elapsed); }
        }
    });
}

fn process_image(img: DynamicImage, bit_depth: u8) -> FrameData {
    let width = img.width();
    let height = img.height();
    let mono: Vec<f64> = match &img {
        DynamicImage::ImageLuma8(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageLuma16(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageRgb8(rgb) => rgb.pixels().map(|p| 0.299 * p[0] as f64 + 0.587 * p[1] as f64 + 0.114 * p[2] as f64).collect(),
        _ => { let gray = img.to_luma8(); gray.as_raw().iter().map(|&v| v as f64).collect() }
    };
    let hist = compute_histogram(&mono, 256);
    let (mean, stddev) = compute_stats(&mono);
    FrameData { mono, width, height, hist, mean, stddev, bit_depth }
}

fn snap_floor(v: f64, step: f64) -> f64 { (v / step).floor() * step }
fn snap_ceil(v: f64, step: f64) -> f64 { (v / step).ceil() * step }

/// ZScale algorithm (simplified IRAF/DS9 style).
/// Samples pixels, sorts them, fits a line to the central portion,
/// and derives display min/max that rejects outliers.
fn zscale(data: &[f64]) -> (f64, f64) {
    if data.is_empty() { return (0.0, 1.0); }

    // Sample up to 10000 evenly spaced pixels
    let n_samples = data.len().min(10_000);
    let step = data.len() as f64 / n_samples as f64;
    let mut samples: Vec<f64> = (0..n_samples)
        .map(|i| data[(i as f64 * step) as usize])
        .collect();
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Remove the bottom and top 0.5% as extreme outliers
    let trim = (samples.len() as f64 * 0.005) as usize;
    let trimmed = if trim > 0 && samples.len() > 2 * trim + 2 {
        &samples[trim..samples.len() - trim]
    } else {
        &samples
    };

    let n = trimmed.len();
    if n < 2 { return (samples[0], samples[samples.len() - 1]); }

    // Fit a line: y = a + b*x where x is the index, y is the pixel value
    // This captures the "ramp" of the sorted distribution
    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = trimmed.iter().sum();
    let sum_xy: f64 = trimmed.iter().enumerate().map(|(i, &v)| i as f64 * v).sum();
    let sum_x2: f64 = (0..n).map(|i| (i as f64) * (i as f64)).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    let (_intercept, slope) = if denom.abs() > 1e-10 {
        let b = (n_f * sum_xy - sum_x * sum_y) / denom;
        let a = (sum_y - b * sum_x) / n_f;
        (a, b)
    } else {
        (trimmed[0], 0.0)
    };

    // The median value and its position
    let median = trimmed[n / 2];

    // Use the slope to determine display range:
    // zmin/zmax are median ± (n/2 * slope * contrast)
    let contrast = 0.25; // DS9 default-ish
    let half_range = (n as f64 / 2.0) * slope.abs() / contrast;

    let zmin = (median - half_range).max(trimmed[0]);
    let zmax = (median + half_range).min(trimmed[n - 1]);

    if zmax <= zmin {
        (trimmed[0], trimmed[n - 1])
    } else {
        (zmin, zmax)
    }
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1400.0, 1000.0]).with_title("Viewer"),
        ..Default::default()
    };
    eframe::run_native("Viewer", options, Box::new(|cc| Ok(Box::new(ViewerApp::new(cc)))))
        .map_err(|e| anyhow::anyhow!("{}", e))?;
    Ok(())
}
