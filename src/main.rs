mod colormaps;
mod fits_source;
mod histogram;
mod imageview;
mod overlays;
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
    mono: Vec<f32>,
    width: u32,
    height: u32,
    hist: histogram::Histogram,
    mean: f32,
    stddev: f32,
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

#[cfg(feature = "starsolve")]
#[derive(serde::Serialize, serde::Deserialize)]
struct SavedConfig {
    solver_db_path: String,
    fov_estimate_deg: f32,
    sigma_threshold: f32,
    min_pixels: usize,
    max_pixels: usize,
    max_centroids: Option<usize>,
    local_bg_block_size: Option<u32>,
    max_elongation: Option<f32>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BottomTab {
    Histogram,
    Controls,
    #[cfg(feature = "starsolve")]
    PlateSolve,
    Log,
}

// ── Recording ───────────────────────────────────────────────────────────────

enum RecordMsg {
    Frame { mono: Vec<f32>, width: u32, height: u32, date_obs: String, exptime_s: f64 },
    Stop,
}

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
    cursor_value: Option<f32>,
    hist_drag: Option<HistDrag>,
    hist_log_y: bool,

    // Overlay system
    overlay_items: Vec<overlays::OverlayItem>,
    #[cfg(feature = "starsolve")]
    show_centroids: bool,
    #[cfg(feature = "starsolve")]
    show_matched_stars: bool,
    #[cfg(feature = "starsolve")]
    centroid_config: tetra3::CentroidExtractionConfig,
    #[cfg(feature = "starsolve")]
    centroid_count: usize,
    #[cfg(feature = "starsolve")]
    centroid_work_tx: Sender<(Vec<f32>, u32, u32, tetra3::CentroidExtractionConfig)>,
    #[cfg(feature = "starsolve")]
    centroid_result_rx: Receiver<Vec<overlays::OverlayItem>>,
    #[cfg(feature = "starsolve")]
    solver_db: Option<std::sync::Arc<tetra3::SolverDatabase>>,
    #[cfg(feature = "starsolve")]
    solver_db_path: Option<std::path::PathBuf>,
    #[cfg(feature = "starsolve")]
    fov_estimate_deg: f32,
    #[cfg(feature = "starsolve")]
    solve_rx: Option<Receiver<tetra3::SolveResult>>,
    #[cfg(feature = "starsolve")]
    last_solve: Option<tetra3::SolveResult>,

    frame_times: Vec<Instant>,
    fps: f64,

    camera_source: CameraSource,
    capture_state: CaptureState,
    capture_running: bool,
    recording: bool,
    rec_tx: Option<Sender<RecordMsg>>,
    rec_filename: String,
    rec_frame_count: u32,

    sim_width: u32,
    sim_height: u32,
    sim_bit_depth: u8,
    sim_fps: u32,
    fits_fps: u32,

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

        // Persistent centroid worker thread
        #[cfg(feature = "starsolve")]
        let (centroid_work_tx, centroid_result_rx) = {
            let (work_tx, work_rx) = bounded::<(Vec<f32>, u32, u32, tetra3::CentroidExtractionConfig)>(1);
            let (result_tx, result_rx) = bounded::<Vec<overlays::OverlayItem>>(1);
            std::thread::spawn(move || {
                while let Ok((pixels, w, h, config)) = work_rx.recv() {
                    let items = match tetra3::extract_centroids_from_raw(&pixels, w, h, &config) {
                        Ok(result) => result.centroids.iter()
                            .map(overlays::centroid_to_overlay)
                            .collect(),
                        Err(_) => Vec::new(),
                    };
                    if result_tx.send(items).is_err() { break; }
                }
            });
            (work_tx, result_rx)
        };

        let mut app = Self {
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
            overlay_items: Vec::new(),
            #[cfg(feature = "starsolve")]
            show_centroids: false,
            #[cfg(feature = "starsolve")]
            show_matched_stars: true,
            #[cfg(feature = "starsolve")]
            centroid_config: tetra3::CentroidExtractionConfig::default(),
            #[cfg(feature = "starsolve")]
            centroid_count: 0,
            #[cfg(feature = "starsolve")]
            centroid_work_tx: centroid_work_tx,
            #[cfg(feature = "starsolve")]
            centroid_result_rx: centroid_result_rx,
            #[cfg(feature = "starsolve")]
            solver_db: None,
            #[cfg(feature = "starsolve")]
            solver_db_path: None,
            #[cfg(feature = "starsolve")]
            fov_estimate_deg: 15.0,
            #[cfg(feature = "starsolve")]
            solve_rx: None,
            #[cfg(feature = "starsolve")]
            last_solve: None,
            frame_times: Vec::new(), fps: 0.0,
            camera_source, capture_state, capture_running,
            recording: false,
            rec_tx: None,
            rec_filename: String::new(),
            rec_frame_count: 0,
            sim_width, sim_height, sim_bit_depth, sim_fps,
            fits_fps: 10,
            bottom_tab: BottomTab::Histogram,
            log, log_rx, log_tx,
            pending_fits_path: None,
            pending_fits_load: None,
            #[cfg(feature = "svbony")]
            discovered_cameras,
            #[cfg(feature = "svbony")]
            camera_error,
        };

        #[cfg(feature = "starsolve")]
        app.load_config();

        app
    }

    #[cfg(feature = "starsolve")]
    fn config_path() -> std::path::PathBuf {
        let dir = dirs::config_dir().unwrap_or_else(|| std::path::PathBuf::from(".")).join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        dir.join("config.json")
    }

    #[cfg(feature = "starsolve")]
    fn save_config(&self) {
        let cfg = SavedConfig {
            solver_db_path: self.solver_db_path.as_ref().map(|p| p.display().to_string()).unwrap_or_default(),
            fov_estimate_deg: self.fov_estimate_deg,
            sigma_threshold: self.centroid_config.sigma_threshold,
            min_pixels: self.centroid_config.min_pixels,
            max_pixels: self.centroid_config.max_pixels,
            max_centroids: self.centroid_config.max_centroids,
            local_bg_block_size: self.centroid_config.local_bg_block_size,
            max_elongation: self.centroid_config.max_elongation,
        };
        if let Ok(json) = serde_json::to_string_pretty(&cfg) {
            let _ = std::fs::write(Self::config_path(), json);
        }
    }

    #[cfg(feature = "starsolve")]
    fn load_config(&mut self) {
        let config_path = Self::config_path();
        if let Ok(data) = std::fs::read_to_string(&config_path) {
            if let Ok(cfg) = serde_json::from_str::<SavedConfig>(&data) {
                if (1.0..=60.0).contains(&cfg.fov_estimate_deg) {
                    self.fov_estimate_deg = cfg.fov_estimate_deg;
                }
                self.centroid_config.sigma_threshold = cfg.sigma_threshold;
                self.centroid_config.min_pixels = cfg.min_pixels;
                self.centroid_config.max_pixels = cfg.max_pixels;
                self.centroid_config.max_centroids = cfg.max_centroids;
                self.centroid_config.local_bg_block_size = cfg.local_bg_block_size;
                self.centroid_config.max_elongation = cfg.max_elongation;

                if !cfg.solver_db_path.is_empty() && std::path::Path::new(&cfg.solver_db_path).exists() {
                    self.add_log(LogEntry::info(format!("Auto-loading database: {}", cfg.solver_db_path)));
                    match tetra3::SolverDatabase::load_from_file(&cfg.solver_db_path) {
                        Ok(db) => {
                            self.add_log(LogEntry::info(format!(
                                "Database loaded: {} patterns, {} stars",
                                db.props.num_patterns, db.star_vectors.len(),
                            )));
                            self.solver_db_path = Some(std::path::PathBuf::from(&cfg.solver_db_path));
                            self.solver_db = Some(std::sync::Arc::new(db));
                        }
                        Err(e) => {
                            self.add_log(LogEntry::error(format!("Auto-load failed: {}", e)));
                        }
                    }
                }
            }
        }
    }

    fn add_log(&mut self, entry: LogEntry) {
        self.log.push(entry);
        if self.log.len() > 500 { self.log.remove(0); }
    }

    fn start_recording(&mut self) {
        // Create data directory
        let data_dir = std::path::PathBuf::from("data");
        if let Err(e) = std::fs::create_dir_all(&data_dir) {
            self.add_log(LogEntry::error(format!("Failed to create data/: {}", e)));
            return;
        }

        let filename = format!("astroviewer-{}.fits", chrono::Local::now().format("%Y%m%d-%H%M%S"));
        let filepath = data_dir.join(&filename);

        let (tx, rx) = bounded::<RecordMsg>(16);
        let log_tx = self.log_tx.clone();
        let fname = filename.clone();

        thread::spawn(move || {
            use fits4::{FitsFile, Hdu, ImageData, PixelData, HeaderValue};

            let mut fits = FitsFile::with_empty_primary();
            fits.primary_mut().header.set("OBJECT", HeaderValue::String("Recording".into()), None);
            let mut frame_count: u32 = 0;

            while let Ok(msg) = rx.recv() {
                match msg {
                    RecordMsg::Frame { mono, width, height, date_obs, exptime_s } => {
                        // Convert f32 mono to i16 with BZERO=32768 for unsigned 16-bit
                        let pixels_i16: Vec<i16> = mono.iter().map(|&v| {
                            let clamped = v.clamp(0.0, 65535.0) as u16;
                            (clamped as i32 - 32768) as i16
                        }).collect();

                        let img = ImageData::new(
                            vec![width as usize, height as usize],
                            PixelData::I16(pixels_i16),
                        );
                        let mut hdu = Hdu::image_extension(img);
                        hdu.header.set("BZERO", HeaderValue::Float(32768.0), Some("unsigned 16-bit offset"));
                        hdu.header.set("BSCALE", HeaderValue::Float(1.0), Some("default scaling"));
                        hdu.header.set("DATE-OBS", HeaderValue::String(date_obs), Some("estimated mid-exposure UTC"));
                        hdu.header.set("EXPTIME", HeaderValue::Float(exptime_s), Some("exposure time in seconds"));
                        fits.push_extension(hdu);
                        frame_count += 1;
                    }
                    RecordMsg::Stop => break,
                }
            }

            // Write file
            if frame_count > 0 {
                match fits.to_file(&filepath) {
                    Ok(_) => {
                        let _ = log_tx.send(LogEntry::info(
                            format!("Recording saved: {} ({} frames)", fname, frame_count)
                        ));
                    }
                    Err(e) => {
                        let _ = log_tx.send(LogEntry::error(
                            format!("Failed to write {}: {}", fname, e)
                        ));
                    }
                }
            } else {
                let _ = log_tx.send(LogEntry::info("Recording cancelled (no frames)".to_string()));
            }
        });

        self.rec_tx = Some(tx);
        self.rec_filename = filename.clone();
        self.rec_frame_count = 0;
        self.recording = true;
        self.add_log(LogEntry::info(format!("Recording started: {}", filename)));
    }

    fn stop_recording(&mut self) {
        if let Some(tx) = self.rec_tx.take() {
            let _ = tx.send(RecordMsg::Stop);
        }
        self.recording = false;
        self.add_log(LogEntry::info(format!(
            "Recording stopped: {} ({} frames)", self.rec_filename, self.rec_frame_count
        )));
    }

    fn record_frame(&mut self, frame: &FrameData) {
        if let Some(tx) = &self.rec_tx {
            // Estimate exposure time from camera controls (microseconds)
            let exposure_us: f64 = {
                #[cfg(feature = "svbony")]
                {
                    if let CaptureState::SVBony { ref control_values, .. } = self.capture_state {
                        control_values.iter()
                            .find(|(ct, _, _)| *ct == svbony::ControlType::Exposure)
                            .map(|(_, v, _)| *v as f64)
                            .unwrap_or(0.0)
                    } else { 0.0 }
                }
                #[cfg(not(feature = "svbony"))]
                { 0.0 }
            };
            let exptime_s = exposure_us / 1_000_000.0;
            // Estimate mid-exposure: now is ~end of readout, so midpoint ≈ now - exposure/2
            let mid = chrono::Utc::now() - chrono::Duration::microseconds((exposure_us / 2.0) as i64);
            let date_obs = mid.format("%Y-%m-%dT%H:%M:%S%.3f").to_string();

            let msg = RecordMsg::Frame {
                mono: frame.mono.clone(),
                width: frame.width,
                height: frame.height,
                date_obs,
                exptime_s,
            };
            if tx.try_send(msg).is_ok() {
                self.rec_frame_count += 1;
            }
        }
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
        if self.recording {
            self.stop_recording();
        }
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
                        start_fits_capture(self.frame_tx.clone(), stop_rx, source, self.fits_fps);
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
                    let mono_f64: Vec<f64> = frame.mono.iter().map(|&v| v as f64).collect();
                    let (zmin, zmax) = zscale(&mono_f64);
                    self.display_params.scale_min = zmin as f32;
                    self.display_params.scale_max = zmax as f32;
                }
                ScaleMode::Full => {
                    self.display_params.scale_min = 0.0;
                    self.display_params.scale_max = ((1u64 << frame.bit_depth) - 1) as f32;
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
            // Dispatch centroid extraction to background thread (non-blocking)
            #[cfg(feature = "starsolve")]
            {
                // Poll for completed centroid results
                if let Ok(items) = self.centroid_result_rx.try_recv() {
                    self.centroid_count = items.len();
                    if self.show_centroids {
                        self.overlay_items = items;
                    } else {
                        self.overlay_items.clear();
                    }
                }

                // Always run centroid extraction (needed for plate solve + count)
                if self.centroid_work_tx.is_empty() {
                    let _ = self.centroid_work_tx.try_send((
                        frame.mono.clone(),
                        frame.width,
                        frame.height,
                        self.centroid_config.clone(),
                    ));
                }
            }

            // Auto plate-solve if database is loaded
            #[cfg(feature = "starsolve")]
            self.maybe_auto_solve(&frame);

            // Poll solve result
            #[cfg(feature = "starsolve")]
            if let Some(rx) = &self.solve_rx {
                if let Ok(result) = rx.try_recv() {
                    // Update FOV estimate from successful solve
                    if result.status == tetra3::SolveStatus::MatchFound {
                        if let Some(fov) = result.fov_rad {
                            self.fov_estimate_deg = fov.to_degrees();
                        }
                    }
                    self.last_solve = Some(result);
                    self.solve_rx = None;
                }
            }

            // Append matched star markers from last solve (every frame)
            #[cfg(feature = "starsolve")]
            if self.show_matched_stars {
                if let Some(ref result) = self.last_solve {
                    if result.status == tetra3::SolveStatus::MatchFound {
                        // Use matched centroid indices to mark which centroids were matched
                        let n_centroids = self.overlay_items.len();
                        for &cent_idx in &result.matched_centroid_indices {
                            if cent_idx < n_centroids {
                                if let overlays::OverlayItem::Centroid { x, y, .. } = &self.overlay_items[cent_idx] {
                                    self.overlay_items.push(overlays::OverlayItem::Marker {
                                        x: *x,
                                        y: *y,
                                        kind: overlays::MarkerKind::Crosshair,
                                        label: None,
                                    });
                                }
                            }
                        }
                    }
                }
            }

            // Record frame if recording
            if self.recording {
                self.record_frame(&frame);
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
                widgets::styled_slider_u32(ui, &mut self.fits_fps, 1..=60, "Playback FPS");
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
                let max_range = self.current_frame.as_ref().map(|f| ((1u64 << f.bit_depth) - 1) as f32).unwrap_or(65535.0);
                widgets::styled_slider(ui, &mut self.display_params.scale_min, 0.0..=max_range, "Min");
                widgets::styled_slider(ui, &mut self.display_params.scale_max, 0.0..=max_range, "Max");
            } else {
                ui.label(format!("Range: {:.0} – {:.0}", self.display_params.scale_min, self.display_params.scale_max));
            }
            ui.add_space(6.0);
            widgets::styled_checkbox(ui, &mut self.display_params.show_axes, "Show Axes");
            widgets::styled_checkbox(ui, &mut self.display_params.show_colorbar, "Show Colorbar");
            #[cfg(feature = "starsolve")]
            {
                widgets::styled_checkbox(ui, &mut self.show_centroids, "Show Centroids");
                widgets::styled_checkbox(ui, &mut self.show_matched_stars, "Show Matched Stars");
            }
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

                let mut tabs: Vec<(BottomTab, &str)> = vec![
                    (BottomTab::Histogram, "Histogram"),
                    (BottomTab::Controls, "Controls"),
                ];
                #[cfg(feature = "starsolve")]
                tabs.push((BottomTab::PlateSolve, "Plate Solve"));
                tabs.push((BottomTab::Log, "Log"));

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
                line_vec.push([(cx - bin_width * 0.5) as f64, y]);
                line_vec.push([(cx + bin_width * 0.5) as f64, y]);
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
                .unwrap_or(65535.0);  // f64 for egui_plot

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
                        let smin = self.display_params.scale_min as f64;
                        let smax = self.display_params.scale_max as f64;
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
                        let dist_min = (plot_x - self.display_params.scale_min as f64).abs();
                        let dist_max = (plot_x - self.display_params.scale_max as f64).abs();
                        if dist_min < grab_radius_data && dist_min <= dist_max { self.hist_drag = Some(HistDrag::Min); }
                        else if dist_max < grab_radius_data { self.hist_drag = Some(HistDrag::Max); }
                    }
                    let bit_max = self.current_frame.as_ref().map(|f| ((1u64 << f.bit_depth) - 1) as f64).unwrap_or(65535.0);
                    match self.hist_drag {
                        Some(HistDrag::Min) => {
                            self.scale_mode = ScaleMode::Manual;
                            self.display_params.scale_min = plot_x.max(0.0).min(self.display_params.scale_max as f64 - 1.0) as f32;
                        }
                        Some(HistDrag::Max) => {
                            self.scale_mode = ScaleMode::Manual;
                            self.display_params.scale_max = plot_x.max(self.display_params.scale_min as f64 + 1.0).min(bit_max) as f32;
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
                            let mut v = cv.1 as f32;
                            let is_exposure = caps.control_type == svbony::ControlType::Exposure;
                            if is_exposure {
                                let max_us = (caps.max_value as f32).min(1_000_000.0);
                                widgets::styled_slider_bare(ui, &mut v, caps.min_value as f32..=max_us);
                            } else {
                                widgets::styled_slider_bare(ui, &mut v, caps.min_value as f32..=caps.max_value as f32);
                            }
                            cv.1 = v.round() as i64;
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

    #[cfg(feature = "starsolve")]
    fn plate_solve_content(&mut self, ui: &mut egui::Ui) {
        // ── Top bar: database + FOV + status + reset ────────────────────────
        ui.horizontal(|ui| {
            // Database
            if self.solver_db.is_none() {
                if widgets::styled_button(ui, "Load Database...") {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Database", &["rkyv"])
                        .pick_file()
                    {
                        self.add_log(LogEntry::info(format!("Loading database: {}...", path.display())));
                        match tetra3::SolverDatabase::load_from_file(path.to_str().unwrap_or("")) {
                            Ok(db) => {
                                self.add_log(LogEntry::info(format!(
                                    "Database: {} patterns, {} stars, {:.1}°–{:.1}°",
                                    db.props.num_patterns, db.star_vectors.len(),
                                    db.props.min_fov_rad.to_degrees(), db.props.max_fov_rad.to_degrees(),
                                )));
                                self.solver_db = Some(std::sync::Arc::new(db));
                                self.solver_db_path = Some(path.clone());
                            }
                            Err(e) => self.add_log(LogEntry::error(format!("Load failed: {}", e))),
                        }
                    }
                }
            } else {
                ui.label(egui::RichText::new("DB").color(egui::Color32::from_rgb(34, 197, 94)));
                if widgets::styled_button(ui, "Unload") {
                    self.solver_db = None;
                    self.last_solve = None;
                }
            }

            ui.separator();

            // FOV
            ui.label("FOV:");
            ui.add(egui::DragValue::new(&mut self.fov_estimate_deg)
                .range(1.0..=60.0).speed(0.5).suffix("°").fixed_decimals(1));

            ui.separator();

            // Solve status
            if self.solver_db.is_some() {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(75.0, 18.0), egui::Sense::hover());
                let (text, color) = if self.solve_rx.is_some() {
                    ("Solving...", egui::Color32::from_rgb(217, 119, 6))
                } else if self.last_solve.as_ref().map_or(false, |s| s.status == tetra3::SolveStatus::MatchFound) {
                    ("Locked", egui::Color32::from_rgb(34, 197, 94))
                } else {
                    ("Searching...", widgets::TEXT_SECONDARY)
                };
                ui.painter().text(rect.left_center(), egui::Align2::LEFT_CENTER, text, egui::FontId::proportional(13.0), color);
            }

            ui.separator();

            // Centroid count (always shown, even when overlay is off)
            if self.centroid_count > 0 {
                ui.label(egui::RichText::new(format!("{} stars", self.centroid_count)).color(widgets::TEXT_SECONDARY));
            }

            // Reset defaults (far right)
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if widgets::styled_button(ui, "Reset") {
                    self.centroid_config = tetra3::CentroidExtractionConfig::default();
                }
            });
        });

        // ── Centroid parameters (compact horizontal grid) ───────────────────
        ui.add_space(2.0);
        // Centroid params — equal-width columns using allocate_ui
        let total_w = ui.available_width();
        let col_w = (total_w / 3.0 - 4.0).max(100.0);
        let label_w = 85.0;
        let slider_w = (col_w - label_w - 8.0).max(40.0);

        // Helper: fixed-width label
        let fixed_label = |ui: &mut egui::Ui, text: String| {
            ui.allocate_ui(egui::vec2(label_w, 20.0), |ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(egui::RichText::new(text).size(12.0));
                });
            });
        };

        // Row 1
        ui.horizontal(|ui| {
            fixed_label(ui, format!("SNR {:.1}", self.centroid_config.sigma_threshold));
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_bare(ui, &mut self.centroid_config.sigma_threshold, 1.0..=20.0);
            });

            let mut v = self.centroid_config.min_pixels as f32;
            fixed_label(ui, format!("Min px {}", self.centroid_config.min_pixels));
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_bare(ui, &mut v, 1.0..=50.0);
            });
            self.centroid_config.min_pixels = v.round() as usize;

            let mut v = self.centroid_config.max_pixels as f32;
            fixed_label(ui, format!("Max px {}", self.centroid_config.max_pixels));
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_log_bare(ui, &mut v, 10.0..=100000.0);
            });
            self.centroid_config.max_pixels = v as usize;
        });

        // Row 2
        ui.horizontal(|ui| {
            let mut v = self.centroid_config.max_centroids.unwrap_or(0) as f32;
            let lbl = if self.centroid_config.max_centroids.is_none() { "Stars all".into() } else { format!("Stars {}", v.round() as usize) };
            fixed_label(ui, lbl);
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_bare(ui, &mut v, 0.0..=500.0);
            });
            self.centroid_config.max_centroids = if (v as usize) == 0 { None } else { Some(v.round() as usize) };

            let mut v = self.centroid_config.local_bg_block_size.unwrap_or(0) as f32;
            let lbl = if self.centroid_config.local_bg_block_size.is_none() { "BG global".into() } else { format!("BG {}", v.round() as u32) };
            fixed_label(ui, lbl);
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_bare(ui, &mut v, 0.0..=256.0);
            });
            self.centroid_config.local_bg_block_size = if (v as u32) == 0 { None } else { Some(v.round() as u32) };

            let mut v = self.centroid_config.max_elongation.unwrap_or(0.0);
            let lbl = if self.centroid_config.max_elongation.is_none() { "Elong. off".into() } else { format!("Elong. {:.1}", v) };
            fixed_label(ui, lbl);
            ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                widgets::styled_slider_bare(ui, &mut v, 0.0..=10.0);
            });
            self.centroid_config.max_elongation = if v < 0.5 { None } else { Some(v) };
        });

        // Save config if any slider was interacted with this frame
        // ── Solve results ───────────────────────────────────────────────────
        if let Some(ref result) = self.last_solve {
            ui.add_space(2.0);
            ui.separator();
            ui.add_space(2.0);
            if result.status == tetra3::SolveStatus::MatchFound {
                ui.horizontal_wrapped(|ui| {
                    let mono = egui::FontId::monospace(12.0);
                    let dim = widgets::TEXT_SECONDARY;
                    let sp = 10.0;
                    if let Some(crval) = result.crval_rad {
                        ui.label(egui::RichText::new("RA").color(dim));
                        ui.label(egui::RichText::new(format!("{:.4}°", crval[0].to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Dec").color(dim));
                        ui.label(egui::RichText::new(format!("{:.4}°", crval[1].to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                    }
                    if let Some(fov) = result.fov_rad {
                        ui.label(egui::RichText::new("FOV").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}°", fov.to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        let ifov = fov.to_degrees() as f64 / result.image_width.max(1) as f64 * 3600.0;
                        ui.label(egui::RichText::new("IFOV").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}\"/px", ifov)).font(mono.clone()));
                        ui.add_space(sp);
                    }
                    if let Some(theta) = result.theta_rad {
                        ui.label(egui::RichText::new("Rot").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}°", theta.to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                    }
                    if let Some(n) = result.num_matches {
                        ui.label(egui::RichText::new("Stars").color(dim));
                        ui.label(egui::RichText::new(format!("{}", n)).font(mono.clone()));
                        ui.add_space(sp);
                    }
                    if let Some(rmse) = result.rmse_rad {
                        ui.label(egui::RichText::new("RMSE").color(dim));
                        ui.label(egui::RichText::new(format!("{:.1}\"", rmse.to_degrees() * 3600.0)).font(mono.clone()));
                        ui.add_space(sp);
                    }
                    ui.label(egui::RichText::new("Solve").color(dim));
                    ui.label(egui::RichText::new(format!("{:.0}ms", result.solve_time_ms)).font(mono));
                });
            } else {
                let msg = match result.status {
                    tetra3::SolveStatus::NoMatch => "No match",
                    tetra3::SolveStatus::Timeout => "Timed out",
                    tetra3::SolveStatus::TooFew => "Too few stars",
                    _ => "",
                };
                ui.label(egui::RichText::new(msg).color(egui::Color32::from_rgb(220, 38, 38)));
            }
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

#[cfg(any(feature = "svbony", feature = "starsolve"))]
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
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        #[cfg(feature = "starsolve")]
        self.save_config();
    }

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
                        if btn.clicked() { self.stop_recording(); }
                        let t = ui.input(|i| i.time);
                        let alpha = ((t * 3.0).sin() * 0.5 + 0.5) as u8 * 200 + 55;
                        ui.painter().circle_filled(
                            egui::pos2(btn.rect.min.x + 8.0, btn.rect.center().y),
                            4.0, egui::Color32::from_rgba_unmultiplied(220, 40, 40, alpha),
                        );
                    } else if ui.button(egui::RichText::new("Record").size(14.0)).clicked() {
                        self.start_recording();
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

        // Status bar
        egui::TopBottomPanel::bottom("status_bar")
            .exact_height(22.0)
            .frame(egui::Frame::new()
                .fill(egui::Color32::from_rgb(237, 238, 242))
                .inner_margin(egui::Margin { left: 10, right: 10, top: 2, bottom: 2 })
                .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(218, 220, 224)))
            )
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    if self.recording {
                        ui.label(egui::RichText::new(format!(
                            "Recording: {} | {} frames",
                            self.rec_filename, self.rec_frame_count
                        )).size(11.0).color(egui::Color32::from_rgb(220, 40, 40)).monospace());
                    } else {
                        ui.label(egui::RichText::new("Ready").size(11.0)
                            .color(egui::Color32::from_rgb(107, 114, 128)).monospace());
                    }
                });
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
                            #[cfg(feature = "starsolve")]
                            BottomTab::PlateSolve => self.plate_solve_content(ui),
                            BottomTab::Log => self.log_content(ui),
                        }
                    });
            });

        // Central panel
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(frame) = &self.current_frame {
                let resp = self.image_viewer.show(ui, &frame.mono, frame.width, frame.height, &self.display_params, &self.colormap, &self.overlay_items);
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
    /// Auto-solve: dispatch to background thread if DB loaded and not already solving.
    /// Called from poll_frame on each new frame.
    #[cfg(feature = "starsolve")]
    fn maybe_auto_solve(&mut self, frame: &FrameData) {
        // Only if DB loaded and not already solving
        if self.solver_db.is_none() || self.solve_rx.is_some() {
            return;
        }

        let db = self.solver_db.as_ref().unwrap().clone();
        let pixels: Vec<f32> = frame.mono.clone();
        let w = frame.width;
        let h = frame.height;
        let config = self.centroid_config.clone();

        // Use previous solve's FOV if available, otherwise user estimate
        let fov_rad = self.last_solve.as_ref()
            .and_then(|s| s.fov_rad)
            .unwrap_or_else(|| (self.fov_estimate_deg as f32).to_radians());

        // Wide FOV tolerance for initial solve, tight once we have a lock
        let fov_max_error = if self.last_solve.as_ref().map_or(true, |s| s.status != tetra3::SolveStatus::MatchFound) {
            Some((10.0_f32).to_radians()) // wide search initially
        } else {
            Some((2.0_f32).to_radians()) // tight once locked
        };

        let (tx, rx) = bounded(1);
        self.solve_rx = Some(rx);

        std::thread::spawn(move || {
            let centroids = match tetra3::extract_centroids_from_raw(&pixels, w, h, &config) {
                Ok(r) => r.centroids,
                Err(_) => return,
            };

            let mut solve_config = tetra3::SolveConfig::new(fov_rad, w, h);
            solve_config.fov_max_error_rad = fov_max_error;
            solve_config.solve_timeout_ms = Some(2000); // fast timeout for live use
            let result = db.solve_from_centroids(&centroids, &solve_config);
            let _ = tx.send(result);
        });
    }



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
        let asinh_alpha = self.display_params.gamma;
        let asinh_norm: f32 = if matches!(self.display_params.transfer, imageview::TransferFn::Asinh) {
            let v = asinh_alpha.asinh();
            if v > 0.0 { 1.0 / v } else { 1.0 }
        } else { 1.0 };

        for ry in 0..roi_h {
            for rx in 0..roi_w {
                let src_idx = ((y0 as usize + ry) * frame.width as usize) + (x0 as usize + rx);
                let val = if src_idx < frame.mono.len() { frame.mono[src_idx] } else { 0.0 };
                let mut t = ((val - self.display_params.scale_min) * inv_range).clamp(0.0, 1.0);
                match self.display_params.transfer {
                    imageview::TransferFn::Linear => { if apply_gamma { t = t.powf(inv_gamma); } }
                    imageview::TransferFn::Asinh => { t = ((asinh_alpha * t).asinh() * asinh_norm).clamp(0.0, 1.0); }
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
        let overlay_items = self.overlay_items.clone();
        let img_w = frame.width as f32;
        let img_h = frame.height as f32;
        let img_cx = img_w / 2.0;
        let img_cy = img_h / 2.0;

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
                    let resp = ui.add(image);
                    let img_rect = resp.rect;

                    // Draw overlays in zoom window coordinate space
                    let scale_x = w / roi_w as f32;
                    let scale_y = h / roi_h as f32;

                    let to_screen = |ox: f32, oy: f32| -> egui::Pos2 {
                        // ox, oy are image-center origin
                        let px = ox + img_cx - x0 as f32;
                        let py = oy + img_cy - y0 as f32;
                        egui::Pos2::new(
                            img_rect.min.x + px * scale_x,
                            img_rect.min.y + py * scale_y,
                        )
                    };

                    let max_mass = overlay_items.iter().filter_map(|item| {
                        if let overlays::OverlayItem::Centroid { mass, .. } = item { Some(*mass) } else { None }
                    }).fold(0.0_f32, f32::max);

                    overlays::draw_overlays(ui.painter(), &overlay_items, to_screen, scale_x, max_mass, 2.0);

                    // Pixel info on hover
                    if let Some(pos) = resp.hover_pos() {
                        let rx = (pos.x - img_rect.min.x) / scale_x;
                        let ry = (pos.y - img_rect.min.y) / scale_y;
                        let px = (x0 as f32 + rx) as u32;
                        let py = (y0 as f32 + ry) as u32;
                        if px < img_w as u32 && py < img_h as u32 {
                            let idx = (py * img_w as u32 + px) as usize;
                            if let Some(&val) = frame.mono.get(idx) {
                                self.cursor_pixel = Some((px, py));
                                self.cursor_value = Some(val);
                            }
                        }
                    }
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
    let mono: Vec<f32> = match &img {
        DynamicImage::ImageLuma8(g) => g.as_raw().iter().map(|&v| v as f32).collect(),
        DynamicImage::ImageLuma16(g) => g.as_raw().iter().map(|&v| v as f32).collect(),
        DynamicImage::ImageRgb8(rgb) => rgb.pixels().map(|p| 0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32).collect(),
        _ => { let gray = img.to_luma8(); gray.as_raw().iter().map(|&v| v as f32).collect() }
    };
    let hist = compute_histogram(&mono, 256);
    let (mean, stddev) = compute_stats(&mono);
    FrameData { mono, width, height, hist, mean, stddev, bit_depth }
}

fn snap_floor(v: f32, step: f32) -> f32 { (v / step).floor() * step }
fn snap_ceil(v: f32, step: f32) -> f32 { (v / step).ceil() * step }

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
