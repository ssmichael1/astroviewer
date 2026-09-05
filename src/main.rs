// Hide the console window on Windows in release builds (keep it in debug for logs).
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

#[cfg(feature = "toupcam")]
mod bayer;
#[cfg(feature = "starsolve")]
mod bright_stars;
mod colormaps;
mod fits_source;
#[cfg(feature = "focus")]
mod focus;
mod histogram;
mod imageview;
mod overlays;
mod pixels;
mod sources;
mod wcs;
mod widgets;

#[cfg(feature = "svbony")]
mod camera;

#[cfg(feature = "gev")]
mod gev_camera;

#[cfg(feature = "gev")]
mod gige;

#[cfg(feature = "indi")]
mod indi_camera;

#[cfg(feature = "toupcam")]
mod toupcam_camera;

use anyhow::Result;
use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};
use eframe::egui;
#[cfg(any(feature = "svbony", feature = "toupcam"))]
use image::DynamicImage;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::thread;
use std::time::Instant;

use colormaps::{Colormap, ColormapKind};
use histogram::compute_histogram_and_stats;
use imageview::{DisplayParams, ImageViewer};
use pixels::Pixels;
use sources::CameraSource;

// ── Data types ──────────────────────────────────────────────────────────────

struct FrameData {
    /// Shared so the plate-solve worker and the recorder can hold the pixels
    /// without copying them — at full sensor resolution this buffer is ~100 MB.
    /// Mutate via `Arc::make_mut`, which is free while the frame is still
    /// uniquely owned (i.e. before it is handed to either).
    mono: Pixels,
    width: u32,
    height: u32,
    hist: histogram::Histogram,
    /// Per-channel R/G/B histograms of the raw CFA mosaic, present only for
    /// color sensors streaming RAW with the pattern intact. Binned identically
    /// to `hist` so the curves overlay directly.
    channel_hists: Option<[histogram::Histogram; 3]>,
    /// Bayer pattern name ("RGGB", …) when the pixel data itself still carries
    /// an intact mosaic — i.e. a color sensor with no hardware or superpixel
    /// binning applied. Recorded as the FITS BAYERPAT keyword so calibration
    /// software can auto-demosaic.
    cfa: Option<&'static str>,
    mean: f32,
    stddev: f32,
    bit_depth: u8,
}

impl FrameData {
    /// Build a frame from raw mono `f32` pixels, computing the histogram and
    /// stats. Used by float sources (float FITS, computed luma) and by anything
    /// that has been background-subtracted.
    #[cfg(any(feature = "svbony", feature = "toupcam"))] // RGB→luma frames from the USB SDKs
    fn new(mono: Vec<f32>, width: u32, height: u32, bit_depth: u8) -> Self {
        let range_max = ((1u64 << bit_depth) - 1) as f32;
        // Single fused pass over the pixels for histogram + mean + stddev; the
        // range is fixed (0..bit-depth max), not data-derived.
        let (hist, mean, stddev) = compute_histogram_and_stats(&mono, histogram::NUM_BINS, 0.0, range_max);
        FrameData { mono: Pixels::F32(Arc::new(mono)), width, height, hist, channel_hists: None, cfa: None, mean, stddev, bit_depth }
    }

    /// Build a frame from native `u16` pixels — integer camera sources (GigE /
    /// INDI) stay `u16` through histogram, stats and colormap with no widening
    /// copy. The stats run directly on the `u16` slice and are identical to the
    /// f32 path (see `histogram::HistPixel`).
    #[allow(dead_code)] // only feature-gated integer sources build u16 frames
    fn new_u16(mono: Vec<u16>, width: u32, height: u32, bit_depth: u8) -> Self {
        let range_max = ((1u64 << bit_depth) - 1) as f32;
        let (hist, mean, stddev) = compute_histogram_and_stats(&mono, histogram::NUM_BINS, 0.0, range_max);
        FrameData { mono: Pixels::U16(Arc::new(mono)), width, height, hist, channel_hists: None, cfa: None, mean, stddev, bit_depth }
    }

    /// Build a frame around an already-shared pixel buffer. FITS playback hands
    /// out the same `Arc` every loop, so a frame costs one stats pass and no copy.
    fn from_pixels(mono: Pixels, width: u32, height: u32, bit_depth: u8) -> Self {
        let range_max = ((1u64 << bit_depth) - 1) as f32;
        let (hist, mean, stddev) = match &mono {
            Pixels::U16(v) => compute_histogram_and_stats(v.as_slice(), histogram::NUM_BINS, 0.0, range_max),
            Pixels::F32(v) => compute_histogram_and_stats(v.as_slice(), histogram::NUM_BINS, 0.0, range_max),
        };
        FrameData { mono, width, height, hist, channel_hists: None, cfa: None, mean, stddev, bit_depth }
    }
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

/// Which x-range the histogram plot shows. The stored histogram always spans
/// the full range; the plot re-bins whichever window is selected.
#[derive(Clone, Copy, PartialEq, Eq)]
enum HistXRange { Full, Data, Display }

impl HistXRange {
    const ALL: &'static [(HistXRange, &'static str)] = &[
        (HistXRange::Full, "Full range"),
        (HistXRange::Data, "Fit data"),
        (HistXRange::Display, "Display range"),
    ];
}

/// Everything the zoom window's texture derives from; an unchanged key skips
/// the per-repaint ROI recolor + upload while the window is open.
#[derive(Clone, Copy, PartialEq)]
struct ZoomKey {
    frame_serial: u64,
    roi: [u32; 4],
    scale_min: f32,
    scale_max: f32,
    gamma: f32,
    asinh_offset: f32,
    transfer: imageview::TransferFn,
    colormap: ColormapKind,
}

enum CaptureState {
    Fits { _stop_tx: Sender<()> },
    #[cfg(feature = "svbony")]
    SVBony {
        handle: camera::CameraHandle,
        control_values: Vec<(svbony::ControlType, i64, bool)>,
    },
    #[cfg(feature = "gev")]
    Gev {
        handle: gev_camera::GevHandle,
        controls: Vec<gev_camera::GevControl>,
    },
    #[cfg(feature = "toupcam")]
    Toupcam {
        // Boxed: the handle (device info, full model description) and the
        // control mirror are much larger than the other variants.
        handle: Box<toupcam_camera::ToupHandle>,
        controls: Box<toupcam_camera::ToupControls>,
    },
    #[cfg(feature = "indi")]
    Indi {
        handle: indi_camera::IndiHandle,
        /// Latest property snapshot from the reader thread.
        props: Vec<indi_camera::IndiProperty>,
        /// Selected INDI device (a server can host several drivers).
        device: String,
        /// Exposure used by the Single/Live capture buttons, in seconds.
        exposure_s: f64,
        /// Whether the live re-trigger loop is on.
        live: bool,
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
    #[serde(default)]
    camera_model_path: String,
    #[serde(default)]
    matched_filter_sigma: Option<f32>,
    /// Use the single-pass fast extraction path (tracking mode).
    #[serde(default)]
    tracking_mode: bool,
    /// Run the plate-solve pipeline (centroid extraction + solving) on live frames.
    #[serde(default = "default_true")]
    solve_enabled: bool,
}

#[cfg(feature = "starsolve")]
fn default_true() -> bool { true }

/// Which figure the Focus tab's trend plot shows.
#[cfg(feature = "focus")]
#[derive(Clone, Copy, PartialEq, Eq)]
enum FocusPlot {
    Hfr,
    Sharpness,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum BottomTab {
    Histogram,
    Controls,
    #[cfg(feature = "starsolve")]
    PlateSolve,
    #[cfg(feature = "focus")]
    Focus,
    Log,
}

impl BottomTab {
    /// Every tab this build has, in display order.
    fn all() -> Vec<BottomTab> {
        let mut tabs = vec![BottomTab::Histogram, BottomTab::Controls];
        #[cfg(feature = "starsolve")]
        tabs.push(BottomTab::PlateSolve);
        #[cfg(feature = "focus")]
        tabs.push(BottomTab::Focus);
        tabs.push(BottomTab::Log);
        tabs
    }

    fn name(self) -> &'static str {
        match self {
            BottomTab::Histogram => "Histogram",
            BottomTab::Controls => "Controls",
            #[cfg(feature = "starsolve")]
            BottomTab::PlateSolve => "Plate Solve",
            #[cfg(feature = "focus")]
            BottomTab::Focus => "Focus",
            BottomTab::Log => "Log",
        }
    }

    /// Tab by saved name; `None` for a tab this build does not have.
    fn from_name(name: &str) -> Option<BottomTab> {
        Self::all().into_iter().find(|t| t.name() == name)
    }
}

// ── Persisted UI settings ───────────────────────────────────────────────────

/// Display and layout state remembered between runs. Every field is optional
/// so a file from an older or differently featured build still loads; enums
/// travel as their display names, matched back case by case.
#[derive(Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
struct UiConfig {
    theme: Option<String>,
    colormap: Option<String>,
    scale_mode: Option<String>,
    transfer: Option<String>,
    gamma: Option<f32>,
    show_axes: Option<bool>,
    show_colorbar: Option<bool>,
    side_panel_open: Option<bool>,
    side_panel_width: Option<f32>,
    bottom_tab: Option<String>,
    bottom_panel_open: Option<bool>,
    bottom_panel_height: Option<f32>,
    window_size: Option<[f32; 2]>,
    window_pos: Option<[f32; 2]>,
}

impl UiConfig {
    fn path() -> std::path::PathBuf {
        let dir = dirs::config_dir().unwrap_or_else(|| std::path::PathBuf::from(".")).join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        dir.join("ui.json")
    }

    fn load() -> Self {
        std::fs::read_to_string(Self::path())
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default()
    }

    fn save(&self) {
        if let Ok(json) = serde_json::to_string_pretty(self) {
            let _ = std::fs::write(Self::path(), json);
        }
    }
}

// ── Recording ───────────────────────────────────────────────────────────────

enum RecordMsg {
    Frame {
        /// Shared with the live frame rather than copied; see [`FrameData::mono`].
        /// Carries the frame's native type so an integer frame writes as
        /// unsigned-16 with no f32 detour.
        mono: Pixels,
        width: u32,
        height: u32,
        date_obs: String,
        exptime_s: f64,
        /// Bayer pattern of the pixel data, when it carries an intact mosaic.
        cfa: Option<&'static str>,
        /// Camera gain setting, in the camera's native units.
        gain: Option<f64>,
        /// Black level / bias offset setting, ADU.
        offset: Option<f64>,
        /// Sensor temperature, °C.
        ccd_temp: Option<f64>,
        /// Sky mapping from the last plate solve, when one is locked.
        wcs: Option<wcs::WcsKeys>,
        /// Sensor ADC depth, written as BITDEPTH so playback scales the
        /// file the way the live view did.
        bit_depth: u8,
        /// Cooler setpoint, °C (only when the cooler is on).
        set_temp: Option<f64>,
    },
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

/// Result of an async FITS load: the path, the loaded source, and an optional precomputed background.
type FitsLoadResult = Result<(std::path::PathBuf, fits_source::FitsSource), String>;

struct ViewerApp {
    frame_tx: Sender<FrameData>,
    frame_rx: Receiver<FrameData>,
    current_frame: Option<FrameData>,
    /// Return path to the active GigE session's decode-buffer pool. When the UI
    /// replaces `current_frame`, the outgoing frame's `u16` buffer is sent back
    /// here for the capture thread to reuse (only if uniquely owned — a clone
    /// still held by the solver or recorder is just dropped). `None` unless a
    /// GigE session is streaming.
    #[cfg(feature = "gev")]
    frame_pool_return: Option<Sender<Vec<u16>>>,

    display_params: DisplayParams,
    colormap: Colormap,
    scale_mode: ScaleMode,
    image_viewer: ImageViewer,
    zoom_texture: Option<egui::TextureHandle>,
    zoom_rgba: Vec<u8>,
    /// Key the current zoom texture was computed from; an unchanged key skips
    /// the per-repaint ROI recolor + upload.
    zoom_key: Option<ZoomKey>,
    /// Bumped whenever `current_frame` is replaced — the frame-identity half
    /// of the image/zoom recolor cache keys.
    frame_serial: u64,

    ui_theme: widgets::UiTheme,

    cursor_pixel: Option<(u32, u32)>,
    cursor_value: Option<f32>,
    /// Track the asinh pivot automatically (per-frame median ≈ sky background).
    asinh_auto_offset: bool,
    hist_drag: Option<HistDrag>,
    hist_log_y: bool,
    /// Overlay per-channel R/G/B histograms when the frame carries them.
    hist_rgb: bool,
    hist_x_range: HistXRange,

    // Overlay system
    overlay_items: Vec<overlays::OverlayItem>,
    #[cfg(feature = "starsolve")]
    show_centroids: bool,
    #[cfg(feature = "starsolve")]
    show_matched_stars: bool,
    #[cfg(feature = "starsolve")]
    show_star_names: bool,
    #[cfg(feature = "starsolve")]
    centroid_config: tetra3::CentroidExtractionConfig,
    /// Extraction path: false = CCL (calibration-quality, matched filter),
    /// true = single-pass fast path — reads each pixel once, no convolution.
    /// Trades faint-star sensitivity for speed; right for live tracking.
    #[cfg(feature = "starsolve")]
    tracking_mode: bool,
    /// Master switch for the whole plate-solve pipeline. Off: frames are not
    /// sent to the worker at all — no centroid extraction, no solving, no
    /// overlays — so the per-frame CPU cost disappears entirely.
    #[cfg(feature = "starsolve")]
    solve_enabled: bool,
    #[cfg(feature = "starsolve")]
    centroid_count: usize,
    #[cfg(feature = "starsolve")]
    centroid_time_ms: f32,
    #[cfg(feature = "starsolve")]
    solver_db: Option<std::sync::Arc<tetra3::SolverDatabase>>,
    #[cfg(feature = "starsolve")]
    solver_db_path: Option<std::path::PathBuf>,
    /// Bundled Gaia catalog the default database can be generated from, if found.
    #[cfg(feature = "starsolve")]
    solver_catalog_path: Option<std::path::PathBuf>,
    /// Receiver for an in-progress background database generation.
    #[cfg(feature = "starsolve")]
    gen_rx: Option<Receiver<Result<tetra3::SolverDatabase, String>>>,
    #[cfg(feature = "starsolve")]
    gen_started: Option<Instant>,
    /// Whether to show the one-time "build the database now?" prompt.
    #[cfg(feature = "starsolve")]
    show_build_prompt: bool,
    #[cfg(feature = "starsolve")]
    fov_estimate_deg: f32,
    /// Job sender for the long-lived extract+solve worker.
    #[cfg(feature = "starsolve")]
    solve_tx: Sender<SolveJob>,
    #[cfg(feature = "starsolve")]
    solve_rx: Receiver<SolveOutput>,
    /// Whether the worker is mid-job; new frames are skipped rather than queued.
    #[cfg(feature = "starsolve")]
    solve_busy: bool,
    /// Centroids from the worker's last completed frame — the source for the
    /// overlay and the index space `matched_centroid_indices` refers to.
    #[cfg(feature = "starsolve")]
    last_centroids: Vec<tetra3::Centroid>,
    #[cfg(feature = "starsolve")]
    last_solve: Option<tetra3::SolveResult>,
    #[cfg(feature = "starsolve")]
    camera_model: Option<tetra3::CameraModel>,
    #[cfg(feature = "starsolve")]
    camera_model_path: Option<std::path::PathBuf>,

    /// Focus trend: one point per completed worker frame, plus best-so-far.
    #[cfg(feature = "focus")]
    focus_history: focus::FocusHistory,
    /// Per-star results from the worker's last frame, for the overlay labels.
    #[cfg(feature = "focus")]
    focus_last: Option<focus::FocusSample>,
    /// Measure only stars inside the zoom ROI (when one is drawn).
    #[cfg(feature = "focus")]
    focus_use_roi: bool,
    /// Draw each measured star's HFR next to it on the image.
    #[cfg(feature = "focus")]
    focus_show_labels: bool,
    #[cfg(feature = "focus")]
    focus_plot: FocusPlot,

    frame_times: Vec<Instant>,
    fps: f64,

    /// Frames dropped in the pump thread's forward to the UI channel (bumped by
    /// the pump thread; shared so the UI can read it).
    pump_drops: Arc<AtomicU64>,
    /// Frames discarded by the UI's keep-latest drain (superseded before the
    /// UI could use them). Only ever touched on the UI thread.
    ui_drain_drops: u64,

    /// Smoothed GigE receive rate (MB/s), recomputed on the UI thread from the
    /// running byte counter over ~1 s windows; 0 for non-GigE sources.
    #[cfg(feature = "gev")]
    gev_rate_mbps: f64,
    /// Byte-counter value and time at the last rate sample.
    #[cfg(feature = "gev")]
    gev_rate_prev: (u64, Instant),

    camera_source: CameraSource,
    capture_state: CaptureState,
    capture_running: bool,
    recording: bool,
    rec_tx: Option<Sender<RecordMsg>>,
    /// Recording writer thread; joined on stop/exit so the file is always
    /// fully flushed before the process can go away.
    rec_join: Option<thread::JoinHandle<()>>,
    rec_filename: String,
    rec_frame_count: u32,

    fits_fps: Arc<AtomicU32>,

    side_panel_open: bool,
    bottom_tab: BottomTab,
    /// Bottom panel shown; off leaves only the tab strip.
    bottom_panel_open: bool,
    /// Last measured panel sizes and window geometry, persisted at exit.
    side_panel_width: f32,
    bottom_panel_height: f32,
    /// `(inner size, outer position)` in logical points.
    window_geometry: Option<([f32; 2], [f32; 2])>,

    // Log
    log: Vec<LogEntry>,
    /// Entries acknowledged: the Log tab has been shown since they arrived.
    /// Everything past this index that is a warning or error is "unread".
    log_seen: usize,
    log_rx: Receiver<LogEntry>,
    log_tx: Sender<LogEntry>,

    // Background subtraction
    bg_subtract_enabled: bool,
    bg_percentile: f32,
    bg_image: Option<Vec<f32>>,
    /// Percentile `bg_image` was computed at; a mismatch with `bg_percentile`
    /// means a recompute is owed.
    bg_computed_pct: Option<f32>,
    bg_hist_range: Option<(f32, f32)>,
    /// In-flight background estimate: (percentile, image, elapsed).
    pending_bg: Option<Receiver<(f32, Vec<f32>, std::time::Duration)>>,
    /// The loaded FITS file's frames, shared with the playback thread, so the
    /// background can be (re)computed without re-reading the file.
    fits_frames: Option<Arc<fits_source::FitsFrames>>,

    // Async file dialog result
    pending_fits_path: Option<Receiver<Option<std::path::PathBuf>>>,
    // Async FITS loading result (path, source, optional background)
    pending_fits_load: Option<Receiver<FitsLoadResult>>,

    /// Every discovered device across all backends, in registry order — the
    /// unified list behind the Source menu, `source_label`, and `open_source`.
    discovered: Vec<sources::DiscoveredSource>,
    /// Whether the Connect-to-Source window is showing.
    connect_dialog_open: bool,
    /// Text of each backend's address-entry field in the Connect dialog,
    /// keyed by descriptor scheme ("gev" → last typed IP).
    manual_inputs: std::collections::HashMap<&'static str, String>,
    /// Substring filter for the GigE controls panel.
    #[cfg(feature = "gev")]
    gev_filter: String,
    /// Substring filter for the INDI properties panel.
    #[cfg(feature = "indi")]
    indi_filter: String,
    /// Error from the last connect attempt (device open, manual-address
    /// parse), shown in the Connect dialog and the side panel.
    camera_error: Option<String>,
}

/// Name of the semibold font family used for headings, section eyebrows, and
/// emphasized labels. egui's `.strong()` only recolors text — real weight has
/// to come from a heavier font, which we synthesize here.
const FONT_STRONG: &str = "strong";

fn strong_family() -> egui::FontFamily {
    egui::FontFamily::Name(FONT_STRONG.into())
}

/// Install the best available system UI + monospace fonts, plus a semibold UI
/// instance for emphasis. The macOS SF files are *variable* fonts, so the
/// semibold is the same file driven to `wght: 600` via the variation axis;
/// other platforms fall back to a static-bold file, then to regular, then to
/// egui's bundled fonts for glyph coverage.
fn install_fonts(ctx: &egui::Context) {
    use egui::epaint::text::VariationCoords;
    use egui::{FontData, FontFamily};

    // First existing path wins, per role.
    const PROP: &[&str] = &[
        "/System/Library/Fonts/SFNS.ttf",                  // macOS — SF Pro (variable)
        "C:/Windows/Fonts/segoeui.ttf",                    // Windows — Segoe UI
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", // Linux
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
    ];
    const PROP_BOLD: &[&str] = &[ // static-bold fallback for non-variable platforms
        "C:/Windows/Fonts/segoeuib.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
    ];
    const MONO: &[&str] = &[
        "/System/Library/Fonts/SFNSMono.ttf", // macOS — SF Mono
        "C:/Windows/Fonts/consola.ttf",       // Windows — Consolas
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf",
    ];

    let read_first = |paths: &[&str]| {
        paths.iter().find_map(|p| std::fs::read(p).ok().map(|d| (p.to_string(), d)))
    };

    let mut fonts = egui::FontDefinitions::default();

    // Proportional regular + semibold instance.
    if let Some((path, data)) = read_first(PROP) {
        let variable = path.contains("SFNS");

        let mut reg = FontData::from_owned(data.clone());
        reg.tweak.y_offset_factor = -0.02;
        fonts.font_data.insert("ui".to_owned(), reg.into());
        fonts.families.entry(FontFamily::Proportional).or_default().insert(0, "ui".to_owned());

        let mut sb = if variable {
            let mut d = FontData::from_owned(data);
            d.tweak.coords = VariationCoords::new([(b"wght", 600.0)]);
            d
        } else if let Some((_, bold)) = read_first(PROP_BOLD) {
            FontData::from_owned(bold)
        } else {
            FontData::from_owned(data)
        };
        sb.tweak.y_offset_factor = -0.02;
        fonts.font_data.insert("ui_semibold".to_owned(), sb.into());

        // Semibold family, with regular UI + egui defaults as glyph fallback.
        let mut fam = vec!["ui_semibold".to_owned()];
        if let Some(prop) = fonts.families.get(&FontFamily::Proportional) {
            fam.extend(prop.iter().filter(|n| *n != "ui_semibold").cloned());
        }
        fonts.families.insert(strong_family(), fam);
    }

    // Monospace.
    if let Some((path, data)) = read_first(MONO) {
        let mut md = FontData::from_owned(data);
        if path.contains("SFNSMono") {
            md.tweak.scale = 0.95; // SF Mono runs large; match the proportional cap height
        }
        md.tweak.y_offset_factor = -0.02;
        fonts.font_data.insert("ui_mono".to_owned(), md.into());
        fonts.families.entry(FontFamily::Monospace).or_default().insert(0, "ui_mono".to_owned());
    }

    // Guarantee the strong family resolves even if no system UI font loaded.
    if !fonts.families.contains_key(&strong_family()) {
        let fallback = fonts.families.get(&FontFamily::Proportional).cloned().unwrap_or_default();
        fonts.families.insert(strong_family(), fallback);
    }

    ctx.set_fonts(fonts);
}

impl ViewerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Load system UI + monospace fonts (with a real semibold for emphasis).
        install_fonts(&cc.egui_ctx);

        // A terminal Ctrl-C (or SIGTERM from a supervisor) becomes a normal
        // window close, so `on_exit` runs: recording flushed, camera stopped
        // and released, settings saved. A second signal while that is in
        // progress exits immediately, in case a camera teardown hangs.
        {
            let ctx = cc.egui_ctx.clone();
            let result = ctrlc::set_handler(move || {
                if QUIT_REQUESTED.swap(true, Ordering::SeqCst) {
                    std::process::exit(130);
                }
                ctx.request_repaint();
            });
            if let Err(e) = result {
                tracing::warn!("signal handler not installed: {e}");
            }
        }

        // Theme
        let mut style = (*cc.egui_ctx.global_style()).clone();
        style.text_styles.insert(egui::TextStyle::Body, egui::FontId::new(13.0, egui::FontFamily::Proportional));
        style.text_styles.insert(egui::TextStyle::Heading, egui::FontId::new(15.5, strong_family()));
        style.text_styles.insert(egui::TextStyle::Button, egui::FontId::new(13.0, egui::FontFamily::Proportional));
        style.text_styles.insert(egui::TextStyle::Monospace, egui::FontId::new(12.5, egui::FontFamily::Monospace));
        style.text_styles.insert(egui::TextStyle::Small, egui::FontId::new(11.0, egui::FontFamily::Proportional));

        style.spacing.item_spacing = egui::vec2(6.0, 4.0);
        style.spacing.button_padding = egui::vec2(12.0, 6.0);
        style.spacing.slider_width = 140.0;
        style.spacing.icon_width = 16.0;
        style.spacing.icon_spacing = 6.0;
        style.spacing.combo_width = 110.0;

        // Theme colors are applied each frame by apply_theme()
        cc.egui_ctx.set_global_style(style);

        // Producers send into a pump thread that forwards to the UI channel
        // and wakes egui, so a frame repaints the moment it lands instead of
        // waiting on a polled tick.
        let (frame_tx, pump_rx) = bounded::<FrameData>(2);
        let (pump_tx, frame_rx) = bounded::<FrameData>(2);
        let pump_ctx = cc.egui_ctx.clone();
        let pump_drops = Arc::new(AtomicU64::new(0));
        let pump_drops_thread = Arc::clone(&pump_drops);
        thread::spawn(move || {
            // Coalesce repaint requests to ~display rate: at 100+ fps a repaint
            // per frame renders far more often than the screen refreshes and
            // wastes CPU/GPU. We still repaint within one 16 ms window of a
            // frame landing (the drain keeps only the latest), so the newest
            // frame is on screen with imperceptible latency.
            const REPAINT_INTERVAL: std::time::Duration = std::time::Duration::from_millis(16);
            let mut last_repaint = Instant::now() - REPAINT_INTERVAL;
            while let Ok(frame) = pump_rx.recv() {
                // Same drop-when-full semantics producers had sending directly.
                if pump_tx.try_send(frame).is_err() {
                    pump_drops_thread.fetch_add(1, Ordering::Relaxed);
                }
                let since = last_repaint.elapsed();
                if since >= REPAINT_INTERVAL {
                    pump_ctx.request_repaint();
                    last_repaint = Instant::now();
                } else {
                    // Guarantee a repaint by the end of the current window.
                    pump_ctx.request_repaint_after(REPAINT_INTERVAL - since);
                }
            }
        });
        let (log_tx, log_rx) = bounded(64);

        let log = vec![LogEntry::info("Viewer started".to_string())];

        let discovered = sources::discover_all();

        // Each addressed backend's connect field starts at its registry
        // default ("localhost" for INDI, empty for GigE).
        #[allow(unused_mut)]
        let mut manual_inputs: std::collections::HashMap<&'static str, String> = sources::backends()
            .iter()
            .filter_map(|b| b.manual.as_ref().map(|m| (b.scheme, m.default.to_string())))
            .collect();
        // Pre-fill the manual GigE connect field with the last IP we connected
        // to, so a reachable-by-unicast camera is one click away on relaunch.
        #[cfg(feature = "gev")]
        if let Ok(ip) = std::fs::read_to_string(Self::gev_last_ip_path()) {
            let ip = ip.trim().to_string();
            if !ip.is_empty() {
                manual_inputs.insert("gev", ip);
            }
        }

        // The app is built idle; the startup source (CLI argument or the
        // remembered last source) is opened after construction, below.
        let camera_source = CameraSource::None;
        let capture_state = CaptureState::Stopped;
        let capture_running = false;

        let ui_theme = widgets::UiTheme::Dark;

        #[cfg(feature = "starsolve")]
        let (solve_tx, solve_rx) = spawn_solve_worker();

        #[allow(unused_mut)]
        let mut app = Self {
            frame_tx, frame_rx,
            current_frame: None,
            #[cfg(feature = "gev")]
            frame_pool_return: None,
            display_params: DisplayParams { scale_min: 0.0, scale_max: 4095.0, ..Default::default() },
            colormap: Colormap::new(ColormapKind::Grayscale),
            scale_mode: ScaleMode::Auto,
            image_viewer: ImageViewer::new(),
            zoom_texture: None,
            zoom_rgba: Vec::new(),
            zoom_key: None,
            frame_serial: 0,
            ui_theme,
            cursor_pixel: None, cursor_value: None,
            asinh_auto_offset: true,
            hist_drag: None,
            hist_log_y: false,
            hist_rgb: true,
            hist_x_range: HistXRange::Full,
            overlay_items: Vec::new(),
            #[cfg(feature = "starsolve")]
            show_centroids: false,
            #[cfg(feature = "starsolve")]
            show_matched_stars: true,
            #[cfg(feature = "starsolve")]
            show_star_names: true,
            #[cfg(feature = "starsolve")]
            centroid_config: tetra3::CentroidExtractionConfig::default(),
            #[cfg(feature = "starsolve")]
            tracking_mode: false,
            #[cfg(feature = "starsolve")]
            solve_enabled: true,
            #[cfg(feature = "starsolve")]
            centroid_count: 0,
            #[cfg(feature = "starsolve")]
            centroid_time_ms: 0.0,
            #[cfg(feature = "starsolve")]
            solver_db: None,
            #[cfg(feature = "starsolve")]
            solver_db_path: None,
            #[cfg(feature = "starsolve")]
            solver_catalog_path: None,
            #[cfg(feature = "starsolve")]
            gen_rx: None,
            #[cfg(feature = "starsolve")]
            gen_started: None,
            #[cfg(feature = "starsolve")]
            show_build_prompt: false,
            #[cfg(feature = "starsolve")]
            fov_estimate_deg: 15.0,
            #[cfg(feature = "starsolve")]
            solve_tx,
            #[cfg(feature = "starsolve")]
            solve_rx,
            #[cfg(feature = "starsolve")]
            solve_busy: false,
            #[cfg(feature = "starsolve")]
            last_centroids: Vec::new(),
            #[cfg(feature = "starsolve")]
            last_solve: None,
            #[cfg(feature = "focus")]
            focus_history: focus::FocusHistory::new(400),
            #[cfg(feature = "focus")]
            focus_last: None,
            #[cfg(feature = "focus")]
            focus_use_roi: false,
            #[cfg(feature = "focus")]
            focus_show_labels: false,
            #[cfg(feature = "focus")]
            focus_plot: FocusPlot::Hfr,
            #[cfg(feature = "starsolve")]
            camera_model: None,
            #[cfg(feature = "starsolve")]
            camera_model_path: None,
            frame_times: Vec::new(), fps: 0.0,
            pump_drops,
            ui_drain_drops: 0,
            #[cfg(feature = "gev")]
            gev_rate_mbps: 0.0,
            #[cfg(feature = "gev")]
            gev_rate_prev: (0, Instant::now()),
            camera_source, capture_state, capture_running,
            recording: false,
            rec_tx: None,
            rec_join: None,
            rec_filename: String::new(),
            rec_frame_count: 0,
            fits_fps: Arc::new(AtomicU32::new(10)),
            side_panel_open: true,
            bottom_tab: BottomTab::Histogram,
            bottom_panel_open: true,
            side_panel_width: 220.0,
            bottom_panel_height: 300.0,
            window_geometry: None,
            log, log_rx, log_tx,
            log_seen: 0,
            bg_subtract_enabled: false,
            bg_percentile: 0.35,
            bg_image: None,
            bg_computed_pct: None,
            bg_hist_range: None,
            pending_bg: None,
            fits_frames: None,
            pending_fits_path: None,
            pending_fits_load: None,
            discovered,
            connect_dialog_open: false,
            manual_inputs,
            #[cfg(feature = "gev")]
            gev_filter: String::new(),
            #[cfg(feature = "indi")]
            indi_filter: String::new(),
            camera_error: None,
        };

        #[cfg(feature = "starsolve")]
        app.load_config();
        app.apply_ui_config(&UiConfig::load());

        // Startup source precedence: an explicit CLI descriptor (or bare FITS
        // path) wins; otherwise reconnect to the last source used; otherwise
        // stay idle and let the user pick from the Source menu.
        if let Some(arg) = std::env::args().nth(1) {
            match CameraSource::parse_descriptor(&arg) {
                Ok(src) => app.open_source(src),
                Err(e) => app.add_log(LogEntry::error(format!("Command-line source: {}", e))),
            }
        } else if let Ok(desc) = std::fs::read_to_string(Self::last_source_path()) {
            let desc = desc.trim();
            if !desc.is_empty() {
                match CameraSource::parse_descriptor(desc) {
                    Ok(src) => {
                        app.add_log(LogEntry::info(format!("Reconnecting to last source: {}", desc)));
                        app.open_source(src);
                    }
                    Err(e) => app.add_log(LogEntry::error(format!("Remembered source: {}", e))),
                }
            }
        }

        app
    }

    /// Restore remembered display and layout settings. Window geometry is
    /// applied in `main` before the window exists; here only the in-app state.
    fn apply_ui_config(&mut self, cfg: &UiConfig) {
        if let Some(t) = cfg.theme.as_deref().and_then(|n| widgets::UiTheme::ALL.iter().find(|(_, l)| *l == n)) {
            self.ui_theme = t.0;
        }
        if let Some(k) = cfg.colormap.as_deref().and_then(|n| ColormapKind::ALL.iter().find(|k| k.name() == n)) {
            self.colormap = Colormap::new(*k);
        }
        if let Some(m) = cfg.scale_mode.as_deref().and_then(|n| ScaleMode::ALL.iter().find(|(_, l)| *l == n)) {
            self.scale_mode = m.0;
        }
        if let Some(t) = cfg.transfer.as_deref().and_then(|n| imageview::TransferFn::ALL.iter().find(|(_, l)| *l == n)) {
            self.display_params.transfer = t.0;
        }
        if let Some(g) = cfg.gamma.filter(|g| g.is_finite() && *g > 0.0) {
            self.display_params.gamma = g;
        }
        if let Some(v) = cfg.show_axes { self.display_params.show_axes = v; }
        if let Some(v) = cfg.show_colorbar { self.display_params.show_colorbar = v; }
        if let Some(v) = cfg.side_panel_open { self.side_panel_open = v; }
        if let Some(w) = cfg.side_panel_width.filter(|w| (120.0..=800.0).contains(w)) {
            self.side_panel_width = w;
        }
        if let Some(t) = cfg.bottom_tab.as_deref().and_then(BottomTab::from_name) {
            self.bottom_tab = t;
        }
        if let Some(v) = cfg.bottom_panel_open { self.bottom_panel_open = v; }
        if let Some(h) = cfg.bottom_panel_height.filter(|h| (120.0..=900.0).contains(h)) {
            self.bottom_panel_height = h;
        }
    }

    fn ui_config(&self) -> UiConfig {
        let label = |theme: widgets::UiTheme| widgets::UiTheme::ALL.iter().find(|(t, _)| *t == theme).map(|(_, l)| l.to_string());
        UiConfig {
            theme: label(self.ui_theme),
            colormap: Some(self.colormap.kind.name().to_string()),
            scale_mode: ScaleMode::ALL.iter().find(|(m, _)| *m == self.scale_mode).map(|(_, l)| l.to_string()),
            transfer: imageview::TransferFn::ALL.iter().find(|(t, _)| *t == self.display_params.transfer).map(|(_, l)| l.to_string()),
            gamma: Some(self.display_params.gamma),
            show_axes: Some(self.display_params.show_axes),
            show_colorbar: Some(self.display_params.show_colorbar),
            side_panel_open: Some(self.side_panel_open),
            side_panel_width: Some(self.side_panel_width),
            bottom_tab: Some(self.bottom_tab.name().to_string()),
            bottom_panel_open: Some(self.bottom_panel_open),
            bottom_panel_height: Some(self.bottom_panel_height),
            window_size: self.window_geometry.map(|(size, _)| size),
            window_pos: self.window_geometry.map(|(_, pos)| pos),
        }
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
            camera_model_path: self.camera_model_path.as_ref().map(|p| p.display().to_string()).unwrap_or_default(),
            matched_filter_sigma: self.centroid_config.matched_filter_sigma,
            tracking_mode: self.tracking_mode,
            solve_enabled: self.solve_enabled,
        };
        if let Ok(json) = serde_json::to_string_pretty(&cfg) {
            let _ = std::fs::write(Self::config_path(), json);
        }
    }

    /// Gaia catalog bundled with packaged builds; the default database is
    /// generated from it on first run.
    #[cfg(feature = "starsolve")]
    const GAIA_CATALOG_FILENAME: &'static str = "gaia_merged.bin";

    /// Cached generated database filename. Encodes the generation parameters so
    /// that changing them produces a new file (old one is simply ignored).
    #[cfg(feature = "starsolve")]
    const GENERATED_SOLVER_FILENAME: &'static str = "solver_fov5-50_mag85.bin";

    /// Parameters used to generate the default database from the Gaia catalog.
    #[cfg(feature = "starsolve")]
    fn solver_gen_config() -> tetra3::GenerateDatabaseConfig {
        tetra3::GenerateDatabaseConfig {
            min_fov_deg: Some(5.0),
            max_fov_deg: 50.0,
            star_max_magnitude: Some(8.5),
            epoch_proper_motion_year: Some(2026.0),
            verification_stars_per_fov: 250,
            patterns_per_lattice_field: 100,
            ..Default::default()
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
                self.centroid_config.matched_filter_sigma = cfg.matched_filter_sigma;
                self.tracking_mode = cfg.tracking_mode;
                self.solve_enabled = cfg.solve_enabled;

                if !cfg.solver_db_path.is_empty() && std::path::Path::new(&cfg.solver_db_path).exists() {
                    self.load_solver_db(std::path::Path::new(&cfg.solver_db_path));
                }

                if !cfg.camera_model_path.is_empty() && std::path::Path::new(&cfg.camera_model_path).exists() {
                    match tetra3::CameraModel::load_from_file(&cfg.camera_model_path) {
                        Ok(cam) => {
                            self.add_log(LogEntry::info(format!(
                                "Camera model loaded: f={:.1}px, {}x{}, FOV {:.2}°",
                                cam.focal_length_px, cam.image_width, cam.image_height, cam.fov_deg(),
                            )));
                            self.fov_estimate_deg = cam.fov_deg() as f32;
                            self.camera_model = Some(cam);
                            self.camera_model_path = Some(std::path::PathBuf::from(&cfg.camera_model_path));
                        }
                        Err(e) => {
                            self.add_log(LogEntry::error(format!("Camera model load failed: {}", e)));
                        }
                    }
                }
            }
        }

        // Provision a plate-solver database when the saved config didn't load
        // one (first run, or a missing/relocated file). Prefer a previously
        // generated cache; otherwise offer to build it from the bundled Gaia
        // catalog. This keeps plate solving working fully offline.
        if self.solver_db.is_none() {
            let cache = Self::generated_solver_path();
            if cache.exists() {
                self.load_solver_db(&cache);
            } else if let Some(catalog) = Self::default_catalog_path() {
                self.solver_catalog_path = Some(catalog);
                self.show_build_prompt = true;
            }
        }
    }

    /// Loads a solver database from `path`, logging the result and updating state.
    #[cfg(feature = "starsolve")]
    fn load_solver_db(&mut self, path: &std::path::Path) {
        self.add_log(LogEntry::info(format!("Auto-loading database: {}", path.display())));
        match tetra3::SolverDatabase::load_from_file(path.to_str().unwrap_or("")) {
            Ok(db) => {
                self.add_log(LogEntry::info(format!(
                    "Database loaded: {} patterns, {} stars",
                    db.props.num_patterns,
                    db.star_vectors.len(),
                )));
                self.solver_db_path = Some(path.to_path_buf());
                self.solver_db = Some(std::sync::Arc::new(db));
            }
            Err(e) => self.add_log(LogEntry::error(format!("Auto-load failed: {}", e))),
        }
    }

    /// Locates the Gaia catalog bundled alongside the executable, if present.
    /// Checks next to the binary (Windows/Linux packages) and the macOS app
    /// bundle's `Resources` directory.
    #[cfg(feature = "starsolve")]
    fn default_catalog_path() -> Option<std::path::PathBuf> {
        let exe = std::env::current_exe().ok()?;
        let dir = exe.parent()?;
        [
            dir.join(Self::GAIA_CATALOG_FILENAME),
            dir.join("../Resources").join(Self::GAIA_CATALOG_FILENAME),
        ]
        .into_iter()
        .find(|p| p.exists())
    }

    /// Path of the generated-and-cached default database in the app data dir.
    #[cfg(feature = "starsolve")]
    fn generated_solver_path() -> std::path::PathBuf {
        let dir = dirs::data_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        dir.join(Self::GENERATED_SOLVER_FILENAME)
    }

    /// Kicks off background generation of the default database from the bundled
    /// catalog. Result is polled by [`Self::poll_solver_generation`].
    #[cfg(feature = "starsolve")]
    fn start_solver_generation(&mut self) {
        if self.gen_rx.is_some() {
            return; // already running
        }
        let Some(catalog) = self.solver_catalog_path.clone() else {
            return;
        };
        self.show_build_prompt = false;
        let cache = Self::generated_solver_path();
        let (tx, rx) = bounded(1);
        self.gen_rx = Some(rx);
        self.gen_started = Some(Instant::now());
        self.add_log(LogEntry::info("Building star database from catalog…".to_string()));
        thread::spawn(move || {
            let config = Self::solver_gen_config();
            let result = tetra3::SolverDatabase::generate_from_gaia(
                catalog.to_str().unwrap_or(""),
                &config,
            )
            .map_err(|e| e.to_string());
            // Cache to disk so subsequent launches load instantly.
            if let Ok(db) = &result {
                let _ = db.save_to_file(cache.to_str().unwrap_or(""));
            }
            let _ = tx.send(result);
        });
    }

    /// Polls the background generation thread and installs the database on
    /// completion. No-op when no generation is in flight.
    #[cfg(feature = "starsolve")]
    fn poll_solver_generation(&mut self) {
        let Some(rx) = self.gen_rx.as_ref() else {
            return;
        };
        match rx.try_recv() {
            Ok(result) => {
                self.gen_rx = None;
                let elapsed = self.gen_started.take().map_or(0.0, |t| t.elapsed().as_secs_f32());
                match result {
                    Ok(db) => {
                        self.add_log(LogEntry::info(format!(
                            "Star database built in {:.1}s: {} patterns, {} stars",
                            elapsed,
                            db.props.num_patterns,
                            db.star_vectors.len(),
                        )));
                        self.solver_db_path = Some(Self::generated_solver_path());
                        self.solver_db = Some(std::sync::Arc::new(db));
                    }
                    Err(e) => self.add_log(LogEntry::error(format!("Database build failed: {}", e))),
                }
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => {
                // Worker died without sending (shouldn't happen) — reset state.
                self.gen_rx = None;
                self.gen_started = None;
                self.add_log(LogEntry::error("Database build thread stopped unexpectedly".to_string()));
            }
        }
    }

    fn add_log(&mut self, entry: LogEntry) {
        self.log.push(entry);
        if self.log.len() > 500 {
            self.log.remove(0);
            self.log_seen = self.log_seen.saturating_sub(1);
        }
    }

    /// Warnings and errors that arrived since the Log tab was last shown.
    fn unread_log(&self) -> impl Iterator<Item = &LogEntry> {
        self.log[self.log_seen.min(self.log.len())..]
            .iter()
            .filter(|e| matches!(e.level, LogLevel::Error | LogLevel::Warn))
    }

    fn pal(&self) -> widgets::Palette {
        self.ui_theme.palette()
    }

    /// Total frames dropped in the app's own software pipeline since capture
    /// started (independent of any network loss): producer→UI channel-full
    /// drops plus the UI's keep-latest discards, and for the GigE backend the
    /// receive→control handoff drops and the frames whose decode was skipped
    /// because the UI channel was already full.
    fn dropped_total(&self) -> u64 {
        #[allow(unused_mut)]
        let mut total = self.pump_drops.load(Ordering::Relaxed) + self.ui_drain_drops;
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { ref handle, .. } = self.capture_state {
            let s = &handle.drop_stats;
            total += s.rx_to_control.load(Ordering::Relaxed)
                + s.control_to_ui.load(Ordering::Relaxed)
                + s.decode_skipped.load(Ordering::Relaxed);
        }
        total
    }

    /// Recompute the smoothed GigE receive rate (MB/s) from the running byte
    /// counter over ~1 s windows. Resets to 0 when no GigE camera is streaming.
    #[cfg(feature = "gev")]
    fn update_gev_rate(&mut self) {
        if let CaptureState::Gev { ref handle, .. } = self.capture_state {
            let bytes = handle.received_bytes.load(Ordering::Relaxed);
            let (prev_bytes, prev_at) = self.gev_rate_prev;
            let dt = prev_at.elapsed().as_secs_f64();
            if dt >= 1.0 {
                let delta = bytes.saturating_sub(prev_bytes) as f64;
                self.gev_rate_mbps = delta / dt / 1.0e6;
                self.gev_rate_prev = (bytes, Instant::now());
            }
        } else if self.gev_rate_mbps != 0.0 {
            self.gev_rate_mbps = 0.0;
            self.gev_rate_prev = (0, Instant::now());
        }
    }

    /// Human-readable label for the current image source.
    fn source_label(&self) -> String {
        match &self.camera_source {
            CameraSource::None => "No source".to_string(),
            CameraSource::FitsFile(path) => {
                format!("FITS: {}", path.file_name().unwrap_or_default().to_string_lossy())
            }
            #[cfg(feature = "indi")]
            CameraSource::Indi(host) => format!("INDI: {}", host),
            // Discovered-device backends: name from the unified list, falling
            // back to the descriptor for a device that is no longer visible.
            #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam"))]
            other => self
                .discovered
                .iter()
                .find(|d| d.source == *other)
                .map(|d| d.name.clone())
                .unwrap_or_else(|| other.descriptor().unwrap_or_default()),
        }
    }

    /// Open the native FITS file picker on a worker thread. No-op if one is
    /// already pending; the chosen path is consumed by `poll_fits_load`.
    fn open_fits_dialog(&mut self) {
        if self.pending_fits_path.is_some() {
            return;
        }
        let (tx, rx) = bounded(1);
        self.pending_fits_path = Some(rx);
        std::thread::spawn(move || {
            let result = rfd::FileDialog::new()
                .add_filter("FITS", &["fits", "fit", "fts"])
                .pick_file();
            let _ = tx.send(result);
        });
    }

    fn apply_theme(&self, ctx: &egui::Context) {
        let pal = self.pal();
        let mut style = (*ctx.global_style()).clone();

        let r = egui::CornerRadius::same(6);
        style.visuals.dark_mode = self.ui_theme.is_dark();
        style.visuals.widgets.noninteractive.corner_radius = r;
        style.visuals.widgets.inactive.corner_radius = r;
        style.visuals.widgets.hovered.corner_radius = r;
        style.visuals.widgets.active.corner_radius = r;
        style.visuals.panel_fill = pal.panel_fill;
        style.visuals.window_fill = pal.window_fill;
        // Text input / DragValue backgrounds
        style.visuals.extreme_bg_color = pal.extreme_bg;
        style.visuals.faint_bg_color = pal.faint_bg;
        style.visuals.widgets.noninteractive.bg_fill = pal.panel_fill;
        style.visuals.widgets.noninteractive.weak_bg_fill = pal.panel_fill;
        style.visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0, pal.section_border);
        style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, pal.text_primary);
        style.visuals.widgets.inactive.bg_fill = pal.bg_raised;
        style.visuals.widgets.inactive.weak_bg_fill = pal.bg_raised; // native button background
        style.visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0, pal.border);
        style.visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.5, pal.text_secondary);
        style.visuals.widgets.hovered.bg_fill = pal.bg_hover;
        style.visuals.widgets.hovered.weak_bg_fill = pal.bg_hover;
        style.visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.5, pal.accent_light);
        style.visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.5, pal.accent);
        style.visuals.widgets.active.bg_fill = pal.bg_hover;
        style.visuals.widgets.active.weak_bg_fill = pal.bg_hover;
        style.visuals.widgets.active.bg_stroke = egui::Stroke::new(1.5, pal.accent);
        style.visuals.widgets.active.fg_stroke = egui::Stroke::new(2.0, pal.accent);
        // Open widgets (active combo boxes, text edits in focus)
        style.visuals.widgets.open.bg_fill = pal.bg_raised;
        style.visuals.widgets.open.weak_bg_fill = pal.bg_raised;
        style.visuals.widgets.open.bg_stroke = egui::Stroke::new(1.5, pal.accent);
        style.visuals.widgets.open.fg_stroke = egui::Stroke::new(2.0, pal.accent);
        style.visuals.widgets.open.corner_radius = r;
        style.visuals.selection.bg_fill = pal.accent;
        style.visuals.selection.stroke = egui::Stroke::new(1.5, pal.check_mark);
        style.visuals.hyperlink_color = pal.accent;
        style.visuals.override_text_color = Some(pal.text_primary);
        style.visuals.window_shadow = egui::Shadow {
            offset: [0, 4], blur: 12, spread: 0,
            color: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 15),
        };
        ctx.set_global_style(style);
    }

    /// Absolute, writable directory for recordings. Using an absolute path under
    /// the user's Documents folder (with fallbacks) matters for bundled apps:
    /// a double-clicked macOS `.app` runs with the working directory set to `/`,
    /// so a relative `data/` path is not creatable and recording silently fails.
    fn recordings_dir() -> std::path::PathBuf {
        dirs::document_dir()
            .or_else(dirs::data_dir)
            .or_else(dirs::home_dir)
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("AstroViewer")
    }

    fn start_recording(&mut self) {
        // Create recordings directory (absolute path — see recordings_dir docs)
        let data_dir = Self::recordings_dir();
        if let Err(e) = std::fs::create_dir_all(&data_dir) {
            self.add_log(LogEntry::error(format!(
                "Failed to create recordings dir {}: {}", data_dir.display(), e
            )));
            return;
        }

        let filename = format!("astroviewer-{}.fits", chrono::Local::now().format("%Y%m%d-%H%M%S"));
        let filepath = data_dir.join(&filename);

        let (tx, rx) = bounded::<RecordMsg>(16);
        let log_tx = self.log_tx.clone();
        let full_path = filepath.display().to_string();
        let fname = full_path.clone();
        // Camera name for INSTRUME; file playback isn't an instrument.
        let instrume = match &self.camera_source {
            CameraSource::None | CameraSource::FitsFile(_) => None,
            #[allow(unreachable_patterns)]
            _ => Some(self.source_label()),
        };

        let join = thread::spawn(move || {
            use fitskit::{FitsFile, Hdu, ImageData, PixelData, HeaderValue};
            use std::io::Write;

            // Stream each HDU to disk as its frame arrives (a FITS file is a
            // plain concatenation of HDUs). This keeps memory flat regardless
            // of recording length and means the data is on disk continuously —
            // the old accumulate-then-write design held gigabytes in RAM and
            // wrote them only at Stop, so quitting during that final write
            // truncated the file.
            let file = match std::fs::File::create(&filepath) {
                Ok(f) => f,
                Err(e) => {
                    let _ = log_tx.send(LogEntry::error(format!("Failed to create {}: {}", fname, e)));
                    // Drain until Stop so senders don't block on a full channel.
                    while let Ok(msg) = rx.recv() {
                        if matches!(msg, RecordMsg::Stop) { break; }
                    }
                    return;
                }
            };
            let mut writer = std::io::BufWriter::new(file);

            let mut primary = FitsFile::with_empty_primary();
            primary.primary_mut().header.set("OBJECT", HeaderValue::String("Recording".into()), None);
            if let Some(name) = instrume {
                primary.primary_mut().header.set("INSTRUME", HeaderValue::String(name), Some("camera"));
            }
            // After the first error, keep draining frames (so the UI thread's
            // sends don't back up) but stop writing; the error is reported at
            // Stop.
            let mut write_err: Option<String> =
                primary.primary().write_to(&mut writer).err().map(|e| e.to_string());
            let mut frame_count: u32 = 0;

            while let Ok(msg) = rx.recv() {
                match msg {
                    RecordMsg::Frame { mono, width, height, date_obs, exptime_s, cfa, gain, offset, ccd_temp, set_temp, wcs, bit_depth } => {
                        if write_err.is_some() {
                            continue;
                        }
                        // Store as unsigned 16-bit via BZERO=32768. A U16 frame
                        // converts straight from its native buffer (bit-for-bit
                        // identical to the old f32 path for integer sources); an
                        // F32 frame (float FITS / background-subtracted) keeps
                        // the clamp(0,65535) semantics.
                        let pixels_i16: Vec<i16> = match &mono {
                            Pixels::U16(v) => v.iter().map(|&px| (px as i32 - 32768) as i16).collect(),
                            Pixels::F32(v) => v.iter().map(|&val| {
                                let clamped = val.clamp(0.0, 65535.0) as u16;
                                (clamped as i32 - 32768) as i16
                            }).collect(),
                        };

                        let img = ImageData::new(
                            vec![width as usize, height as usize],
                            PixelData::I16(pixels_i16),
                        );
                        let mut hdu = Hdu::image_extension(img);
                        hdu.header.set("BZERO", HeaderValue::Float(32768.0), Some("unsigned 16-bit offset"));
                        hdu.header.set("BSCALE", HeaderValue::Float(1.0), Some("default scaling"));
                        hdu.header.set("DATE-OBS", HeaderValue::String(date_obs), Some("estimated mid-exposure UTC"));
                        hdu.header.set("EXPTIME", HeaderValue::Float(exptime_s), Some("exposure time in seconds"));
                        hdu.header.set("BITDEPTH", HeaderValue::Integer(bit_depth as i64), Some("sensor ADC bit depth"));
                        // Only written when the pixels still carry an intact
                        // mosaic (no hardware or superpixel binning) so
                        // calibration software never demosaics mono data.
                        if let Some(pat) = cfa {
                            hdu.header.set("BAYERPAT", HeaderValue::String(pat.into()), Some("CFA order of pixel (0,0)"));
                            hdu.header.set("XBAYROFF", HeaderValue::Integer(0), Some("CFA X phase offset"));
                            hdu.header.set("YBAYROFF", HeaderValue::Integer(0), Some("CFA Y phase offset"));
                        }
                        if let Some(g) = gain {
                            hdu.header.set("GAIN", HeaderValue::Float(g), Some("camera gain setting"));
                        }
                        if let Some(o) = offset {
                            hdu.header.set("OFFSET", HeaderValue::Float(o), Some("black level / bias offset (ADU)"));
                        }
                        if let Some(t) = ccd_temp {
                            hdu.header.set("CCD-TEMP", HeaderValue::Float(t), Some("sensor temperature (C)"));
                        }
                        if let Some(t) = set_temp {
                            hdu.header.set("SET-TEMP", HeaderValue::Float(t), Some("cooler setpoint (C)"));
                        }
                        if let Some(w) = wcs {
                            w.write(&mut hdu.header);
                        }
                        match hdu.write_to(&mut writer) {
                            Ok(()) => frame_count += 1,
                            Err(e) => write_err = Some(e.to_string()),
                        }
                    }
                    RecordMsg::Stop => break,
                }
            }

            // Flush the buffered tail and force it to disk before claiming
            // success — BufWriter's Drop silently ignores flush errors.
            if write_err.is_none() {
                let finished = writer
                    .flush()
                    .and_then(|()| writer.into_inner().map_err(|e| e.into_error())?.sync_all());
                if let Err(e) = finished {
                    write_err = Some(e.to_string());
                }
            }

            match write_err {
                Some(e) => {
                    let _ = log_tx.send(LogEntry::error(
                        format!("Failed to write {}: {} ({} frames written)", fname, e, frame_count)
                    ));
                }
                None if frame_count > 0 => {
                    let _ = log_tx.send(LogEntry::info(
                        format!("Recording saved: {} ({} frames)", fname, frame_count)
                    ));
                }
                None => {
                    let _ = std::fs::remove_file(&filepath);
                    let _ = log_tx.send(LogEntry::info("Recording cancelled (no frames)".to_string()));
                }
            }
        });

        self.rec_join = Some(join);
        self.rec_tx = Some(tx);
        self.rec_filename = filename.clone();
        self.rec_frame_count = 0;
        self.recording = true;
        // Tell the GEV capture thread to keep every frame (never skip decode)
        // while recording.
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { ref handle, .. } = self.capture_state {
            handle.recording.store(true, Ordering::Relaxed);
        }
        self.add_log(LogEntry::info(format!("Recording started: {}", full_path)));
    }

    fn stop_recording(&mut self) {
        if let Some(tx) = self.rec_tx.take() {
            let _ = tx.send(RecordMsg::Stop);
        }
        // Wait for the writer to drain the queue, flush, and fsync — the
        // "Recording saved" log line arrives only once the data is on disk,
        // and quitting right after Stop can no longer truncate the file.
        if let Some(jh) = self.rec_join.take() {
            let _ = jh.join();
        }
        self.recording = false;
        // Let the GEV capture thread resume skipping decode on frames the UI
        // can't keep up with.
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { ref handle, .. } = self.capture_state {
            handle.recording.store(false, Ordering::Relaxed);
        }
        self.add_log(LogEntry::info(format!(
            "Recording stopped: {} ({} frames)", self.rec_filename, self.rec_frame_count
        )));
    }

    fn record_frame(&mut self, frame: &FrameData) {
        if let Some(tx) = &self.rec_tx {
            // Exposure, gain, and temperatures from the active camera's controls.
            #[cfg_attr(not(any(feature = "svbony", feature = "gev", feature = "toupcam", feature = "indi")), allow(unused_mut))]
            let mut exposure_us: f64 = 0.0;
            #[cfg_attr(not(any(feature = "svbony", feature = "gev", feature = "toupcam")), allow(unused_mut))]
            let mut gain: Option<f64> = None;
            #[cfg_attr(not(any(feature = "svbony", feature = "gev", feature = "toupcam")), allow(unused_mut))]
            let mut offset: Option<f64> = None;
            #[cfg_attr(not(feature = "toupcam"), allow(unused_mut))]
            let mut ccd_temp: Option<f64> = None;
            #[cfg_attr(not(feature = "toupcam"), allow(unused_mut))]
            let mut set_temp: Option<f64> = None;
            #[cfg(feature = "svbony")]
            if let CaptureState::SVBony { ref control_values, .. } = self.capture_state {
                exposure_us = control_values.iter()
                    .find(|(ct, _, _)| *ct == svbony::ControlType::Exposure)
                    .map(|(_, v, _)| *v as f64)
                    .unwrap_or(0.0);
                gain = control_values.iter()
                    .find(|(ct, _, _)| *ct == svbony::ControlType::Gain)
                    .map(|(_, v, _)| *v as f64);
                offset = control_values.iter()
                    .find(|(ct, _, _)| *ct == svbony::ControlType::BlackLevel)
                    .map(|(_, v, _)| *v as f64);
            }
            #[cfg(feature = "gev")]
            if let CaptureState::Gev { ref controls, .. } = self.capture_state {
                // GenICam ExposureTime is reported in microseconds.
                exposure_us = controls.iter()
                    .find(|c| c.name == "ExposureTime")
                    .map(|c| c.fvalue)
                    .unwrap_or(0.0);
                gain = controls.iter().find(|c| c.name == "Gain").map(|c| c.fvalue);
                offset = controls.iter().find(|c| c.name == "BlackLevel").map(|c| c.fvalue);
            }
            #[cfg(feature = "toupcam")]
            if let CaptureState::Toupcam { ref controls, .. } = self.capture_state {
                exposure_us = controls.exposure_us as f64;
                gain = Some(controls.gain as f64);
                // The pedestal lives in the probe-gated Advanced table.
                offset = controls.advanced.iter()
                    .find(|c| c.opt.0 == toupcam::sys::TOUPCAM_OPTION_BLACKLEVEL)
                    .map(|c| c.value as f64);
                ccd_temp = controls.temperature_c.map(|t| t as f64);
                set_temp = controls.tec_on.then_some(controls.tec_target_c as f64);
            }
            #[cfg(feature = "indi")]
            if let CaptureState::Indi { exposure_s, .. } = self.capture_state {
                // Use the requested exposure — the driver's CCD_EXPOSURE value
                // counts down during the exposure, so it isn't the duration.
                exposure_us = exposure_s * 1_000_000.0;
            }
            let exptime_s = exposure_us / 1_000_000.0;
            // Estimate mid-exposure: now is ~end of readout, so midpoint ≈ now - exposure/2
            let mid = chrono::Utc::now() - chrono::Duration::microseconds((exposure_us / 2.0) as i64);
            let date_obs = mid.format("%Y-%m-%dT%H:%M:%S%.3f").to_string();

            // WCS from the most recent lock. It may trail the frame by a
            // solve job; at live frame rates that is the same pointing.
            #[cfg(feature = "starsolve")]
            let wcs = self
                .last_solve
                .as_ref()
                .and_then(|r| r.as_ref().ok())
                .and_then(|sol| wcs::WcsKeys::from_solution(sol, frame.width, frame.height));
            #[cfg(not(feature = "starsolve"))]
            let wcs = None;
            let msg = RecordMsg::Frame {
                mono: frame.mono.clone(),
                width: frame.width,
                height: frame.height,
                date_obs,
                exptime_s,
                cfa: frame.cfa,
                gain,
                offset,
                ccd_temp,
                set_temp,
                wcs,
                bit_depth: frame.bit_depth,
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

    /// Toolbar Stop. For a GigE session, pause acquisition but keep the
    /// session (control, socket, stream channel) so Play resumes without the
    /// teardown-and-reconnect that wedges some cameras' stream engines. Every
    /// other source stops fully.
    fn pause_or_stop(&mut self) {
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { handle, .. } = &self.capture_state {
            let _ = handle.cmd_tx.send(gev_camera::GevCmd::Pause);
            self.capture_running = false;
            self.frame_times.clear();
            self.fps = 0.0;
            return;
        }
        self.stop_capture();
    }

    /// Toolbar Play. Resume a paused GigE session in place; otherwise open the
    /// selected source fresh.
    fn play_or_resume(&mut self) {
        // Nothing selected: Play used to do nothing, silently. Send the
        // user to the one place a source can be chosen instead.
        if matches!(self.camera_source, CameraSource::None) {
            self.connect_dialog_open = true;
            return;
        }
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { handle, .. } = &self.capture_state {
            let _ = handle.cmd_tx.send(gev_camera::GevCmd::Resume);
            self.capture_running = true;
            return;
        }
        let source = self.camera_source.clone();
        self.open_source(source);
    }

    fn stop_capture(&mut self) {
        match std::mem::replace(&mut self.capture_state, CaptureState::Stopped) {
            CaptureState::Fits { _stop_tx } => {}
            #[cfg(feature = "svbony")]
            CaptureState::SVBony { mut handle, .. } => {
                let _ = handle.cmd_tx.send(camera::CameraCmd::Stop);
                // Wait for capture thread to finish so the SDK cleans up before we drop
                if let Some(jh) = handle.join_handle.take() {
                    let _ = jh.join();
                }
            }
            #[cfg(feature = "gev")]
            CaptureState::Gev { mut handle, .. } => {
                handle.stop();
            }
            #[cfg(feature = "toupcam")]
            CaptureState::Toupcam { mut handle, .. } => {
                let _ = handle.cmd_tx.send(toupcam_camera::ToupCmd::Stop);
                // Wait for the capture thread so the SDK closes cleanly before drop.
                if let Some(jh) = handle.join_handle.take() {
                    let _ = jh.join();
                }
            }
            #[cfg(feature = "indi")]
            CaptureState::Indi { mut handle, .. } => {
                handle.stop();
            }
            CaptureState::Stopped => {}
        }
        self.capture_running = false;
        #[cfg(feature = "gev")]
        { self.frame_pool_return = None; }
        // Reopening a file reloads it, so nothing keeps these alive but us.
        self.fits_frames = None;
        self.pending_bg = None;
        self.frame_times.clear();
        self.fps = 0.0;
        self.pump_drops.store(0, Ordering::Relaxed);
        self.ui_drain_drops = 0;
        while self.frame_rx.try_recv().is_ok() {}
        if self.recording {
            self.stop_recording();
        }
    }

    /// Install `frame` as the current frame, returning the outgoing frame's
    /// `u16` buffer to the GigE decode pool when it is uniquely owned (a clone
    /// still held by the solver or recorder is dropped instead). This is what
    /// makes the pool recycle: the capture thread hands ownership forward, and
    /// the UI hands the spent buffer back here.
    fn replace_current_frame(&mut self, frame: FrameData) {
        let old = self.current_frame.replace(frame);
        #[cfg(feature = "gev")]
        if let (Some(tx), Some(old)) = (&self.frame_pool_return, old) {
            if let Pixels::U16(arc) = old.mono {
                if let Some(buf) = Arc::into_inner(arc) {
                    // Non-blocking: a full pool just drops the buffer.
                    let _ = tx.try_send(buf);
                }
            }
        }
        #[cfg(not(feature = "gev"))]
        drop(old);
    }

    fn start_fits(&mut self, path: std::path::PathBuf) {
        self.stop_capture();
        self.add_log(LogEntry::info(format!(
            "Loading FITS: {}...",
            path.file_name().unwrap_or_default().to_string_lossy()
        )));
        self.camera_source = CameraSource::FitsFile(path.clone());

        // Load in a background thread to keep the UI live. The temporal
        // background is not computed here: it is only needed once the user
        // turns subtraction on, and on a long cube it costs as much as the load.
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
                        self.fits_frames = Some(source.frames());
                        self.bg_image = None;
                        self.bg_computed_pct = None;
                        self.bg_hist_range = None;
                        if nframes < 2 {
                            self.bg_subtract_enabled = false;
                        } else if self.bg_subtract_enabled {
                            // Subtraction was left on from the previous file.
                            self.recompute_background();
                        }
                        let (stop_tx, stop_rx) = bounded(1);
                        start_fits_capture(self.frame_tx.clone(), stop_rx, source, self.fits_fps.clone());
                        self.capture_state = CaptureState::Fits { _stop_tx: stop_tx };
                        self.capture_running = true;
                        self.persist_last_source();
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

    /// Estimate the temporal background from the loaded frames on a worker
    /// thread. One job at a time: if the percentile moves while a job runs,
    /// `poll_bg` notices the mismatch when it lands and starts another.
    fn recompute_background(&mut self) {
        let Some(frames) = self.fits_frames.clone() else { return };
        if frames.num_frames() < 2 || self.pending_bg.is_some() {
            return;
        }
        let percentile = self.bg_percentile;
        let (tx, rx) = bounded(1);
        self.pending_bg = Some(rx);
        std::thread::spawn(move || {
            let t0 = Instant::now();
            let bg = frames.compute_background(percentile);
            let _ = tx.send((percentile, bg, t0.elapsed()));
        });
    }

    fn poll_bg(&mut self) {
        if let Some(rx) = &self.pending_bg {
            if let Ok((pct, bg, elapsed)) = rx.try_recv() {
                self.pending_bg = None;
                self.bg_image = Some(bg);
                self.bg_computed_pct = Some(pct);
                self.bg_hist_range = None;
                let nframes = self.fits_frames.as_ref().map_or(0, |f| f.num_frames());
                self.add_log(LogEntry::info(format!(
                    "Background: {:.0}th percentile of {} frames in {:.0} ms",
                    pct * 100.0, nframes, elapsed.as_secs_f64() * 1e3
                )));
                if self.bg_percentile != pct {
                    self.recompute_background();
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
                let control_values: Vec<_> = handle.controls.iter().zip(handle.initial_values.iter())
                    .map(|(caps, &(val, auto))| (caps.control_type, val, auto))
                    .collect();
                self.add_log(LogEntry::info(format!("Camera opened: {}", info.name)));
                let camera_id = info.camera_id;
                self.capture_state = CaptureState::SVBony { handle, control_values };
                self.camera_source = CameraSource::SVBony(camera_id);
                self.capture_running = true;
                self.persist_last_source();
            }
            Err(e) => {
                let msg = format!("Failed to open camera: {}", e);
                self.camera_error = Some(msg.clone());
                self.add_log(LogEntry::error(msg));
            }
        }
    }

    #[cfg(feature = "toupcam")]
    fn start_toupcam(&mut self, info: &toupcam::DeviceInfo) {
        self.stop_capture();
        self.camera_error = None;

        match toupcam_camera::start_camera(info, self.frame_tx.clone(), self.log_tx.clone()) {
            Ok((handle, controls)) => {
                self.add_log(LogEntry::info(format!("Camera opened: {}", info.display_name)));
                let id = info.id.clone();
                self.capture_state = CaptureState::Toupcam { handle: Box::new(handle), controls: Box::new(controls) };
                self.camera_source = CameraSource::Toupcam(id);
                self.capture_running = true;
                self.persist_last_source();
            }
            Err(e) => {
                let msg = format!("Failed to open camera: {}", e);
                self.camera_error = Some(msg.clone());
                self.add_log(LogEntry::error(msg));
            }
        }
    }

    #[cfg(feature = "gev")]
    fn start_gev(&mut self, info: &gev_camera::GevDeviceInfo) {
        self.stop_capture();
        self.camera_error = None;

        match gev_camera::start_camera(info, self.frame_tx.clone(), self.log_tx.clone()) {
            Ok(handle) => {
                let controls = handle.controls.clone();
                self.frame_pool_return = Some(handle.buffer_return.clone());
                self.add_log(LogEntry::info(format!("GigE camera opened: {}", info.display_name())));
                let id = info.id.clone();
                self.capture_state = CaptureState::Gev { handle, controls };
                self.camera_source = CameraSource::Gev(id);
                self.capture_running = true;
                self.persist_last_source();
                // Remember this IP so the manual connect field is pre-filled
                // on the next launch (and reflects the live connection now).
                self.manual_inputs.insert("gev", info.ip.to_string());
                let _ = std::fs::write(Self::gev_last_ip_path(), info.ip.to_string());
            }
            Err(e) => {
                let msg = format!("Failed to open GigE camera: {}", e);
                self.camera_error = Some(msg.clone());
                self.add_log(LogEntry::error(msg));
            }
        }
    }

    /// Open any source by identity — the single entry point used by the CLI
    /// argument, last-source reconnect, and the Source menu. Cameras absent
    /// from the discovered list trigger one re-enumeration before failing.
    fn open_source(&mut self, source: CameraSource) {
        match source {
            CameraSource::None => {}
            CameraSource::FitsFile(path) => self.start_fits(path),
            #[cfg(feature = "indi")]
            CameraSource::Indi(host) => self.start_indi(&host),
            // Discovered-device backends, dispatched via the unified list.
            #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam"))]
            source => {
                // A raw GigE IP connects directly, without waiting on a
                // broadcast re-enumeration it may never answer anyway.
                #[cfg(feature = "gev")]
                if let CameraSource::Gev(id) = &source {
                    if let Ok(ip) = id.parse::<std::net::Ipv4Addr>() {
                        self.start_gev(&gev_camera::GevDeviceInfo {
                            ip,
                            model: String::new(),
                            manufacturer: String::new(),
                            id: ip.to_string(),
                        });
                        return;
                    }
                }
                // A missing device warrants re-enumerating only its own
                // backend, not sweeping every other backend's bus.
                if self.discovered.iter().all(|d| d.source != source) {
                    self.refresh_backend_of(&source);
                }
                match self.discovered.iter().find(|d| d.source == source) {
                    Some(d) => {
                        let info = d.info.clone();
                        self.start_discovered(&info);
                    }
                    None => self.source_not_found(source.descriptor().unwrap_or_default()),
                }
            }
        }
    }

    /// Open a discovered device with its backend's start function.
    #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam"))]
    fn start_discovered(&mut self, info: &sources::SourceInfo) {
        match info {
            #[cfg(feature = "svbony")]
            sources::SourceInfo::SVBony(info) => self.start_svbony(info),
            #[cfg(feature = "toupcam")]
            sources::SourceInfo::Toupcam(info) => self.start_toupcam(info),
            #[cfg(feature = "gev")]
            sources::SourceInfo::Gev(info) => self.start_gev(info),
        }
    }

    /// Connect to the INDI server at `addr` ("host" or "host:port"). Frames
    /// only start once the user connects a device and starts an exposure from
    /// the Controls tab, so switch there on success.
    #[cfg(feature = "indi")]
    fn start_indi(&mut self, addr: &str) {
        self.stop_capture();
        self.camera_error = None;

        let addr = addr.trim().to_string();
        let (host, port) = match addr.rsplit_once(':') {
            Some((h, p)) if p.parse::<u16>().is_ok() => (h.to_string(), p.parse().unwrap()),
            _ => (addr.clone(), indi_camera::DEFAULT_PORT),
        };
        match indi_camera::start_client(&host, port, self.frame_tx.clone(), self.log_tx.clone()) {
            Ok(handle) => {
                self.add_log(LogEntry::info(format!("INDI: connected to {host}:{port}")));
                self.capture_state = CaptureState::Indi {
                    handle,
                    props: Vec::new(),
                    device: String::new(),
                    exposure_s: 1.0,
                    live: false,
                };
                self.camera_source = CameraSource::Indi(addr);
                self.capture_running = true;
                self.bottom_tab = BottomTab::Controls;
                self.persist_last_source();
            }
            Err(e) => {
                let msg = format!("INDI connect failed: {}", e);
                self.camera_error = Some(msg.clone());
                self.add_log(LogEntry::error(msg));
            }
        }
    }

    #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam"))]
    fn source_not_found(&mut self, descriptor: String) {
        let msg = format!("Source not found: {}", descriptor);
        self.camera_error = Some(msg.clone());
        self.add_log(LogEntry::error(msg));
    }

    /// Re-enumerate every camera backend (the Source menu's Refresh).
    fn refresh_sources(&mut self) {
        self.discovered = sources::discover_all();
    }

    /// Re-enumerate only `source`'s backend, splicing its fresh rows into
    /// `discovered` in place of that backend's old ones.
    #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam"))]
    fn refresh_backend_of(&mut self, source: &CameraSource) {
        let Some(b) = sources::backends().iter().find(|b| Some(b.scheme) == source.scheme())
        else {
            return;
        };
        self.discovered.retain(|d| d.backend != b.name);
        self.discovered.extend(b.discover_devices());
    }

    /// Every discovered source across all backends with kind-tagged labels —
    /// the unified list the Source menu presents.
    fn discovered_source_list(&self) -> Vec<(CameraSource, String)> {
        self.discovered.iter().map(|d| (d.source.clone(), d.label())).collect()
    }

    /// File remembering the last successfully opened source's descriptor.
    fn last_source_path() -> std::path::PathBuf {
        let dir = dirs::config_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        dir.join("last_source")
    }

    /// Called after each successful open so the next launch reconnects.
    fn persist_last_source(&self) {
        if let Some(desc) = self.camera_source.descriptor() {
            let _ = std::fs::write(Self::last_source_path(), desc);
        }
    }

    /// Path of the remembered last-connected GigE IP, used to pre-fill the
    /// manual "Connect to IP" field on the next launch.
    #[cfg(feature = "gev")]
    fn gev_last_ip_path() -> std::path::PathBuf {
        let dir = dirs::config_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        dir.join("gev_last_ip")
    }

    fn poll_frame(&mut self) {
        // Refresh GigE control snapshots (updated values/writability after edits).
        #[cfg(feature = "gev")]
        if let CaptureState::Gev { ref handle, ref mut controls } = self.capture_state {
            while let Ok(snap) = handle.controls_rx.try_recv() {
                *controls = snap;
            }
        }
        // Latest ToupTek telemetry (temperature, power, wheel/focuser state).
        #[cfg(feature = "toupcam")]
        if let CaptureState::Toupcam { ref handle, ref mut controls } = self.capture_state {
            while let Ok(t) = handle.telemetry_rx.try_recv() {
                if let Some(v) = t.temperature_c { controls.temperature_c = Some(v); }
                if let Some(v) = t.power_mw { controls.power_mw = Some(v); }
                if let Some(v) = t.tec_voltage { controls.tec_voltage = Some(v); }
                if let Some(v) = t.chamber_ht { controls.chamber_ht = Some(v); }
                if let Some(v) = t.env_ht { controls.env_ht = Some(v); }
                // Track the camera-chosen exposure while auto-exposure runs.
                if controls.auto_exposure {
                    if let Some(us) = t.real_exposure_us {
                        controls.exposure_us = us.clamp(controls.exposure_min, controls.exposure_max);
                    }
                }
                if let Some(roi) = t.roi { controls.roi = roi; }
                if let (Some(fw), Some(p)) = (controls.filter_wheel.as_mut(), t.filter_position) {
                    fw.position = (p >= 0).then_some(p as u32);
                }
                if let Some(f) = controls.focuser.as_mut() {
                    if let Some(p) = t.focuser_position { f.position = p; }
                    if let Some(m) = t.focuser_moving { f.moving = m; }
                }
            }
        }
        // Refresh INDI property snapshots; default the device picker to the
        // first device with a CONNECTION property (skipping e.g. INDIGO's
        // virtual "Server" device).
        #[cfg(feature = "indi")]
        if let CaptureState::Indi { ref handle, ref mut props, ref mut device, .. } = self.capture_state {
            if let Some(snap) = handle.props.lock().unwrap().take() {
                *props = snap;
            }
            if device.is_empty() {
                if let Some(p) = props
                    .iter()
                    .find(|p| p.name == indi_camera::PROP_CONNECTION)
                    .or(props.first())
                {
                    *device = p.device.clone();
                }
            }
        }
        // Collect the solve worker's last completed frame. Done on every UI
        // update rather than only when a camera frame arrives, so `solve_busy`
        // (and the "Solving…" indicator) reflects the worker, not the frame rate.
        // Centroids and solve arrive together, so matched indices stay aligned.
        #[cfg(feature = "starsolve")]
        if let Ok(out) = self.solve_rx.try_recv() {
            self.solve_busy = false;
            // A job dispatched before Solve was switched off lands here after
            // the switch cleared everything; keeping it would bring the
            // centroid and star overlays back on the next frame. Only the
            // busy flag is taken from it.
            if self.solve_enabled {
                self.centroid_time_ms = out.extract_ms;
                self.centroid_count = out.centroids.len();
                self.last_centroids = out.centroids;
                #[cfg(feature = "focus")]
                if let Some(sample) = out.focus {
                    #[allow(unused_mut)]
                    let mut focuser_pos: Option<i32> = None;
                    #[cfg(feature = "toupcam")]
                    if let CaptureState::Toupcam { ref controls, .. } = self.capture_state {
                        focuser_pos = controls.focuser.as_ref().map(|f| f.position);
                    }
                    self.focus_history.push(&sample, focuser_pos);
                    self.focus_last = Some(sample);
                }
                if let Some(result) = out.solve {
                    // Update FOV estimate from successful solve
                    if let Ok(ref sol) = result {
                        self.fov_estimate_deg = sol.fov_rad.to_degrees();
                    }
                    self.last_solve = Some(result);
                }
            }
        }

        let mut latest = None;
        loop {
            match self.frame_rx.try_recv() {
                Ok(frame) => {
                    // Keep only the newest frame; each superseded one is a drop.
                    if latest.is_some() { self.ui_drain_drops += 1; }
                    latest = Some(frame);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => { self.capture_running = false; break; }
            }
        }
        if let Some(mut frame) = latest {
            // Apply background subtraction if enabled
            if self.bg_subtract_enabled {
                if let Some(bg) = &self.bg_image {
                    if bg.len() == frame.mono.len() {
                        // The subtracted result has negatives, so the frame
                        // becomes the F32 variant. `make_f32_mut` widens a U16
                        // buffer once; the frame is still uniquely owned here
                        // (ahead of the solve-worker dispatch and the recorder),
                        // so an already-F32 buffer edits in place for free.
                        let mono = frame.mono.make_f32_mut();
                        for (px, bg_val) in mono.iter_mut().zip(bg.iter()) {
                            *px -= bg_val;
                        }
                        // Recompute histogram and stats on subtracted data
                        // Use stable range that only expands across frames
                        let (dmin, dmax) = mono.iter().fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &v| (lo.min(v), hi.max(v)));
                        let (rmin, rmax) = match self.bg_hist_range {
                            Some((prev_min, prev_max)) => (prev_min.min(dmin), prev_max.max(dmax)),
                            None => (dmin, dmax),
                        };
                        self.bg_hist_range = Some((rmin, rmax));
                        let (hist, mean, stddev) = compute_histogram_and_stats(mono, histogram::NUM_BINS, rmin, rmax);
                        frame.hist = hist;
                        frame.mean = mean;
                        frame.stddev = stddev;
                    }
                }
            }
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
                    self.display_params.scale_min = zmin as f32;
                    self.display_params.scale_max = zmax as f32;
                }
                ScaleMode::Full => {
                    if self.bg_subtract_enabled && self.bg_image.is_some() {
                        self.display_params.scale_min = frame.hist.data_min;
                        self.display_params.scale_max = frame.hist.data_max;
                    } else {
                        self.display_params.scale_min = 0.0;
                        self.display_params.scale_max = ((1u64 << frame.bit_depth) - 1) as f32;
                    }
                }
                ScaleMode::Manual => {}
            }
            // Auto asinh pivot: the frame median is a robust sky-background
            // estimate (stars occupy a negligible pixel fraction), and
            // frame.hist already reflects background subtraction when enabled.
            if self.asinh_auto_offset && matches!(self.display_params.transfer, imageview::TransferFn::Asinh) {
                self.display_params.asinh_offset = frame.hist.percentile(0.5);
            }
            let now = Instant::now();
            self.frame_times.push(now);
            while self.frame_times.len() > 30 { self.frame_times.remove(0); }
            if self.frame_times.len() >= 2 {
                let dt = self.frame_times.last().unwrap().duration_since(self.frame_times[0]);
                self.fps = (self.frame_times.len() - 1) as f64 / dt.as_secs_f64();
            }
            // Hand this frame to the worker (skipped while it is still busy).
            // Off means off: no extraction job, no widening copy, nothing for
            // the worker to do. The overlays were cleared when it was switched off.
            #[cfg(feature = "starsolve")]
            if self.solve_enabled {
                self.maybe_dispatch_solve(&frame.mono, frame.width, frame.height, frame.bit_depth);
            }

            // Rebuild overlays from the most recent completed extraction. These
            // may lag the displayed frame by a job; at these frame rates the
            // difference is not visible, and it keeps the UI thread free.
            //
            // Capped: each centroid tessellates to a ~360-index ellipse, and
            // wgpu aborts the process past a 256 MB buffer (~180k centroids).
            // A daylight frame on a large sensor can fragment into far more —
            // draw only the first (brightest; tetra3 sorts by mass) few
            // thousand. Extraction and solving still see the full list.
            #[cfg(feature = "starsolve")]
            if self.solve_enabled && self.show_centroids {
                const MAX_CENTROID_OVERLAYS: usize = 4000;
                self.overlay_items = self
                    .last_centroids
                    .iter()
                    .take(MAX_CENTROID_OVERLAYS)
                    .map(overlays::centroid_to_overlay)
                    .collect();
            } else {
                self.overlay_items.clear();
            }

            // Append matched star markers from last solve (every frame)
            #[cfg(feature = "starsolve")]
            if self.show_matched_stars {
                if let Some(Ok(ref sol)) = self.last_solve {
                    // Use matched centroid indices to mark which centroids were matched
                    let n_centroids = self.overlay_items.len();
                    for &cent_idx in &sol.matched_centroid_indices {
                        if cent_idx < n_centroids {
                            if let overlays::OverlayItem::Centroid { x, y, semi_major, .. } = &self.overlay_items[cent_idx] {
                                // Gap sized to clear the centroid ellipse so the
                                // star core stays unobscured.
                                self.overlay_items.push(overlays::OverlayItem::Marker {
                                    x: *x,
                                    y: *y,
                                    kind: overlays::MarkerKind::GappedCrosshair((semi_major * 1.5).max(3.0)),
                                    label: None,
                                });
                            }
                        }
                    }
                }
            }

            // Per-star HFR labels from the last focus measurement, so tilt and
            // field curvature show as a gradient of numbers across the frame.
            #[cfg(feature = "focus")]
            if self.focus_show_labels {
                if let Some(sample) = &self.focus_last {
                    for s in &sample.stars {
                        self.overlay_items.push(overlays::OverlayItem::Marker {
                            x: s.x,
                            y: s.y,
                            kind: overlays::MarkerKind::Label,
                            label: Some(format!("{:.2}", s.hfr)),
                        });
                    }
                }
            }

            // Append named bright star labels from last solve
            #[cfg(feature = "starsolve")]
            if self.show_star_names && self.show_matched_stars && self.show_centroids {
                if let Some(Ok(ref sol)) = self.last_solve {
                    let hw = frame.width as f32 / 2.0;
                    let hh = frame.height as f32 / 2.0;
                    for star in bright_stars::NAMED_STARS {
                        if let Some((px, py)) = sol.world_to_pixel(star.ra_deg, star.dec_deg) {
                            let px = px as f32;
                            let py = py as f32;
                            if px.abs() < hw && py.abs() < hh {
                                self.overlay_items.push(overlays::OverlayItem::Marker {
                                    x: px,
                                    y: py,
                                    kind: overlays::MarkerKind::Label,
                                    label: Some(star.name.to_string()),
                                });
                            }
                        }
                    }
                }
            }

            // Record frame if recording
            if self.recording {
                self.record_frame(&frame);
            }

            self.replace_current_frame(frame);
            self.frame_serial += 1;
        }
    }

    // ── Side panel ──────────────────────────────────────────────────────────

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();

        // Poll pending FITS file dialog result (outside section closure)
        if let Some(rx) = &self.pending_fits_path {
            if let Ok(result) = rx.try_recv() {
                if let Some(path) = result {
                    self.start_fits(path);
                }
                self.pending_fits_path = None;
            }
        }

        section(ui, "Camera", &pal, |ui| {
            // Current source status line
            ui.label(egui::RichText::new(self.source_label()).size(13.0).color(pal.accent));

            if let Some(err) = &self.camera_error {
                ui.label(egui::RichText::new(err).color(pal.status_err).small());
            }

            // Source-specific settings
            if let CameraSource::FitsFile(_) = &self.camera_source {
                ui.add_space(4.0);
                let mut fps = self.fits_fps.load(Ordering::Relaxed);
                let changed = widgets::tip(ui, "Rate at which frames of the FITS file are replayed (loops)", |ui| {
                    widgets::styled_slider_u32(ui, &mut fps, 1..=60, "Playback FPS", &pal)
                });
                if changed {
                    self.fits_fps.store(fps, Ordering::Relaxed);
                }
            }

            // Everything about choosing a source — device lists, manual
            // addresses, FITS files — lives in the Connect dialog.
            ui.add_space(6.0);
            let tip = format!("Choose a camera, INDI server, or FITS file ({})", ui.ctx().format_shortcut(&SC_CONNECT));
            if widgets::tip(ui, &tip, |ui| widgets::styled_button(ui, "Connect\u{2026}", &pal)) {
                self.connect_dialog_open = true;
            }
        });

        ui.add_space(4.0);

        section(ui, "Display", &pal, |ui| {
            let cmap_options: Vec<(ColormapKind, &str)> = ColormapKind::ALL.iter().map(|&k| (k, k.name())).collect();
            let label_w = 65.0;
            egui::Grid::new("display_grid").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Colormap"); });
                let changed = widgets::tip(ui, "False-color palette applied to the scaled pixel values", |ui| {
                    widgets::combo_box(ui, "colormap", "", &mut self.colormap.kind, &cmap_options, &pal)
                });
                if changed {
                    self.colormap = Colormap::new(self.colormap.kind);
                }
                ui.end_row();

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Scale"); });
                widgets::tip(ui, "How the display min/max are chosen.\nFull Range: the sensor bit depth.\nAuto: this frame's min and max.\nZScale: robust range that ignores outliers (DS9-style).\nManual: set below, or drag the lines on the histogram.", |ui| {
                    widgets::combo_box(ui, "scale_mode", "", &mut self.scale_mode, ScaleMode::ALL, &pal);
                });
                ui.end_row();

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Transfer"); });
                widgets::tip(ui, "Linear (with gamma), or asinh to compress a high dynamic range so faint detail and bright cores both stay visible", |ui| {
                    widgets::combo_box(ui, "transfer_fn", "", &mut self.display_params.transfer, imageview::TransferFn::ALL, &pal);
                });
                ui.end_row();

                let gamma_label = match self.display_params.transfer {
                    imageview::TransferFn::Linear => "Gamma",
                    imageview::TransferFn::Asinh => "Alpha",
                };
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label(gamma_label); });
                ui.horizontal(|ui| {
                    // Asinh softening benefits from much larger values than gamma:
                    // faint-end boost is alpha/asinh(alpha), so alpha ~1000 gives ~130x.
                    let log_max = match self.display_params.transfer {
                        imageview::TransferFn::Linear => 1.0,
                        imageview::TransferFn::Asinh => 3.0,
                    };
                    let mut log_gamma = self.display_params.gamma.log10().min(log_max);
                    let gamma_tip = match self.display_params.transfer {
                        imageview::TransferFn::Linear => "Gamma: below 1 brightens faint detail, above 1 darkens it (log-spaced slider)",
                        imageview::TransferFn::Asinh => "Asinh softening: larger values boost the faint end more strongly (log-spaced slider)",
                    };
                    widgets::tip(ui, gamma_tip, |ui| {
                        ui.allocate_ui(egui::vec2(100.0, 20.0), |ui| {
                            widgets::styled_slider_bare(ui, &mut log_gamma, -1.0..=log_max, &pal);
                        });
                    });
                    self.display_params.gamma = 10.0_f32.powf(log_gamma);
                    ui.label(egui::RichText::new(format!("{:.2}", self.display_params.gamma)).monospace().size(12.0));
                });
                ui.end_row();

                if matches!(self.display_params.transfer, imageview::TransferFn::Asinh) {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| { ui.set_width(label_w); ui.label("Offset"); });
                    ui.horizontal(|ui| {
                        let was_auto = self.asinh_auto_offset;
                        widgets::tip(ui, "Asinh pivot: the stretch is linear below this level and logarithmic above it.\nAuto tracks the frame median, a robust sky-background estimate.", |ui| {
                            widgets::styled_checkbox(ui, &mut self.asinh_auto_offset, "Auto", &pal);
                        });
                        if self.asinh_auto_offset {
                            // Refresh immediately on toggle rather than waiting
                            // for the next frame (matters for paused sources).
                            if !was_auto {
                                if let Some(f) = &self.current_frame {
                                    self.display_params.asinh_offset = f.hist.percentile(0.5);
                                }
                            }
                        } else {
                            let (off_lo, off_hi) = (self.display_params.scale_min, self.display_params.scale_max);
                            ui.allocate_ui(egui::vec2(100.0, 20.0), |ui| {
                                widgets::styled_slider_bare(ui, &mut self.display_params.asinh_offset, off_lo..=off_hi, &pal);
                            });
                        }
                        ui.label(egui::RichText::new(format!("{:.1}", self.display_params.asinh_offset)).monospace().size(12.0));
                    });
                    ui.end_row();
                }
            });
            ui.horizontal(|ui| {
                ui.add_space(label_w + 8.0);
                let reset_label = match self.display_params.transfer {
                    imageview::TransferFn::Linear => "Reset Gamma",
                    imageview::TransferFn::Asinh => "Reset Alpha",
                };
                if widgets::tip(ui, "Back to 1.0 (no stretch)", |ui| widgets::styled_button(ui, reset_label, &pal)) {
                    self.display_params.gamma = 1.0;
                }
            });
            if self.scale_mode == ScaleMode::Manual {
                ui.add_space(4.0);
                let (range_lo, range_hi) = if self.bg_subtract_enabled {
                    self.bg_hist_range.unwrap_or((-1000.0, 1000.0))
                } else {
                    let max = self.current_frame.as_ref().map(|f| ((1u64 << f.bit_depth) - 1) as f32).unwrap_or(65535.0);
                    (0.0, max)
                };
                widgets::tip(ui, "Pixel value mapped to the bottom of the colormap. Drag the red line on the histogram for finer control.", |ui| {
                    widgets::styled_slider(ui, &mut self.display_params.scale_min, range_lo..=range_hi, "Min", &pal);
                });
                widgets::tip(ui, "Pixel value mapped to the top of the colormap. Drag the blue line on the histogram for finer control.", |ui| {
                    widgets::styled_slider(ui, &mut self.display_params.scale_max, range_lo..=range_hi, "Max", &pal);
                });
                // Exact entry: the sliders are coarse over a 16-bit range.
                ui.horizontal(|ui| {
                    let step = ((range_hi - range_lo) / 2000.0).max(0.01) as f64;
                    let dp = &mut self.display_params;
                    ui.label("Min");
                    widgets::tip(ui, "Type or drag an exact lower limit", |ui| {
                        ui.add(egui::DragValue::new(&mut dp.scale_min).range(range_lo..=range_hi).speed(step).max_decimals(1));
                    });
                    ui.label("Max");
                    widgets::tip(ui, "Type or drag an exact upper limit", |ui| {
                        ui.add(egui::DragValue::new(&mut dp.scale_max).range(range_lo..=range_hi).speed(step).max_decimals(1));
                    });
                    if dp.scale_max < dp.scale_min {
                        dp.scale_max = dp.scale_min;
                    }
                });
            } else {
                ui.label(format!("Range: {:.0} – {:.0}", self.display_params.scale_min, self.display_params.scale_max));
            }
            ui.add_space(6.0);
            widgets::tip(ui, "Pixel-coordinate ticks along the image edges", |ui| {
                widgets::styled_checkbox(ui, &mut self.display_params.show_axes, "Show Axes", &pal);
            });
            widgets::tip(ui, "Colormap strip labeled with the current display range", |ui| {
                widgets::styled_checkbox(ui, &mut self.display_params.show_colorbar, "Show Colorbar", &pal);
            });
        });

        ui.add_space(4.0);

        section(ui, "Background", &pal, |ui| {
            let can_bg = self.fits_frames.as_ref().is_some_and(|f| f.num_frames() > 1);
            let was_enabled = self.bg_subtract_enabled;
            if can_bg {
                widgets::tip(ui, "Subtract a per-pixel temporal percentile taken across every frame of the FITS file (computed on first use)", |ui| {
                    widgets::styled_checkbox(ui, &mut self.bg_subtract_enabled, "Subtract Background", &pal);
                });
            } else {
                ui.add_enabled(false, egui::Checkbox::new(&mut self.bg_subtract_enabled, "Subtract Background"));
                ui.label(egui::RichText::new("Load a multi-frame FITS to enable").weak().small());
            }
            if can_bg && self.bg_subtract_enabled {
                if self.pending_bg.is_some() {
                    ui.horizontal(|ui| {
                        ui.add(egui::Spinner::new().size(12.0));
                        ui.label(egui::RichText::new("Computing background…").small().color(pal.text_secondary));
                    });
                }
                let old_pct = self.bg_percentile;
                let mut pct_int = (self.bg_percentile * 100.0).round() as u32;
                widgets::tip(ui, "Percentile across frames taken as each pixel's background; lower rejects more of the moving stars", |ui| {
                    widgets::styled_slider_u32(ui, &mut pct_int, 10..=50, "Percentile", &pal);
                });
                self.bg_percentile = pct_int as f32 / 100.0;
                if self.bg_percentile != old_pct {
                    self.recompute_background();
                }
            }
            if !was_enabled && self.bg_subtract_enabled {
                self.scale_mode = ScaleMode::Auto;
                self.bg_hist_range = None;
                if self.bg_computed_pct != Some(self.bg_percentile) {
                    self.recompute_background();
                }
            }
            if was_enabled && !self.bg_subtract_enabled {
                self.bg_hist_range = None;
            }
        });

        ui.add_space(4.0);

        section(ui, "Statistics", &pal, |ui| {
            let lw = 65.0;
            egui::Grid::new("stats_grid").num_columns(2).spacing([8.0, 3.0]).show(ui, |ui| {
                stat_row(ui, lw, "FPS", &format!("{:.1}", self.fps), &pal);
                #[cfg(feature = "gev")]
                if matches!(self.capture_state, CaptureState::Gev { .. }) {
                    stat_row(ui, lw, "Rx rate", &format!("{:.1} MB/s", self.gev_rate_mbps), &pal);
                }
                stat_row(ui, lw, "Dropped", &format!("{}", self.dropped_total()), &pal);
                if let Some(frame) = &self.current_frame {
                    stat_row(ui, lw, "Size", &format!("{} x {}", frame.width, frame.height), &pal);
                    stat_row(ui, lw, "Bit depth", &format!("{}", frame.bit_depth), &pal);
                    stat_row(ui, lw, "Mean", &format!("{:.1}", frame.mean), &pal);
                    stat_row(ui, lw, "Std Dev", &format!("{:.1}", frame.stddev), &pal);
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
        let pal = self.pal();
        // The strip must measure exactly its own height: a collapsed panel
        // is sized from its content, so any extra allocation here pushes the
        // whole strip below the window. Restored at the end, since the tab
        // contents draw in this same ui and would inherit the change.
        let item_spacing_y = ui.spacing().item_spacing.y;
        ui.spacing_mut().item_spacing.y = 0.0;

        let avail = ui.available_rect_before_wrap();
        let bar_rect = egui::Rect::from_min_size(avail.min, egui::vec2(avail.width(), 28.0));
        ui.painter().rect_filled(bar_rect, egui::CornerRadius::ZERO, pal.tab_bar);

        ui.scope_builder(egui::UiBuilder::new().max_rect(bar_rect), |ui| {
            ui.horizontal_centered(|ui| {
                ui.spacing_mut().item_spacing.x = 0.0;

                // Unread warnings and errors show as a count on the Log tab.
                let unread = self.unread_log().count();
                let unread_err = self.unread_log().any(|e| matches!(e.level, LogLevel::Error));
                let badge_font = egui::FontId::new(10.0, egui::FontFamily::Proportional);
                let badge_w = |ui: &egui::Ui| {
                    ui.painter().layout_no_wrap(unread.to_string(), badge_font.clone(), pal.tab_active_text).size().x + 10.0
                };

                for tab in BottomTab::all() {
                    let label = tab.name();
                    let is_active = self.bottom_tab == tab && self.bottom_panel_open;
                    let badge = (tab == BottomTab::Log && unread > 0).then(|| badge_w(ui));
                    let font = egui::FontId::new(12.0, egui::FontFamily::Proportional);
                    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), pal.tab_active_text);
                    let tab_w = galley.size().x + 24.0 + badge.map_or(0.0, |w| w + 4.0);
                    let tab_rect = egui::Rect::from_min_size(
                        ui.cursor().min,
                        egui::vec2(tab_w, 28.0),
                    );
                    let resp = ui.allocate_rect(tab_rect, egui::Sense::click());

                    if is_active {
                        ui.painter().rect_filled(tab_rect, egui::CornerRadius::ZERO, pal.tab_active_bg);
                        ui.painter().hline(
                            tab_rect.x_range(),
                            tab_rect.min.y,
                            egui::Stroke::new(2.0, pal.accent),
                        );
                    } else if resp.hovered() {
                        ui.painter().rect_filled(tab_rect, egui::CornerRadius::ZERO, pal.tab_hover_bg);
                    }

                    let text_color = if is_active { pal.tab_active_text } else { pal.tab_inactive_text };
                    let text_pos = match badge {
                        Some(w) => egui::Pos2::new(tab_rect.center().x - (w + 4.0) / 2.0, tab_rect.center().y),
                        None => tab_rect.center(),
                    };
                    ui.painter().text(text_pos, egui::Align2::CENTER_CENTER, label, font, text_color);
                    if let Some(w) = badge {
                        let color = if unread_err { pal.status_err } else { pal.status_warn };
                        let center = egui::Pos2::new(text_pos.x + galley.size().x / 2.0 + 4.0 + w / 2.0, tab_rect.center().y);
                        let rect = egui::Rect::from_center_size(center, egui::vec2(w, 14.0));
                        ui.painter().rect_filled(rect, egui::CornerRadius::same(7), color);
                        ui.painter().text(center, egui::Align2::CENTER_CENTER, unread.to_string(), badge_font.clone(), egui::Color32::WHITE);
                    }

                    if resp.clicked() {
                        self.bottom_tab = tab;
                        self.bottom_panel_open = true;
                    }
                }

                // Collapse / expand chevron at the far right of the strip.
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add_space(6.0);
                    let glyph = if self.bottom_panel_open { "\u{25BC}" } else { "\u{25B2}" };
                    let tip = if self.bottom_panel_open { "Collapse the panel (B)" } else { "Expand the panel (B)" };
                    let (rect, resp) = ui.allocate_exact_size(egui::vec2(22.0, 22.0), egui::Sense::click());
                    if resp.hovered() {
                        ui.painter().rect_filled(rect, egui::CornerRadius::same(4), pal.tab_hover_bg);
                    }
                    ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, glyph, egui::FontId::proportional(9.0), pal.tab_inactive_text);
                    if resp.on_hover_text(tip).clicked() {
                        self.bottom_panel_open = !self.bottom_panel_open;
                    }
                });
            });
        });

        ui.allocate_rect(bar_rect, egui::Sense::hover());
        ui.spacing_mut().item_spacing.y = item_spacing_y;
    }

    fn histogram_content(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        if self.current_frame.is_none() { return; }
        let has_rgb = self.current_frame.as_ref().is_some_and(|f| f.channel_hists.is_some()) && !self.bg_subtract_enabled;

        // ── Toolbar: Log Y, RGB, x-range ────────────────────────────────────
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            widgets::tip(ui, "Plot log₁₀(count + 1) so faint tails stay visible beside the sky peak", |ui| {
                widgets::styled_checkbox(ui, &mut self.hist_log_y, "Log Y", &pal);
            });
            if has_rgb {
                ui.add_space(8.0);
                widgets::tip(ui, "Overlay per-channel R/G/B histograms of the raw Bayer mosaic (G sits ~2× higher: it covers half the sensor)", |ui| {
                    widgets::styled_checkbox(ui, &mut self.hist_rgb, "RGB", &pal);
                });
            }
            ui.add_space(12.0);
            ui.label(egui::RichText::new("X axis").color(pal.text_secondary));
            widgets::tip(ui, "Full range: the sensor bit depth.\nFit data: the span of pixel values in this frame.\nDisplay range: the current scale min–max.", |ui| {
                widgets::combo_box(ui, "hist_x_range", "", &mut self.hist_x_range, HistXRange::ALL, &pal);
            });
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    egui::RichText::new("drag the red / blue lines to set a manual display range")
                        .size(11.0)
                        .color(pal.text_secondary),
                );
            });
        });

        // ── Curves for the selected window ──────────────────────────────────
        // Everything derived from the frame is pulled into locals here so the
        // borrow ends before the drag handler below mutates display state.
        let Some(frame) = &self.current_frame else { return };
        let hist = &frame.hist;
        let full_lo = hist.edges.first().copied().unwrap_or(0.0) as f64;
        let full_hi = hist.edges.last().copied().unwrap_or(65535.0) as f64;
        let bit_max = ((1u64 << frame.bit_depth) - 1) as f64;
        let smin = self.display_params.scale_min as f64;
        let smax = self.display_params.scale_max as f64;
        let manual = self.scale_mode == ScaleMode::Manual;

        let pad = |lo: f64, hi: f64| {
            let w = (hi - lo).max(1.0);
            (lo - w * 0.03, hi + w * 0.03)
        };
        let (lo, hi) = match self.hist_x_range {
            HistXRange::Full => (full_lo, full_hi),
            HistXRange::Data => {
                let (mut lo, mut hi) = pad(hist.data_min as f64, hist.data_max as f64);
                // Keep user-placed lines reachable while fitted to the data.
                if manual { lo = lo.min(smin); hi = hi.max(smax); }
                (lo, hi)
            }
            HistXRange::Display => pad(smin, smax),
        };
        let (lo, hi) = (lo.max(full_lo), hi.min(full_hi));
        let (lo, hi) = if hi <= lo { (full_lo, full_hi) } else { (lo, hi) };

        let log_y = self.hist_log_y;
        let main_line = hist_step_line(hist, lo, hi, log_y);
        let rgb_lines: Vec<(&'static str, egui::Color32, Vec<[f64; 2]>)> = if has_rgb && self.hist_rgb {
            frame.channel_hists.as_ref().map_or(Vec::new(), |chs| {
                vec![
                    ("R", egui::Color32::from_rgb(235, 87, 87), hist_step_line(&chs[0], lo, hi, log_y)),
                    ("G", egui::Color32::from_rgb(76, 187, 106), hist_step_line(&chs[1], lo, hi, log_y)),
                    ("B", egui::Color32::from_rgb(96, 165, 250), hist_step_line(&chs[2], lo, hi, log_y)),
                ]
            })
        } else {
            Vec::new()
        };

        let plot_height = ui.available_height().max(80.0);
        let y_label = if log_y { "log₁₀(count+1)" } else { "" };

        // A single coarse "nice" step (~4-5 lines) across whatever window is
        // shown keeps the histogram surgical instead of graph-papery.
        let grid_step = {
            let raw = ((hi - lo) / 4.0).max(1e-6);
            let mag = 10f64.powf(raw.log10().floor());
            let norm = raw / mag;
            let nice = if norm < 1.5 { 1.0 } else if norm < 3.0 { 2.0 } else if norm < 7.0 { 5.0 } else { 10.0 };
            nice * mag
        };

        // One grab zone, in screen pixels, shared by the hover highlight, the
        // drag start, and the resize cursor, so the three always agree and the
        // target stays the same size at any panel width or x-range.
        const GRAB_PX: f64 = 12.0;
        let mut near_line = false;
        let plot_resp = egui_plot::Plot::new("histogram")
            .height(plot_height)
            .y_axis_label(y_label)
            .show_axes([true, false])
            .allow_drag(false).allow_zoom(false).allow_scroll(false).allow_boxed_zoom(false)
            .show_grid([true, false])
            .x_grid_spacer(egui_plot::uniform_grid_spacer(move |_| [grid_step, grid_step * 5.0, grid_step * 10.0]))
            .x_axis_label("Pixel Value")
            .include_x(lo)
            .include_x(hi)
            .include_y(0.0)
            .set_margin_fraction(egui::vec2(0.01, 0.0))
            .show(ui, |plot_ui| {
                plot_ui.line(
                    egui_plot::Line::new("histogram", egui_plot::PlotPoints::from(main_line))
                        .color(pal.plot_line)
                        .width(1.5)
                        .fill(0.0)
                        .fill_alpha(0.35),
                );
                for (name, color, pts) in rgb_lines {
                    plot_ui.line(
                        egui_plot::Line::new(name, egui_plot::PlotPoints::from(pts))
                            .color(color)
                            .width(1.0),
                    );
                }

                // Display-range lines, in every scale mode: dimmed when the
                // mode sets them automatically, solid once they are manual.
                // Dragging either one switches to Manual, so showing them is
                // what makes that discoverable.
                let grab_radius_data = GRAB_PX * plot_ui.transform().dvalue_dpos()[0].abs();
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
                near_line = near_min || near_max;
                let line_style = |near: bool, bright: egui::Color32, base: (u8, u8, u8)| -> (egui::Color32, f32) {
                    if near {
                        (bright, 4.0)
                    } else if manual {
                        (egui::Color32::from_rgba_unmultiplied(base.0, base.1, base.2, 200), 3.0)
                    } else {
                        (egui::Color32::from_rgba_unmultiplied(base.0, base.1, base.2, 110), 2.0)
                    }
                };
                let (min_c, min_w) = line_style(near_min, egui::Color32::from_rgb(252, 85, 85), (220, 50, 50));
                plot_ui.vline(egui_plot::VLine::new("scale_min", smin).color(min_c).width(min_w).style(egui_plot::LineStyle::Solid));
                let (max_c, max_w) = line_style(near_max, egui::Color32::from_rgb(96, 165, 250), (59, 130, 246));
                plot_ui.vline(egui_plot::VLine::new("scale_max", smax).color(max_c).width(max_w).style(egui_plot::LineStyle::Solid));
            });

        // ── Drag handling ───────────────────────────────────────────────────
        if near_line || self.hist_drag.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        }
        if plot_resp.response.dragged_by(egui::PointerButton::Primary) {
            if let Some(ptr) = plot_resp.response.hover_pos() {
                let transform = plot_resp.transform;
                let plot_x = transform.value_from_position(ptr).x;
                let grab_radius_data = GRAB_PX * transform.dvalue_dpos()[0].abs();
                if plot_resp.response.drag_started() {
                    let dist_min = (plot_x - smin).abs();
                    let dist_max = (plot_x - smax).abs();
                    if dist_min < grab_radius_data && dist_min <= dist_max { self.hist_drag = Some(HistDrag::Min); }
                    else if dist_max < grab_radius_data { self.hist_drag = Some(HistDrag::Max); }
                }
                let (drag_lo, drag_hi) = if self.bg_subtract_enabled {
                    self.bg_hist_range.map(|(lo, hi)| (lo as f64, hi as f64)).unwrap_or((-1000.0, 1000.0))
                } else {
                    (0.0, bit_max)
                };
                match self.hist_drag {
                    Some(HistDrag::Min) => {
                        self.scale_mode = ScaleMode::Manual;
                        self.display_params.scale_min = plot_x.max(drag_lo).min(self.display_params.scale_max as f64 - 1.0) as f32;
                    }
                    Some(HistDrag::Max) => {
                        self.scale_mode = ScaleMode::Manual;
                        self.display_params.scale_max = plot_x.max(self.display_params.scale_min as f64 + 1.0).min(drag_hi) as f32;
                    }
                    None => {}
                }
            }
        }
        if plot_resp.response.drag_stopped() { self.hist_drag = None; }
    }

    #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam", feature = "indi"))]
    fn controls_content(&mut self, ui: &mut egui::Ui) {
        #[cfg(feature = "gev")]
        if matches!(self.capture_state, CaptureState::Gev { .. }) {
            self.gev_controls_content(ui);
            return;
        }
        #[cfg(feature = "toupcam")]
        if matches!(self.capture_state, CaptureState::Toupcam { .. }) {
            self.toupcam_controls_content(ui);
            return;
        }
        #[cfg(feature = "indi")]
        if matches!(self.capture_state, CaptureState::Indi { .. }) {
            self.indi_controls_content(ui);
            return;
        }
        #[cfg(feature = "svbony")]
        {
            let pal = self.pal();
            if let CaptureState::SVBony { ref handle, ref mut control_values } = self.capture_state {
                let n = handle.controls.len();
                let mid = (n + 1) / 2;
                let label_w = 120.0_f32;
                let value_w = 80.0_f32;
                let auto_w = 65.0_f32;

                // Two-column layout
                ui.columns(2, |cols| {
                    let col_w = cols[0].available_width();
                    let slider_w = (col_w - label_w - value_w - auto_w - 24.0).max(60.0);
                    cols[0].spacing_mut().item_spacing.y = 8.0;
                    for idx in 0..mid {
                        Self::draw_control(&mut cols[0], &handle.controls[idx], &mut control_values[idx],
                            &handle.cmd_tx, label_w, slider_w, value_w, auto_w, &pal);
                    }
                    let col_w = cols[1].available_width();
                    let slider_w = (col_w - label_w - value_w - auto_w - 24.0).max(60.0);
                    cols[1].spacing_mut().item_spacing.y = 8.0;
                    for idx in mid..n {
                        Self::draw_control(&mut cols[1], &handle.controls[idx], &mut control_values[idx],
                            &handle.cmd_tx, label_w, slider_w, value_w, auto_w, &pal);
                    }
                });
                return;
            }
        }
        // No camera connected (or non-SVBony build).
        let pal = self.pal();
        ui.vertical_centered(|ui| {
            ui.add_space(20.0);
            ui.label(egui::RichText::new("No camera connected").color(pal.text_secondary));
        });
    }

    /// Render the curated ToupTek controls: exposure (+auto/target), gain, and
    /// speed always; Cooling, Corrections, Filter Wheel, and Focuser groups
    /// only when the model has the hardware; and a collapsed Advanced section
    /// for the capability-gated option table. The toupcam SDK has no runtime
    /// control discovery, so anything shown here is hand-picked; options not
    /// listed stay at their SDK defaults.
    #[cfg(feature = "toupcam")]
    fn toupcam_controls_content(&mut self, ui: &mut egui::Ui) {
        use toupcam_camera::{CorrectionAction, ToupCmd};
        let pal = self.pal();
        let log_tx = self.log_tx.clone();
        let CaptureState::Toupcam { ref handle, ref mut controls } = self.capture_state else { return };

        let label_w = 120.0_f32;
        let value_w = 80.0_f32;
        let slider_w = (ui.available_width() * 0.5 - label_w - value_w - 90.0).max(60.0);

        ui.add_space(6.0);
        egui::Grid::new("toup_controls")
            .num_columns(4)
            .spacing([4.0, 8.0])
            .show(ui, |ui| {
                // ── Exposure: log slider + unit-aware entry + Auto ──────────
                ctrl_label(ui, label_w, "Exposure");
                let old_exp = controls.exposure_us;
                // Slider capped at 10 s so the log scale stays usable; the
                // entry field still accepts the full range.
                let slider_max = (controls.exposure_max as f32).min(10_000_000.0);
                let mut v = controls.exposure_us as f32;
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_log_bare(
                        ui, &mut v,
                        (controls.exposure_min as f32).max(1.0)..=slider_max,
                        &pal,
                    );
                });
                let mut v_us = v.round() as f64;
                let speed = (v_us * 0.005).max(1.0);
                let dv = egui::DragValue::new(&mut v_us)
                    .range(controls.exposure_min as f64..=controls.exposure_max as f64)
                    .speed(speed)
                    .custom_formatter(|v, _| {
                        if v >= 1_000_000.0 { format!("{:.2} s", v / 1_000_000.0) }
                        else if v >= 1_000.0 { format!("{:.1} ms", v / 1_000.0) }
                        else { format!("{:.0} µs", v) }
                    })
                    .custom_parser(|s| {
                        let s = s.trim();
                        if let Some(n) = s.strip_suffix("ms").and_then(|n| n.trim().parse::<f64>().ok()) {
                            Some(n * 1_000.0)
                        } else if let Some(n) = s.strip_suffix("µs").or_else(|| s.strip_suffix("us")).and_then(|n| n.trim().parse::<f64>().ok()) {
                            Some(n)
                        } else if let Some(n) = s.strip_suffix('s').and_then(|n| n.trim().parse::<f64>().ok()) {
                            Some(n * 1_000_000.0)
                        } else {
                            s.parse::<f64>().ok()
                        }
                    });
                ui.add_sized([value_w, 20.0], dv);
                controls.exposure_us = (v_us.round() as u32)
                    .clamp(controls.exposure_min, controls.exposure_max);
                if controls.exposure_us != old_exp {
                    let _ = handle.cmd_tx.send(ToupCmd::SetExposure(controls.exposure_us));
                }
                let mut auto = controls.auto_exposure;
                if widgets::styled_checkbox(ui, &mut auto, "Auto", &pal) {
                    controls.auto_exposure = auto;
                    let _ = handle.cmd_tx.send(ToupCmd::SetAutoExposure(auto));
                }
                ui.end_row();

                // ── Auto-exposure target brightness (only meaningful in auto)
                if controls.auto_exposure {
                    ctrl_label(ui, label_w, "AE Target");
                    let old = controls.auto_expo_target;
                    let mut t = controls.auto_expo_target as f32;
                    ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                        widgets::styled_slider_bare(ui, &mut t, 16.0..=220.0, &pal);
                    });
                    controls.auto_expo_target = t.round() as u16;
                    ui.add_sized([value_w, 20.0], egui::DragValue::new(&mut controls.auto_expo_target).range(16..=220));
                    if controls.auto_expo_target != old {
                        let _ = handle.cmd_tx.send(ToupCmd::SetAutoExpoTarget(controls.auto_expo_target));
                    }
                    ui.end_row();
                }

                // ── Gain (percent, 100 = 1×) ────────────────────────────────
                ctrl_label(ui, label_w, "Gain");
                let old_gain = controls.gain;
                let mut g = controls.gain as f32;
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(
                        ui, &mut g,
                        controls.gain_min as f32..=(controls.gain_max.max(controls.gain_min + 1)) as f32,
                        &pal,
                    );
                });
                controls.gain = g.round() as u16;
                ui.add_sized(
                    [value_w, 20.0],
                    egui::DragValue::new(&mut controls.gain)
                        .range(controls.gain_min..=controls.gain_max.max(controls.gain_min + 1))
                        .suffix(" %"),
                );
                if controls.gain != old_gain {
                    let _ = handle.cmd_tx.send(ToupCmd::SetGain(controls.gain));
                }
                ui.end_row();

                // ── Frame-speed level (USB/link bandwidth tier) ─────────────
                if controls.max_speed > 0 {
                    ctrl_label(ui, label_w, "Speed Level");
                    let old = controls.speed;
                    let mut s = controls.speed as f32;
                    ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                        widgets::styled_slider_bare(ui, &mut s, 0.0..=controls.max_speed as f32, &pal);
                    });
                    controls.speed = s.round() as u16;
                    ui.add_sized(
                        [value_w, 20.0],
                        egui::DragValue::new(&mut controls.speed)
                            .range(0..=controls.max_speed)
                            .suffix(format!(" / {}", controls.max_speed)),
                    );
                    if controls.speed != old {
                        let _ = handle.cmd_tx.send(ToupCmd::SetSpeed(controls.speed));
                    }
                    ui.end_row();
                }

                // ── Capture mode: free-running video vs software trigger ────
                if controls.has_soft_trigger {
                    ctrl_label(ui, label_w, "Capture Mode");
                    let mut mode = controls.trigger_mode as i32;
                    if widgets::combo_box(
                        ui, "toup_trig_mode", "", &mut mode,
                        &[(0, "Video"), (1, "Triggered")], &pal,
                    ) {
                        controls.trigger_mode = mode != 0;
                        let _ = handle.cmd_tx.send(ToupCmd::SetTriggerMode(controls.trigger_mode));
                    }
                    if controls.trigger_mode
                        && ui.button("Snap")
                            .on_hover_text("Fire one software-triggered exposure")
                            .clicked()
                    {
                        let _ = handle.cmd_tx.send(ToupCmd::Snap);
                    }
                    ui.end_row();
                }

                // ── Sensor readout resolution (restarts the stream) ─────────
                if controls.resolutions.len() > 1 {
                    ctrl_label(ui, label_w, "Resolution");
                    let labels: Vec<String> = controls
                        .resolutions
                        .iter()
                        .map(|(w, h)| format!("{} × {}", w, h))
                        .collect();
                    let opts: Vec<(u32, &str)> = labels
                        .iter()
                        .enumerate()
                        .map(|(i, s)| (i as u32, s.as_str()))
                        .collect();
                    let mut sel = controls.resolution_index;
                    if widgets::combo_box(ui, "toup_resolution", "", &mut sel, &opts, &pal) {
                        controls.resolution_index = sel;
                        let _ = handle.cmd_tx.send(ToupCmd::SetResolution(sel));
                        // ROI resets to the new full frame; telemetry corrects.
                        if let Some(&(w, h)) = controls.resolutions.get(sel as usize) {
                            controls.roi = (0, 0, w, h);
                            controls.roi_edit = controls.roi;
                        }
                    }
                    ui.end_row();
                }

                // ── Software 2×2 superpixel binning (color sensors) ─────────
                if controls.is_color {
                    ctrl_label(ui, label_w, "Superpixel");
                    let mut sp = controls.superpixel;
                    if widgets::styled_checkbox(ui, &mut sp, "2×2 mono bin", &pal) {
                        controls.superpixel = sp;
                        let _ = handle.cmd_tx.send(ToupCmd::SetSuperpixel(sp));
                    }
                    ui.label(
                        egui::RichText::new("exact CFA average, ½ res")
                            .size(11.0).color(pal.text_secondary),
                    ).on_hover_text(
                        "Averages each 2×2 Bayer cell into one mono pixel — removes the \
                         mosaic checkerboard without interpolation. Applies to display, \
                         statistics, and recording.",
                    );
                    ui.end_row();
                }

                // ── Cooling (TEC-equipped models only) ──────────────────────
                if controls.has_tec {
                    ctrl_label(ui, label_w, "Cooler");
                    let mut on = controls.tec_on;
                    if widgets::styled_checkbox(ui, &mut on, "TEC on", &pal) {
                        controls.tec_on = on;
                        let _ = handle.cmd_tx.send(ToupCmd::SetOption(toupcam::Opt::TEC, on as i32));
                    }
                    ui.end_row();

                    ctrl_label(ui, label_w, "Target Temp");
                    let old_target = controls.tec_target_c;
                    let mut t = controls.tec_target_c;
                    let (tec_lo, tec_hi) = controls.tec_range;
                    ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                        widgets::styled_slider_bare(ui, &mut t, tec_lo..=tec_hi, &pal);
                    });
                    ui.add_sized(
                        [value_w, 20.0],
                        egui::DragValue::new(&mut t)
                            .range(tec_lo..=tec_hi)
                            .speed(0.1)
                            .fixed_decimals(1)
                            .suffix(" °C"),
                    );
                    if (t - old_target).abs() >= 0.05 {
                        controls.tec_target_c = t;
                        let _ = handle.cmd_tx.send(ToupCmd::SetOption(
                            toupcam::Opt::TEC_TARGET,
                            (t * 10.0).round() as i32,
                        ));
                    }
                    ui.end_row();
                }

                // ── Read-only telemetry ─────────────────────────────────────
                if let Some(temp) = controls.temperature_c {
                    ctrl_label(ui, label_w, "Sensor Temp");
                    ui.label(
                        egui::RichText::new(format!("{:.1} °C", temp))
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();
                }
                if let Some(mw) = controls.power_mw {
                    ctrl_label(ui, label_w, "Power Draw");
                    ui.label(
                        egui::RichText::new(format!("{:.1} W", mw as f32 / 1000.0))
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();
                }
                if let Some(v) = controls.tec_voltage {
                    ctrl_label(ui, label_w, "TEC Drive");
                    let text = match controls.tec_voltage_max {
                        Some(max) => format!("{:.1} V / {:.1} V max", v, max),
                        None => format!("{:.1} V", v),
                    };
                    ui.label(
                        egui::RichText::new(text)
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();
                }
                if let Some((rh, t)) = controls.chamber_ht {
                    ctrl_label(ui, label_w, "Chamber");
                    ui.label(
                        egui::RichText::new(format!("{:.1} %RH  {:.1} °C", rh, t))
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();
                }
                if let Some((rh, t)) = controls.env_ht {
                    ctrl_label(ui, label_w, "Ambient");
                    ui.label(
                        egui::RichText::new(format!("{:.1} %RH  {:.1} °C", rh, t))
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();
                }
            });

        // ── Advanced: curated, capability-gated SDK options ─────────────────
        if !controls.advanced.is_empty() {
            use toupcam_camera::AdvKind;
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Advanced").strong().color(pal.accent),
            )
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("toup_advanced")
                    .num_columns(3)
                    .spacing([4.0, 8.0])
                    .show(ui, |ui| {
                        for c in controls.advanced.iter_mut() {
                            ctrl_label(ui, label_w, c.label);
                            let old = c.value;
                            match c.kind {
                                AdvKind::Bool => {
                                    let mut on = c.value != 0;
                                    if widgets::styled_checkbox(ui, &mut on, "", &pal) {
                                        c.value = on as i32;
                                    }
                                }
                                AdvKind::Int { min, max } => {
                                    let mut v = c.value as f32;
                                    ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                                        widgets::styled_slider_bare(ui, &mut v, min as f32..=max as f32, &pal);
                                    });
                                    c.value = v.round() as i32;
                                    ui.add_sized([value_w, 20.0], egui::DragValue::new(&mut c.value).range(min..=max));
                                }
                                AdvKind::Enum(variants) => {
                                    widgets::combo_box(ui, c.label, "", &mut c.value, variants, &pal);
                                }
                            }
                            if c.value != old {
                                let _ = handle.cmd_tx.send(ToupCmd::SetOption(c.opt, c.value));
                            }
                            ui.end_row();
                        }
                    });
            });
        }

        // ── Hardware sensor ROI (reduces readout region on-camera) ──────────
        {
            let (full_w, full_h) = controls
                .resolutions
                .get(controls.resolution_index as usize)
                .copied()
                .unwrap_or((controls.roi.2, controls.roi.3));
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Sensor ROI").strong().color(pal.accent),
            )
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("toup_roi").num_columns(5).spacing([6.0, 8.0]).show(ui, |ui| {
                    ctrl_label(ui, label_w, "Active");
                    let (x, y, w, h) = controls.roi;
                    ui.label(
                        egui::RichText::new(format!("{} × {} @ ({}, {})", w, h, x, y))
                            .monospace().size(12.0).color(pal.text_secondary),
                    );
                    ui.end_row();

                    let e = &mut controls.roi_edit;
                    ctrl_label(ui, label_w, "Offset");
                    ui.add(egui::DragValue::new(&mut e.0).range(0..=full_w.saturating_sub(8)).speed(2).prefix("x "));
                    ui.add(egui::DragValue::new(&mut e.1).range(0..=full_h.saturating_sub(8)).speed(2).prefix("y "));
                    ui.end_row();

                    ctrl_label(ui, label_w, "Size");
                    ui.add(egui::DragValue::new(&mut e.2).range(8..=full_w).speed(2).prefix("w "));
                    ui.add(egui::DragValue::new(&mut e.3).range(8..=full_h).speed(2).prefix("h "));
                    if ui.button("Apply").on_hover_text("Set the sensor readout region (values rounded to even)").clicked() {
                        // SDK constraints: offsets/sizes even, minimum 8×8,
                        // region inside the frame.
                        e.0 = (e.0 & !1).min(full_w.saturating_sub(8));
                        e.1 = (e.1 & !1).min(full_h.saturating_sub(8));
                        e.2 = (e.2 & !1).clamp(8, full_w - e.0);
                        e.3 = (e.3 & !1).clamp(8, full_h - e.1);
                        let _ = handle.cmd_tx.send(ToupCmd::SetRoi(e.0, e.1, e.2, e.3));
                    }
                    if ui.button("Full Frame").clicked() {
                        *e = (0, 0, full_w, full_h);
                        let _ = handle.cmd_tx.send(ToupCmd::SetRoi(0, 0, 0, 0));
                    }
                    ui.end_row();
                });
            });
        }

        // ── Corrections: on-camera flat/dark/fixed-pattern pipelines ────────
        if !controls.corrections.is_empty() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Corrections").strong().color(pal.accent),
            )
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("toup_corrections")
                    .num_columns(5)
                    .spacing([6.0, 8.0])
                    .show(ui, |ui| {
                        for c in controls.corrections.iter_mut() {
                            let kind = c.kind;
                            ctrl_label(ui, label_w, kind.label());
                            let mut on = c.enabled;
                            if widgets::styled_checkbox(ui, &mut on, "Apply", &pal) {
                                c.enabled = on;
                                let _ = handle.cmd_tx.send(ToupCmd::Correction(
                                    kind, CorrectionAction::Enable(on),
                                ));
                            }
                            if ui.button("Capture").on_hover_text(kind.capture_hint()).clicked() {
                                let _ = log_tx.try_send(LogEntry::info(format!(
                                    "{}: {}", kind.label(), kind.capture_hint(),
                                )));
                                let _ = handle.cmd_tx.send(ToupCmd::Correction(
                                    kind, CorrectionAction::Capture,
                                ));
                                c.enabled = true; // a successful capture auto-applies
                            }
                            if ui.button("Import…").clicked() {
                                if let Some(path) = rfd::FileDialog::new().pick_file() {
                                    let _ = handle.cmd_tx.send(ToupCmd::Correction(
                                        kind,
                                        CorrectionAction::Import(path.to_string_lossy().into_owned()),
                                    ));
                                    c.enabled = true;
                                }
                            }
                            if ui.button("Export…").clicked() {
                                let default = format!(
                                    "{}.dat",
                                    kind.label().to_lowercase().replace(' ', "_")
                                );
                                if let Some(path) = rfd::FileDialog::new().set_file_name(default).save_file() {
                                    let _ = handle.cmd_tx.send(ToupCmd::Correction(
                                        kind,
                                        CorrectionAction::Export(path.to_string_lossy().into_owned()),
                                    ));
                                }
                            }
                            ui.end_row();
                        }
                    });
            });
        }

        // ── Filter wheel (models with an integrated/connected wheel) ────────
        if let Some(fw) = controls.filter_wheel.as_mut() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Filter Wheel").strong().color(pal.accent),
            )
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("toup_wheel").num_columns(3).spacing([6.0, 8.0]).show(ui, |ui| {
                    ctrl_label(ui, label_w, "Position");
                    match fw.position {
                        Some(pos) => {
                            let labels: Vec<String> =
                                (0..fw.slots).map(|i| format!("Slot {}", i + 1)).collect();
                            let opts: Vec<(u32, &str)> = labels
                                .iter()
                                .enumerate()
                                .map(|(i, s)| (i as u32, s.as_str()))
                                .collect();
                            let mut sel = pos.min(fw.slots.saturating_sub(1));
                            if widgets::combo_box(ui, "toup_wheel_pos", "", &mut sel, &opts, &pal) {
                                let _ = handle.cmd_tx.send(ToupCmd::SetFilterPosition(sel));
                                fw.position = None; // shows "moving" until telemetry updates
                            }
                        }
                        None => {
                            ui.label(
                                egui::RichText::new("moving…")
                                    .color(pal.text_secondary).italics(),
                            );
                        }
                    }
                    if ui.button("Home").on_hover_text("Reset / re-home the wheel").clicked() {
                        let _ = handle.cmd_tx.send(ToupCmd::ResetFilterWheel);
                        fw.position = None;
                    }
                    ui.end_row();
                });
            });
        }

        // ── Astro auto-focuser ───────────────────────────────────────────────
        if let Some(f) = controls.focuser.as_mut() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Focuser").strong().color(pal.accent),
            )
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("toup_focuser").num_columns(4).spacing([6.0, 8.0]).show(ui, |ui| {
                    ctrl_label(ui, label_w, "Position");
                    let pos_text = if f.moving {
                        format!("{} (moving)", f.position)
                    } else {
                        f.position.to_string()
                    };
                    ui.label(egui::RichText::new(pos_text).monospace().size(12.0));
                    ui.end_row();

                    ctrl_label(ui, label_w, "Target");
                    ui.add(egui::DragValue::new(&mut f.target).range(0..=f.max_step).speed(10));
                    if ui.button("Move").clicked() {
                        let _ = handle.cmd_tx.send(ToupCmd::SetFocuserPosition(f.target));
                    }
                    if ui.button("Halt").clicked() {
                        let _ = handle.cmd_tx.send(ToupCmd::FocuserHalt);
                    }
                    ui.end_row();
                });
            });
        }

        // ── ST4 autoguider port: manual pulse for cable/mount testing ───────
        if controls.has_st4 {
            use toupcam::GuideDirection;
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Guide (ST4)").strong().color(pal.accent),
            )
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("toup_st4").num_columns(2).spacing([6.0, 8.0]).show(ui, |ui| {
                    ctrl_label(ui, label_w, "Pulse Length");
                    ui.add(
                        egui::DragValue::new(&mut controls.guide_ms)
                            .range(10..=10_000)
                            .speed(10)
                            .suffix(" ms"),
                    );
                    ui.end_row();

                    ctrl_label(ui, label_w, "Pulse");
                    ui.horizontal(|ui| {
                        for (label, dir) in [
                            ("N", GuideDirection::North),
                            ("S", GuideDirection::South),
                            ("E", GuideDirection::East),
                            ("W", GuideDirection::West),
                        ] {
                            if ui.button(label).clicked() {
                                let _ = handle.cmd_tx.send(ToupCmd::Guide(dir, controls.guide_ms));
                            }
                        }
                        if ui.button("Stop").on_hover_text("Cancel any pulse in progress").clicked() {
                            let _ = handle.cmd_tx.send(ToupCmd::Guide(GuideDirection::Stop, 0));
                        }
                    });
                    ui.end_row();
                });
            });
        }

        // ── Static device identity ───────────────────────────────────────────
        if !controls.info_rows.is_empty() {
            ui.add_space(6.0);
            egui::CollapsingHeader::new(
                egui::RichText::new("Camera Info").strong().color(pal.accent),
            )
            .default_open(false)
            .show(ui, |ui| {
                egui::Grid::new("toup_info").num_columns(2).spacing([6.0, 6.0]).show(ui, |ui| {
                    for (label, value) in &controls.info_rows {
                        ctrl_label(ui, label_w, label);
                        ui.label(
                            egui::RichText::new(value)
                                .monospace().size(12.0).color(pal.text_secondary),
                        );
                        ui.end_row();
                    }
                });
            });
        }
    }

    /// Render the GigE camera's GenICam-derived controls, grouped by the
    /// camera's own categories in collapsible sections, with a substring
    /// filter. Writable floats/ints → sliders, enums → combo, bools →
    /// checkbox, commands → button; read-only features render as telemetry
    /// text. Controls disabled by an `Auto` mode re-enable automatically once
    /// the capture thread pushes a fresh snapshot.
    #[cfg(feature = "gev")]
    fn gev_controls_content(&mut self, ui: &mut egui::Ui) {
        use gev_camera::{GevCmd, GevControlKind};
        let pal = self.pal();
        if !matches!(self.capture_state, CaptureState::Gev { .. }) { return }

        // Filter box — the full GenICam tree can run to hundreds of features.
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.gev_filter)
                    .hint_text("Filter features…")
                    .desired_width(180.0),
            );
            if !self.gev_filter.is_empty() && ui.small_button("✕").clicked() {
                self.gev_filter.clear();
            }
        });
        let filter = self.gev_filter.trim().to_lowercase();
        let filtering = !filter.is_empty();
        let matches = |c: &gev_camera::GevControl| {
            !filtering
                || c.display.to_lowercase().contains(&filter)
                || c.name.to_lowercase().contains(&filter)
        };

        let CaptureState::Gev { ref handle, ref mut controls } = self.capture_state else { return };

        // Categories in first-seen (XML tree) order.
        let mut cats: Vec<String> = Vec::new();
        for c in controls.iter() {
            if !cats.contains(&c.category) { cats.push(c.category.clone()); }
        }

        for cat in &cats {
            if !controls.iter().any(|c| &c.category == cat && matches(c)) {
                continue;
            }
            ui.add_space(2.0);
            let mut header = egui::CollapsingHeader::new(
                egui::RichText::new(cat).strong().color(pal.accent),
            )
            .default_open(gev_category_default_open(cat));
            if filtering {
                header = header.open(Some(true)); // reveal matches; reverts when cleared
            }
            header.show(ui, |ui| {
                ui.spacing_mut().slider_width = 120.0;
                egui::Grid::new(format!("gev_ctrl_{cat}"))
                    .num_columns(3)
                    .spacing([10.0, 6.0])
                    .show(ui, |ui| {
                        for c in controls.iter_mut().filter(|c| &c.category == cat && matches(c)) {
                            let lbl = ui.label(egui::RichText::new(&c.display).color(pal.text_secondary));
                            let mut hover = c.name.clone();
                            if c.needs_restart {
                                hover.push_str("\nChanging this briefly stops and restarts the stream");
                            }
                            lbl.on_hover_text(hover);
                            let unit = c.unit.clone();
                            let unit_lbl = move |ui: &mut egui::Ui| {
                                ui.label(egui::RichText::new(unit).small().color(pal.text_secondary));
                            };
                            let ro_text = |ui: &mut egui::Ui, text: String| {
                                ui.label(egui::RichText::new(text).monospace().color(pal.text_primary));
                            };
                            // `needs_restart` controls (PixelFormat, Width/Height, binning)
                            // read as read-only *while acquiring*, but we apply them by
                            // stopping/restarting the stream — so keep them editable.
                            let enabled = c.writable || c.needs_restart;
                            match &c.kind {
                                GevControlKind::Float if enabled => {
                                    let old = c.fvalue;
                                    let mut v = c.fvalue;
                                    let range = c.fmin..=c.fmax.max(c.fmin + 1.0);
                                    ui.add(egui::Slider::new(&mut v, range).logarithmic(true));
                                    unit_lbl(ui);
                                    if (v - old).abs() > f64::EPSILON {
                                        c.fvalue = v;
                                        let _ = handle.cmd_tx.send(GevCmd::SetFloat(c.name.clone(), v));
                                    }
                                }
                                GevControlKind::Float | GevControlKind::ReadOnly => {
                                    ro_text(ui, fmt_gev_float(c.fvalue));
                                    unit_lbl(ui);
                                }
                                GevControlKind::Integer if enabled => {
                                    let old = c.value;
                                    let mut v = c.value;
                                    ui.add(egui::Slider::new(&mut v, c.min..=c.max.max(c.min + 1)));
                                    unit_lbl(ui);
                                    if v != old {
                                        c.value = v;
                                        let _ = handle.cmd_tx.send(GevCmd::SetInt(c.name.clone(), v));
                                    }
                                }
                                GevControlKind::Integer => {
                                    ro_text(ui, c.value.to_string());
                                    unit_lbl(ui);
                                }
                                GevControlKind::IpV4 if enabled => {
                                    // Octets in human order; `ip_swapped` says how the
                                    // camera packs them into the integer.
                                    let mut oct = if c.ip_swapped {
                                        (c.value as u32).to_le_bytes()
                                    } else {
                                        (c.value as u32).to_be_bytes()
                                    };
                                    let mut edited = false;
                                    let mut active = false;
                                    ui.horizontal(|ui| {
                                        ui.spacing_mut().item_spacing.x = 2.0;
                                        for (k, o) in oct.iter_mut().enumerate() {
                                            if k > 0 {
                                                ui.label(egui::RichText::new(".").monospace());
                                            }
                                            let r = ui.add(egui::DragValue::new(o).speed(1));
                                            edited |= r.changed();
                                            active |= r.has_focus() || r.dragged();
                                        }
                                    });
                                    ui.label("");
                                    if edited {
                                        c.value = if c.ip_swapped {
                                            u32::from_le_bytes(oct)
                                        } else {
                                            u32::from_be_bytes(oct)
                                        } as i64;
                                    }
                                    // Write only once the user is done editing — sending
                                    // per keystroke would set half-typed addresses.
                                    let dirty_id = egui::Id::new(("gev_ip_dirty", &c.name));
                                    let mut dirty: bool =
                                        ui.data(|d| d.get_temp(dirty_id)).unwrap_or(false);
                                    dirty |= edited;
                                    if dirty && !active {
                                        let _ = handle.cmd_tx.send(GevCmd::SetInt(c.name.clone(), c.value));
                                        dirty = false;
                                    }
                                    ui.data_mut(|d| d.insert_temp(dirty_id, dirty));
                                }
                                GevControlKind::IpV4 => {
                                    let o = if c.ip_swapped {
                                        (c.value as u32).to_le_bytes()
                                    } else {
                                        (c.value as u32).to_be_bytes()
                                    };
                                    ro_text(ui, format!("{}.{}.{}.{}", o[0], o[1], o[2], o[3]));
                                    ui.label("");
                                }
                                GevControlKind::MacAddr => {
                                    let b = (c.value as u64).to_be_bytes();
                                    let m: [u8; 6] = if c.ip_swapped {
                                        [b[7], b[6], b[5], b[4], b[3], b[2]]
                                    } else {
                                        [b[2], b[3], b[4], b[5], b[6], b[7]]
                                    };
                                    ro_text(ui, format!(
                                        "{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                                        m[0], m[1], m[2], m[3], m[4], m[5]
                                    ));
                                    ui.label("");
                                }
                                GevControlKind::Enumeration(opts) if enabled => {
                                    let opts = opts.clone();
                                    let idx = c.value as usize;
                                    let cur = opts.get(idx).cloned().unwrap_or_default();
                                    let mut pick: Option<(usize, String)> = None;
                                    egui::ComboBox::from_id_salt(&c.name)
                                        .selected_text(cur)
                                        .show_ui(ui, |ui| {
                                            for (i, o) in opts.iter().enumerate() {
                                                if ui.selectable_label(i == idx, o).clicked() {
                                                    pick = Some((i, o.clone()));
                                                }
                                            }
                                        });
                                    ui.label("");
                                    if let Some((i, sym)) = pick {
                                        c.value = i as i64;
                                        let _ = handle.cmd_tx.send(GevCmd::SetEnum(c.name.clone(), sym));
                                    }
                                }
                                GevControlKind::Enumeration(opts) => {
                                    let cur = opts.get(c.value as usize).cloned().unwrap_or_default();
                                    ro_text(ui, cur);
                                    ui.label("");
                                }
                                GevControlKind::Boolean if enabled => {
                                    let mut on = c.value != 0;
                                    let resp = ui.add(egui::Checkbox::new(&mut on, ""));
                                    ui.label("");
                                    if resp.changed() {
                                        c.value = on as i64;
                                        let _ = handle.cmd_tx.send(GevCmd::SetBool(c.name.clone(), on));
                                    }
                                }
                                GevControlKind::Boolean => {
                                    ro_text(ui, if c.value != 0 { "On".into() } else { "Off".into() });
                                    ui.label("");
                                }
                                GevControlKind::Command => {
                                    if ui.button("Execute").clicked() {
                                        let _ = handle.cmd_tx.send(GevCmd::Execute(c.name.clone()));
                                    }
                                    ui.label("");
                                }
                            }
                            ui.end_row();
                        }
                    });
            });
        }
    }

    /// Render the INDI device's properties: a device picker with connection
    /// state, a capture block (exposure, Single / Live), then the generic
    /// property vectors grouped by the driver's own groups — the same
    /// collapsible-grid pattern as the GigE panel.
    #[cfg(feature = "indi")]
    fn indi_controls_content(&mut self, ui: &mut egui::Ui) {
        use indi_camera::{BlobMode, IndiCmd, IndiProperty, IndiValue, PropState};
        let pal = self.pal();
        if !matches!(self.capture_state, CaptureState::Indi { .. }) { return }

        // Filter box first — self.indi_filter can't be borrowed once
        // capture_state is destructured below.
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.indi_filter)
                    .hint_text("Filter properties…")
                    .desired_width(180.0),
            );
            if !self.indi_filter.is_empty() && ui.small_button("✕").clicked() {
                self.indi_filter.clear();
            }
        });
        let filter = self.indi_filter.trim().to_lowercase();
        let filtering = !filter.is_empty();

        let CaptureState::Indi {
            ref handle, ref mut props, ref mut device, ref mut exposure_s, ref mut live,
        } = self.capture_state else { return };

        if props.is_empty() {
            ui.vertical_centered(|ui| {
                ui.add_space(20.0);
                ui.label(egui::RichText::new("Waiting for INDI properties…").color(pal.text_secondary));
            });
            return;
        }

        // Device row: picker + connection state.
        let mut devices: Vec<String> = Vec::new();
        for p in props.iter() {
            if !devices.contains(&p.device) { devices.push(p.device.clone()); }
        }
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Device").color(pal.text_secondary));
            egui::ComboBox::from_id_salt("indi_device")
                .selected_text(device.clone())
                .show_ui(ui, |ui| {
                    for d in &devices {
                        if ui.selectable_label(*d == *device, d).clicked() {
                            *device = d.clone();
                        }
                    }
                });
            // Item names differ by dialect: INDI uses CONNECT/DISCONNECT,
            // INDIGO (protocol 2.0) CONNECTED/DISCONNECTED — read them from
            // the property definition instead of assuming.
            let conn = props.iter().find(|p| {
                p.device == *device && p.name == indi_camera::PROP_CONNECTION
            });
            let connected = conn.is_some_and(|p| {
                p.elements.iter().any(|el| {
                    (el.name == "CONNECT" || el.name == "CONNECTED")
                        && matches!(el.value, IndiValue::Switch(true))
                })
            });
            let indigo_names = conn
                .is_some_and(|p| p.elements.iter().any(|el| el.name == "CONNECTED"));
            let (on_item, off_item) = if indigo_names {
                ("CONNECTED", "DISCONNECTED")
            } else {
                ("CONNECT", "DISCONNECT")
            };
            if connected {
                ui.label(egui::RichText::new("● connected")
                    .color(pal.status_ok).small());
                if ui.small_button("Disconnect").clicked() {
                    let _ = handle.cmd_tx.send(IndiCmd::SetSwitch {
                        device: device.clone(),
                        property: indi_camera::PROP_CONNECTION.to_string(),
                        values: vec![(on_item.into(), false), (off_item.into(), true)],
                    });
                }
            } else if ui.small_button("Connect").clicked() {
                let _ = handle.cmd_tx.send(IndiCmd::Connect { device: device.clone() });
                let _ = handle.cmd_tx.send(IndiCmd::EnableBlob {
                    device: device.clone(), mode: BlobMode::Also,
                });
            }
        });

        // Capture block — INDI exposures are one-shot; Live re-triggers on
        // each received frame.
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Exposure (s)").color(pal.text_secondary));
            ui.add(egui::DragValue::new(exposure_s).speed(0.1).range(0.001..=3600.0));
            if *live {
                if ui.button("Stop Live").clicked() {
                    let _ = handle.cmd_tx.send(IndiCmd::StopLive);
                    *live = false;
                }
            } else {
                if ui.button("Single").clicked() {
                    let _ = handle.cmd_tx.send(IndiCmd::EnableBlob {
                        device: device.clone(), mode: BlobMode::Also,
                    });
                    let _ = handle.cmd_tx.send(IndiCmd::StartExposure {
                        device: device.clone(), seconds: *exposure_s, live: false,
                    });
                }
                if ui.button("Live").clicked() {
                    let _ = handle.cmd_tx.send(IndiCmd::EnableBlob {
                        device: device.clone(), mode: BlobMode::Also,
                    });
                    let _ = handle.cmd_tx.send(IndiCmd::StartExposure {
                        device: device.clone(), seconds: *exposure_s, live: true,
                    });
                    *live = true;
                }
            }
            let exposing = props.iter().any(|p| {
                p.device == *device
                    && p.name == indi_camera::PROP_EXPOSURE
                    && p.state == PropState::Busy
            });
            if exposing {
                ui.label(egui::RichText::new("exposing…").color(pal.accent).small());
            }
        });

        // Property groups (CONNECTION is handled by the device row above).
        let matches = |p: &IndiProperty| {
            !filtering
                || p.label.to_lowercase().contains(&filter)
                || p.name.to_lowercase().contains(&filter)
                || p.elements.iter().any(|el| el.label.to_lowercase().contains(&filter))
        };
        let mut groups: Vec<String> = Vec::new();
        for p in props.iter().filter(|p| p.device == *device) {
            if !groups.contains(&p.group) { groups.push(p.group.clone()); }
        }

        for group in &groups {
            if !props.iter().any(|p| {
                p.device == *device && &p.group == group
                    && p.name != indi_camera::PROP_CONNECTION && matches(p)
            }) {
                continue;
            }
            ui.add_space(2.0);
            let title = if group.is_empty() { "Other" } else { group.as_str() };
            let mut header = egui::CollapsingHeader::new(
                egui::RichText::new(title).strong().color(pal.accent),
            )
            .default_open(group.contains("Main"));
            if filtering {
                header = header.open(Some(true)); // reveal matches; reverts when cleared
            }
            header.show(ui, |ui| {
                egui::Grid::new(format!("indi_prop_{group}"))
                    .num_columns(3)
                    .spacing([10.0, 6.0])
                    .show(ui, |ui| {
                        for prop in props.iter_mut().filter(|p| {
                            p.device == *device && &p.group == group
                                && p.name != indi_camera::PROP_CONNECTION
                        }) {
                            if !matches(prop) { continue }
                            indi_property_rows(ui, prop, &handle.cmd_tx, &pal);
                        }
                    });
            });
        }
    }

    #[cfg(feature = "svbony")]
    fn draw_control(
        ui: &mut egui::Ui,
        caps: &svbony::ControlCaps,
        cv: &mut (svbony::ControlType, i64, bool),
        cmd_tx: &Sender<camera::CameraCmd>,
        label_w: f32,
        slider_w: f32,
        value_w: f32,
        auto_w: f32,
        pal: &widgets::Palette,
    ) {
        let old_val = cv.1;
        let old_auto = cv.2;

        if !caps.is_writable {
            // Read-only control
            egui::Grid::new(egui::Id::new("ctrl").with(&caps.name))
                .num_columns(2).spacing([4.0, 4.0]).show(ui, |ui| {
                ctrl_label(ui, label_w, &caps.name);
                ui.label(egui::RichText::new(format_control_value(caps.control_type, cv.1))
                    .monospace().size(12.0).color(pal.text_secondary));
                ui.end_row();
            });
        } else if caps.max_value - caps.min_value <= 1 {
            // Boolean toggle
            egui::Grid::new(egui::Id::new("ctrl").with(&caps.name))
                .num_columns(2).spacing([4.0, 4.0]).show(ui, |ui| {
                ctrl_label(ui, label_w, "");
                let mut on = cv.1 != 0;
                if widgets::styled_checkbox(ui, &mut on, &caps.name, pal) {
                    cv.1 = if on { 1 } else { 0 };
                }
                ui.end_row();
            });
        } else {
            let is_exposure = caps.control_type == svbony::ControlType::Exposure;
            egui::Grid::new(egui::Id::new("ctrl").with(&caps.name))
                .num_columns(4).spacing([4.0, 4.0]).show(ui, |ui| {
                // Label
                ctrl_label(ui, label_w, &caps.name);
                // Slider
                let mut v = cv.1 as f32;
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    if is_exposure {
                        let max_us = (caps.max_value as f32).min(10_000_000.0);
                        widgets::styled_slider_log_bare(ui, &mut v, (caps.min_value as f32).max(1.0)..=max_us, pal);
                    } else {
                        widgets::styled_slider_bare(ui, &mut v, caps.min_value as f32..=caps.max_value as f32, pal);
                    }
                });
                cv.1 = v.round() as i64;
                // Value display
                if is_exposure {
                    let mut v_us = cv.1 as f64;
                    let speed = (v_us * 0.005).max(1.0);
                    let dv = egui::DragValue::new(&mut v_us)
                        .range(caps.min_value as f64..=(caps.max_value as f64).min(10_000_000.0))
                        .speed(speed)
                        .custom_formatter(|v, _| {
                            if v >= 1_000_000.0 { format!("{:.2} s", v / 1_000_000.0) }
                            else if v >= 1_000.0 { format!("{:.1} ms", v / 1_000.0) }
                            else { format!("{:.0} µs", v) }
                        })
                        .custom_parser(|s| {
                            let s = s.trim();
                            if let Some(n) = s.strip_suffix("ms").and_then(|n| n.trim().parse::<f64>().ok()) {
                                Some(n * 1_000.0)
                            } else if let Some(n) = s.strip_suffix("µs").or_else(|| s.strip_suffix("us")).and_then(|n| n.trim().parse::<f64>().ok()) {
                                Some(n)
                            } else if let Some(n) = s.strip_suffix('s').and_then(|n| n.trim().parse::<f64>().ok()) {
                                Some(n * 1_000_000.0)
                            } else {
                                s.parse::<f64>().ok()
                            }
                        });
                    ui.add_sized([value_w, 20.0], dv);
                    cv.1 = v_us.round() as i64;
                } else if caps.control_type == svbony::ControlType::TargetTemperature {
                    // SDK stores temperature in tenths of a °C.
                    let mut v_c = cv.1 as f64 / 10.0;
                    let dv = egui::DragValue::new(&mut v_c)
                        .range(caps.min_value as f64 / 10.0..=caps.max_value as f64 / 10.0)
                        .speed(0.1)
                        .fixed_decimals(1)
                        .suffix(" °C");
                    ui.add_sized([value_w, 20.0], dv);
                    cv.1 = (v_c * 10.0).round() as i64;
                } else {
                    let dv = egui::DragValue::new(&mut cv.1)
                        .range(caps.min_value..=caps.max_value);
                    ui.add_sized([value_w, 20.0], dv);
                }
                // Auto checkbox
                if caps.is_auto_supported {
                    ui.allocate_ui(egui::vec2(auto_w, 20.0), |ui| {
                        let mut auto = cv.2;
                        if widgets::styled_checkbox(ui, &mut auto, "Auto", pal) { cv.2 = auto; }
                    });
                }
                ui.end_row();
            });
        }

        if cv.1 != old_val || cv.2 != old_auto {
            let _ = cmd_tx.send(camera::CameraCmd::SetControl(cv.0, cv.1, cv.2));
        }
    }

    #[cfg(feature = "starsolve")]
    fn plate_solve_content(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        // ── Top bar: solve toggle + database + FOV + status + reset ─────────
        ui.horizontal(|ui| {
            // Master switch for the whole pipeline: extraction and solving.
            let was_enabled = self.solve_enabled;
            widgets::tip(ui, "Run centroid extraction and plate solving on live frames.\nOff: frames skip the solver thread entirely, freeing the CPU; centroid and star overlays clear.", |ui| {
                widgets::styled_checkbox(ui, &mut self.solve_enabled, "Solve", &pal);
            });
            if was_enabled && !self.solve_enabled {
                // Nothing produced while off should linger: stale centroids
                // would otherwise keep being drawn over every new frame, and a
                // stale lock would keep seeding FOV/attitude hints.
                self.last_solve = None;
                self.last_centroids.clear();
                self.centroid_count = 0;
                self.overlay_items.clear();
                #[cfg(feature = "focus")]
                {
                    self.focus_last = None;
                }
            }
            // Overlay switches live beside the pipeline that produces them.
            widgets::tip(ui, "Ellipses at extracted star centroids, cyan (faint) to yellow (bright)", |ui| {
                widgets::styled_checkbox(ui, &mut self.show_centroids, "Centroids", &pal);
            });
            widgets::tip(ui, "Cross-hairs on centroids the plate solve matched to catalog stars", |ui| {
                widgets::styled_checkbox(ui, &mut self.show_matched_stars, "Matched", &pal);
            });
            widgets::tip(ui, "Labels for named bright stars in the solved field (needs Centroids and Matched)", |ui| {
                widgets::styled_checkbox(ui, &mut self.show_star_names, "Names", &pal);
            });
            ui.separator();

            // Database
            if self.gen_rx.is_some() {
                ui.add(egui::Spinner::new().size(14.0));
                let secs = self.gen_started.map_or(0.0, |t| t.elapsed().as_secs_f32());
                ui.label(egui::RichText::new(format!("Building star database… {:.0}s", secs))
                    .color(pal.status_warn));
            } else if self.solver_db.is_none() {
                if widgets::tip(ui, "Load a tetra3 pattern database (.bin)", |ui| widgets::styled_button(ui, "Load Database...", &pal)) {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Database", &["bin"])
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
                // Offer to build the default database from the bundled catalog.
                if self.solver_catalog_path.is_some()
                    && widgets::tip(ui, "Generate the default 5–50° database from the bundled Gaia catalog (one time, cached)", |ui| {
                        widgets::styled_button(ui, "Build default DB", &pal)
                    })
                {
                    self.start_solver_generation();
                }
            } else {
                ui.label(egui::RichText::new("DB").color(pal.status_ok));
                if widgets::styled_button(ui, "Unload", &pal) {
                    self.solver_db = None;
                    self.last_solve = None;
                }
            }

            ui.separator();

            // Camera model
            if self.camera_model.is_none() {
                if widgets::tip(ui, "Load a camera model (.bin): focal length and distortion from a prior calibration", |ui| widgets::styled_button(ui, "Load Calib...", &pal)) {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Camera Model", &["bin"])
                        .pick_file()
                    {
                        match tetra3::CameraModel::load_from_file(&path) {
                            Ok(cam) => {
                                self.add_log(LogEntry::info(format!(
                                    "Camera model: f={:.1}px, {}x{}, FOV {:.2}°",
                                    cam.focal_length_px, cam.image_width, cam.image_height, cam.fov_deg(),
                                )));
                                self.fov_estimate_deg = cam.fov_deg() as f32;
                                self.camera_model = Some(cam);
                                self.camera_model_path = Some(path);
                            }
                            Err(e) => self.add_log(LogEntry::error(format!("Camera model load failed: {}", e))),
                        }
                    }
                }
            } else {
                ui.label(egui::RichText::new("Cal").color(pal.status_ok));
                if widgets::styled_button(ui, "Unload", &pal) {
                    self.camera_model = None;
                    self.camera_model_path = None;
                }
            }

            ui.separator();

            // FOV
            ui.label("FOV:");
            ui.add(egui::DragValue::new(&mut self.fov_estimate_deg)
                .range(1.0..=60.0).speed(0.5).suffix("°").fixed_decimals(1))
                .on_hover_text("Estimated horizontal field of view. Bounds the pattern search until the first solve locks, then tracks the solved value.");

            ui.separator();

            // Solve status
            if self.solver_db.is_some() {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(75.0, 18.0), egui::Sense::hover());
                let (text, color) = if !self.solve_enabled {
                    ("Off", pal.text_secondary)
                } else if self.solve_busy {
                    ("Solving...", pal.status_warn)
                } else if self.last_solve.as_ref().is_some_and(|s| s.is_ok()) {
                    ("Locked", pal.status_ok)
                } else {
                    ("Searching...", pal.text_secondary)
                };
                ui.painter().text(rect.left_center(), egui::Align2::LEFT_CENTER, text, egui::FontId::proportional(13.0), color);
            }

            ui.separator();

            // Centroid count (always shown, even when overlay is off)
            if self.centroid_count > 0 {
                ui.label(egui::RichText::new(format!("{} stars", self.centroid_count)).color(pal.text_secondary));
            }

            // Reset defaults (far right)
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if widgets::tip(ui, "Restore the default extraction parameters", |ui| widgets::styled_button(ui, "Reset", &pal)) {
                    self.centroid_config = tetra3::CentroidExtractionConfig::default();
                }
            });
        });

        // ── Loaded database details ─────────────────────────────────────────
        if let Some(db) = self.solver_db.as_ref() {
            let p = &db.props;
            let name = self
                .solver_db_path
                .as_ref()
                .and_then(|path| path.file_name())
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| "database".to_string());

            ui.add_space(3.0);
            egui::Frame::new()
                .fill(pal.bg_surface)
                .stroke(egui::Stroke::new(1.0, pal.section_border))
                .corner_radius(egui::CornerRadius::same(6))
                .inner_margin(egui::Margin { left: 8, right: 8, top: 5, bottom: 5 })
                .show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    let dim = |ui: &mut egui::Ui, s: String| {
                        ui.label(egui::RichText::new(s).size(12.0).color(pal.text_secondary));
                    };
                    let sep = |ui: &mut egui::Ui| {
                        ui.label(egui::RichText::new("•").size(12.0).color(pal.border));
                    };
                    ui.horizontal_wrapped(|ui| {
                        ui.label(egui::RichText::new(&name).size(12.0).strong().color(pal.text_primary));
                        sep(ui);
                        dim(ui, format!("{} patterns", p.num_patterns));
                        sep(ui);
                        dim(ui, format!("{} stars", db.star_vectors.len()));
                        sep(ui);
                        dim(ui, format!("FOV {:.1}–{:.1}°", p.min_fov_rad.to_degrees(), p.max_fov_rad.to_degrees()));
                        sep(ui);
                        dim(ui, format!("mag ≤ {:.1}", p.star_max_magnitude));
                        sep(ui);
                        dim(ui, format!("err {:.4} / {} bins", p.pattern_max_error, p.pattern_bins));
                        sep(ui);
                        dim(ui, format!("verify {}/FOV", p.verification_stars_per_fov));
                        sep(ui);
                        dim(ui, format!("epoch {} · PM {:.0}", p.epoch_equinox, p.epoch_proper_motion_year));
                    });
                    if let Some(cam) = self.camera_model.as_ref() {
                        ui.horizontal_wrapped(|ui| {
                            ui.label(egui::RichText::new("calib").size(12.0).strong().color(pal.text_primary));
                            sep(ui);
                            dim(ui, format!("f = {:.1} px", cam.focal_length_px));
                            sep(ui);
                            dim(ui, format!("{}×{}", cam.image_width, cam.image_height));
                            sep(ui);
                            dim(ui, format!("FOV {:.2}°", cam.fov_deg()));
                        });
                    }
                });
        }

        // ── Centroid parameters (compact horizontal grid) ───────────────────
        ui.add_space(2.0);
        let total_w = ui.available_width();
        let col_w = (total_w / 3.0 - 4.0).max(100.0);
        let label_w = 92.0;
        let slider_w = (col_w - label_w - 8.0).max(40.0);

        // Name on the left, value right-aligned in a mono column so the numbers
        // line up into a scannable column directly before each slider.
        let param_label = |ui: &mut egui::Ui, name: &str, value: &str| {
            ui.allocate_ui(egui::vec2(label_w, 20.0), |ui| {
                ui.set_min_width(label_w);
                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    ui.label(egui::RichText::new(name).size(12.0).color(pal.text_secondary));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(egui::RichText::new(value).size(12.0).monospace().color(pal.text_primary));
                    });
                });
            });
        };

        // Row 1
        ui.horizontal(|ui| {
            widgets::tip(ui, "Detection threshold in units of the local noise σ; higher keeps only brighter stars", |ui| {
                param_label(ui, "Pix σ", &format!("{:.1}", self.centroid_config.sigma_threshold));
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(ui, &mut self.centroid_config.sigma_threshold, 1.0..=20.0, &pal);
                });
            });

            let mut v = self.centroid_config.min_pixels as f32;
            widgets::tip(ui, "Smallest blob (in pixels) accepted as a star; rejects hot pixels", |ui| {
                param_label(ui, "Min px", &format!("{}", self.centroid_config.min_pixels));
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(ui, &mut v, 1.0..=50.0, &pal);
                });
            });
            self.centroid_config.min_pixels = v.round() as usize;

            let mut v = self.centroid_config.max_pixels as f32;
            widgets::tip(ui, "Largest blob accepted; rejects saturated stars, planets and glare (log-spaced slider)", |ui| {
                param_label(ui, "Max px", &format!("{}", self.centroid_config.max_pixels));
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_log_bare(ui, &mut v, 10.0..=100000.0, &pal);
                });
            });
            self.centroid_config.max_pixels = v as usize;
        });

        // Row 2
        ui.horizontal(|ui| {
            let mut v = self.centroid_config.max_centroids.unwrap_or(0) as f32;
            let val = if self.centroid_config.max_centroids.is_none() { "all".into() } else { format!("{}", v.round() as usize) };
            widgets::tip(ui, "Keep only the N brightest centroids for solving; 0 = all", |ui| {
                param_label(ui, "Stars", &val);
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(ui, &mut v, 0.0..=500.0, &pal);
                });
            });
            self.centroid_config.max_centroids = if (v as usize) == 0 { None } else { Some(v.round() as usize) };

            let mut v = self.centroid_config.local_bg_block_size.unwrap_or(0) as f32;
            let val = if self.centroid_config.local_bg_block_size.is_none() { "global".into() } else { format!("{} px", v.round() as u32) };
            widgets::tip(ui, "Local background grid cell size in pixels; 0 = one global background level", |ui| {
                param_label(ui, "BG", &val);
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(ui, &mut v, 0.0..=256.0, &pal);
                });
            });
            self.centroid_config.local_bg_block_size = if (v as u32) == 0 { None } else { Some(v.round() as u32) };

            let mut v = self.centroid_config.max_elongation.unwrap_or(0.0);
            let val = if self.centroid_config.max_elongation.is_none() { "off".into() } else { format!("{:.1}", v) };
            widgets::tip(ui, "Reject blobs whose major/minor axis ratio exceeds this (trails, close doubles); below 0.5 = off", |ui| {
                param_label(ui, "Elong", &val);
                ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                    widgets::styled_slider_bare(ui, &mut v, 0.0..=10.0, &pal);
                });
            });
            self.centroid_config.max_elongation = if v < 0.5 { None } else { Some(v) };
        });

        // Row 3
        ui.horizontal(|ui| {
            let mut v = self.centroid_config.matched_filter_sigma.unwrap_or(0.0);
            let val = if self.centroid_config.matched_filter_sigma.is_none() { "off".into() } else { format!("{:.1}", v) };
            widgets::tip(ui, "Matched-filter Gaussian σ (pixels) applied before detection; 0 = off. Not used in tracking mode.", |ui| {
                param_label(ui, "Blur σ", &val);
                // The fast path has no matched filter — disable rather than hide,
                // so the setting visibly survives round-trips through tracking mode.
                ui.add_enabled_ui(!self.tracking_mode, |ui| {
                    ui.allocate_ui(egui::vec2(slider_w, 20.0), |ui| {
                        widgets::styled_slider_bare(ui, &mut v, 0.0..=5.0, &pal);
                    });
                });
            });
            if !self.tracking_mode {
                self.centroid_config.matched_filter_sigma = if v < 0.1 { None } else { Some(v) };
            }

            ui.add_space(12.0);
            widgets::tip(ui, "Single-pass fast extractor for frame-to-frame tracking: much faster, slightly less precise, no matched filter", |ui| {
                widgets::styled_checkbox(ui, &mut self.tracking_mode, "Tracking mode", &pal);
            });
            ui.label(
                egui::RichText::new("single-pass fast extraction")
                    .size(11.0)
                    .color(pal.text_secondary),
            );
        });

        // ── Solve results ───────────────────────────────────────────────────
        if let Some(ref result) = self.last_solve {
            ui.add_space(2.0);
            ui.separator();
            ui.add_space(2.0);
            match result {
                Ok(sol) => {
                    ui.horizontal_wrapped(|ui| {
                        let mono = egui::FontId::monospace(12.0);
                        let dim = pal.text_secondary;
                        let sp = 10.0;
                        let crval = sol.crval_rad;
                        ui.label(egui::RichText::new("RA").color(dim));
                        ui.label(egui::RichText::new(format!("{:.4}°", crval[0].to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Dec").color(dim));
                        ui.label(egui::RichText::new(format!("{:.4}°", crval[1].to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("FOV").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}°", sol.fov_rad.to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        let ifov = sol.fov_rad.to_degrees() as f64
                            / sol.camera_model.image_width.max(1) as f64
                            * 3600.0;
                        ui.label(egui::RichText::new("IFOV").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}\"/px", ifov)).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Rot").color(dim));
                        ui.label(egui::RichText::new(format!("{:.2}°", sol.theta_rad.to_degrees())).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Stars").color(dim));
                        ui.label(egui::RichText::new(format!("{}", sol.num_matches)).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("RMSE").color(dim));
                        ui.label(egui::RichText::new(format!("{:.1}\"", sol.rmse_rad.to_degrees() * 3600.0)).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Extract").color(dim));
                        ui.label(egui::RichText::new(format!("{:.0}ms", self.centroid_time_ms)).font(mono.clone()));
                        ui.add_space(sp);
                        ui.label(egui::RichText::new("Solve").color(dim));
                        ui.label(egui::RichText::new(format!("{:.0}ms", sol.solve_time_ms)).font(mono));
                    });
                }
                Err(fail) => {
                    let msg = match fail.status {
                        tetra3::SolveStatus::NoMatch => "No match",
                        tetra3::SolveStatus::Timeout => "Timed out",
                        tetra3::SolveStatus::TooFew => "Too few stars",
                        tetra3::SolveStatus::InvalidConfig => "Invalid solver config",
                    };
                    ui.label(egui::RichText::new(msg).color(pal.status_err));
                }
            }
        }
    }

    /// The Focus tab: the current HFR in digits large enough to read from the
    /// telescope, the best value since the last reset, and a trend of the
    /// last few hundred frames. Focusing is watching a number move as the
    /// knob turns, so the history and the best-so-far matter as much as the
    /// current value.
    #[cfg(feature = "focus")]
    fn focus_content(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        let dim = pal.text_secondary;
        let good = pal.status_ok;
        let warn = pal.status_warn;

        // ── Top bar ─────────────────────────────────────────────────────────
        ui.horizontal(|ui| {
            if widgets::tip(ui, "Forget the trend and the best-so-far. Do this after a large focuser move or a filter change.", |ui| {
                widgets::styled_button(ui, "Reset", &pal)
            }) {
                self.focus_history.reset();
            }
            ui.separator();
            widgets::tip(ui, "Measure only stars inside the zoom ROI (draw one by dragging on the image). Off, or with no ROI drawn: the brightest stars anywhere in the frame.", |ui| {
                widgets::styled_checkbox(ui, &mut self.focus_use_roi, "ROI only", &pal);
            });
            widgets::tip(ui, "Write each measured star's HFR next to it on the image. A gradient of values across the frame shows tilt or field curvature.", |ui| {
                widgets::styled_checkbox(ui, &mut self.focus_show_labels, "Label stars", &pal);
            });
            ui.separator();
            widgets::tip(ui, "HFR: half flux radius of the brightest stars, smaller is better. Sharpness: whole-frame contrast, larger is better; useful far from focus when no stars are detected.", |ui| {
                widgets::combo_box(ui, "focus_plot", "Plot", &mut self.focus_plot,
                    &[(FocusPlot::Hfr, "HFR"), (FocusPlot::Sharpness, "Sharpness")], &pal);
            });
            ui.separator();
            // Pipeline state, so a silent readout explains itself.
            let (text, color) = if !self.solve_enabled {
                ("Solve is off: enable it in the Plate Solve tab", warn)
            } else if self.focus_use_roi && self.image_viewer.roi_rect.is_none() {
                ("ROI only: drag a region on the image", warn)
            } else if self.focus_last.as_ref().is_some_and(|s| s.hfr_px.is_none()) {
                ("No measurable stars: too faint, saturated, or elongated", dim)
            } else {
                ("", dim)
            };
            if !text.is_empty() {
                ui.label(egui::RichText::new(text).color(color));
            }
        });
        ui.add_space(4.0);

        let latest = self.focus_history.latest().copied();
        let best = self.focus_history.best().copied();
        // Arcseconds per pixel from the last solve, when locked.
        let arcsec_per_px: Option<f64> = self.last_solve.as_ref().and_then(|s| s.as_ref().ok()).map(|sol| {
            sol.fov_rad.to_degrees() as f64 / sol.camera_model.image_width.max(1) as f64 * 3600.0
        });

        ui.horizontal_top(|ui| {
            // ── Readout column ──────────────────────────────────────────────
            ui.vertical(|ui| {
                ui.set_width(210.0);
                let big = egui::FontId::monospace(44.0);
                let mid = egui::FontId::monospace(20.0);
                let small = egui::FontId::proportional(12.0);

                ui.label(egui::RichText::new("HFR").color(dim).font(small.clone()));
                let hfr = latest.and_then(|p| p.hfr_px);
                let hfr_txt = hfr.map_or("--.--".to_string(), |h| format!("{:.2}", h));
                let hfr_color = match (hfr, best.and_then(|b| b.hfr_px)) {
                    (Some(h), Some(b)) if h <= b * 1.02 => good,
                    (Some(_), _) => pal.text_primary,
                    (None, _) => dim,
                };
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new(hfr_txt).font(big).color(hfr_color));
                    ui.vertical(|ui| {
                        ui.add_space(14.0);
                        ui.label(egui::RichText::new("px").color(dim).font(small.clone()));
                        if let (Some(h), Some(spp)) = (hfr, arcsec_per_px) {
                            ui.label(egui::RichText::new(format!("{:.2}\"", h as f64 * spp)).color(dim).font(small.clone()));
                        }
                    });
                });

                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("Best").color(dim).font(small.clone()));
                    let best_txt = best.and_then(|b| b.hfr_px).map_or("--.--".to_string(), |h| format!("{:.2}", h));
                    ui.label(egui::RichText::new(best_txt).font(mid).color(good));
                    if let Some(p) = best.and_then(|b| b.focuser_pos) {
                        ui.label(egui::RichText::new(format!("@ {}", p)).color(dim).font(small.clone()));
                    }
                });

                ui.add_space(6.0);
                let stat = |ui: &mut egui::Ui, k: &str, v: String| {
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(k).color(dim).font(small.clone()));
                        ui.label(egui::RichText::new(v).font(egui::FontId::monospace(12.0)));
                    });
                };
                let (n, cand) = self.focus_last.as_ref().map_or((0, 0), |s| (s.stars.len(), s.candidates));
                stat(ui, "Stars", format!("{} of {}", n, cand));
                stat(ui, "Sharpness", latest.map_or("--".to_string(), |p| format!("{:.2}", p.sharpness)));
                if let Some(p) = latest.and_then(|p| p.focuser_pos) {
                    stat(ui, "Focuser", format!("{}", p));
                }
            });

            ui.separator();

            // ── Trend plot ──────────────────────────────────────────────────
            let (points, best_line, y_label): (Vec<[f64; 2]>, Option<f64>, &str) = match self.focus_plot {
                FocusPlot::Hfr => (
                    self.focus_history.iter().filter_map(|p| p.hfr_px.map(|h| [p.t, h as f64])).collect(),
                    best.and_then(|b| b.hfr_px).map(|h| h as f64),
                    "HFR (px)",
                ),
                FocusPlot::Sharpness => (
                    self.focus_history.iter().map(|p| [p.t, p.sharpness as f64]).collect(),
                    None,
                    "Sharpness",
                ),
            };
            let plot_height = ui.available_height().max(60.0);
            egui_plot::Plot::new("focus_trend")
                .height(plot_height)
                .y_axis_label(y_label)
                .x_axis_label("Time (s)")
                .show_axes([true, true])
                .allow_drag(false).allow_zoom(false).allow_scroll(false).allow_boxed_zoom(false)
                .show_grid([false, true])
                .include_y(0.0)
                .set_margin_fraction(egui::vec2(0.02, 0.1))
                .show(ui, |plot_ui| {
                    if let Some(b) = best_line {
                        plot_ui.hline(egui_plot::HLine::new("best", b).color(good).width(1.0).style(egui_plot::LineStyle::dashed_dense()));
                    }
                    if !points.is_empty() {
                        plot_ui.line(egui_plot::Line::new(y_label, egui_plot::PlotPoints::from(points)).color(pal.plot_line).width(1.5));
                    }
                });
        });
    }

    fn log_content(&mut self, ui: &mut egui::Ui) {
        let pal = self.pal();
        // Showing the tab acknowledges everything in it.
        self.log_seen = self.log.len();
        if widgets::styled_button(ui, "Clear", &pal) {
            self.log.clear();
        }
        ui.add_space(4.0);

        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for entry in self.log.iter().rev() {
                    let color = match entry.level {
                        LogLevel::Info => pal.text_secondary,
                        LogLevel::Warn => pal.status_warn,
                        LogLevel::Error => pal.status_err,
                    };
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(&entry.timestamp).monospace().size(11.0).color(pal.text_secondary));
                        ui.label(egui::RichText::new(&entry.message).size(12.0).color(color));
                    });
                }
            });
    }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Categories worth showing expanded by default; the rest (device info,
/// transport layer, …) start collapsed.
#[cfg(feature = "gev")]
fn gev_category_default_open(cat: &str) -> bool {
    let c = cat.to_ascii_lowercase();
    c.contains("acquisition") || c.contains("image") || c.contains("analog") || c.contains("gain")
}

/// Format a GenICam float for telemetry display: integers plain, otherwise
/// precision scaled to magnitude.
#[cfg(any(feature = "gev", feature = "indi"))]
fn fmt_gev_float(v: f64) -> String {
    let a = v.abs();
    if v == v.trunc() && a < 1e7 {
        format!("{v:.0}")
    } else if a >= 1000.0 {
        format!("{v:.1}")
    } else if a >= 1.0 {
        format!("{v:.3}")
    } else {
        format!("{v:.5}")
    }
}

/// Render the grid rows for one INDI property vector (label | widget | state).
/// Numbers and text commit the *whole* vector at end-of-edit — INDI expects
/// full-vector writes, and per-tick sends would spam the wire. Switches send
/// immediately; a OneOfMany switch vector renders as a single combo row.
#[cfg(feature = "indi")]
fn indi_property_rows(
    ui: &mut egui::Ui,
    prop: &mut indi_camera::IndiProperty,
    cmd_tx: &Sender<indi_camera::IndiCmd>,
    pal: &widgets::Palette,
) {
    use indi_camera::{IndiCmd, IndiValue, PropPerm, PropState, SwitchRule};

    let writable = prop.perm != PropPerm::Ro;
    let state_dot = |ui: &mut egui::Ui, state: PropState| {
        let (color, name) = match state {
            PropState::Idle => (pal.text_secondary, "idle"),
            PropState::Ok => (pal.status_ok, "ok"),
            PropState::Busy => (pal.status_warn, "busy"),
            PropState::Alert => (pal.status_err, "alert"),
        };
        ui.label(egui::RichText::new("●").color(color).small()).on_hover_text(name);
    };
    let ro_text = |ui: &mut egui::Ui, text: String| {
        ui.label(egui::RichText::new(text).monospace().color(pal.text_primary));
    };

    // Vectors are homogeneous by protocol; the first element sets the type.
    let Some(first) = prop.elements.first() else { return };

    // OneOfMany switch vectors are radio groups → single combo row.
    if matches!(first.value, IndiValue::Switch(_)) && prop.rule == Some(SwitchRule::OneOfMany) {
        ui.label(egui::RichText::new(&prop.label).color(pal.text_secondary))
            .on_hover_text(&prop.name);
        let on_idx = prop.elements.iter()
            .position(|el| matches!(el.value, IndiValue::Switch(true)));
        let cur = on_idx.map(|i| prop.elements[i].label.clone()).unwrap_or_default();
        if writable {
            let mut pick: Option<usize> = None;
            egui::ComboBox::from_id_salt(("indi_sw", &prop.device, &prop.name))
                .selected_text(cur)
                .show_ui(ui, |ui| {
                    for (i, el) in prop.elements.iter().enumerate() {
                        if ui.selectable_label(Some(i) == on_idx, &el.label).clicked() {
                            pick = Some(i);
                        }
                    }
                });
            if let Some(i) = pick {
                for (j, el) in prop.elements.iter_mut().enumerate() {
                    el.value = IndiValue::Switch(i == j);
                }
                let _ = cmd_tx.send(IndiCmd::SetSwitch {
                    device: prop.device.clone(),
                    property: prop.name.clone(),
                    values: indi_switch_payload(prop),
                });
            }
        } else {
            ro_text(ui, cur);
        }
        state_dot(ui, prop.state);
        ui.end_row();
        return;
    }

    let n = prop.elements.len();
    let (pdev, pname, plabel, pstate) =
        (prop.device.clone(), prop.name.clone(), prop.label.clone(), prop.state);
    let mut commit_numbers = false;
    let mut commit_texts = false;
    let mut commit_switches = false;

    for idx in 0..n {
        let el_label = {
            let el = &prop.elements[idx];
            if n == 1 || el.label == plabel {
                plabel.clone()
            } else {
                format!("{} · {}", plabel, el.label)
            }
        };
        let hover = format!("{}.{}", pname, prop.elements[idx].name);
        ui.label(egui::RichText::new(el_label).color(pal.text_secondary)).on_hover_text(hover);

        let el = &mut prop.elements[idx];
        match &mut el.value {
            IndiValue::Number { value, min, max, step, .. } => {
                if writable {
                    let id = egui::Id::new(("indi_num", &pdev, &pname, &el.name));
                    let speed = if *step > 0.0 {
                        *step
                    } else if *max > *min {
                        (*max - *min) / 1000.0
                    } else {
                        0.1
                    };
                    let mut dv = egui::DragValue::new(value).speed(speed);
                    if *max > *min { dv = dv.range(*min..=*max); }
                    let r = ui.add(dv);
                    let mut dirty: bool = ui.data(|d| d.get_temp(id)).unwrap_or(false);
                    dirty |= r.changed();
                    let active = r.dragged() || r.has_focus();
                    if dirty && !active {
                        commit_numbers = true;
                        dirty = false;
                    }
                    ui.data_mut(|d| d.insert_temp(id, dirty));
                } else {
                    ro_text(ui, fmt_gev_float(*value));
                }
            }
            IndiValue::Switch(on) => {
                if writable {
                    let mut v = *on;
                    if ui.add(egui::Checkbox::new(&mut v, "")).changed() {
                        *on = v;
                        commit_switches = true;
                    }
                } else {
                    ro_text(ui, if *on { "On".into() } else { "Off".into() });
                }
            }
            IndiValue::Text(t) => {
                if writable {
                    // Edit a temp buffer while focused; commit on Enter so
                    // half-typed values never hit the wire.
                    let id = egui::Id::new(("indi_text", &pdev, &pname, &el.name));
                    let mut buf: String = ui.data(|d| d.get_temp(id)).unwrap_or_else(|| t.clone());
                    let r = ui.add(egui::TextEdit::singleline(&mut buf).desired_width(160.0));
                    if r.lost_focus() {
                        if ui.input(|i| i.key_pressed(egui::Key::Enter)) && buf != *t {
                            *t = buf;
                            commit_texts = true;
                        }
                        ui.data_mut(|d| d.remove::<String>(id));
                    } else if r.has_focus() {
                        ui.data_mut(|d| d.insert_temp(id, buf));
                    }
                } else {
                    ro_text(ui, t.clone());
                }
            }
            IndiValue::Light(s) => state_dot(ui, *s),
            IndiValue::Blob { format, size } => {
                let text = if *size > 0 {
                    format!("{} ({} bytes)", format, size)
                } else if format.is_empty() {
                    "—".to_string()
                } else {
                    format.clone()
                };
                ro_text(ui, text);
            }
        }
        if idx == 0 { state_dot(ui, pstate); } else { ui.label(""); }
        ui.end_row();
    }

    if commit_numbers {
        let values = prop.elements.iter().filter_map(|el| match el.value {
            IndiValue::Number { value, .. } => Some((el.name.clone(), value)),
            _ => None,
        }).collect();
        let _ = cmd_tx.send(IndiCmd::SetNumber {
            device: pdev.clone(), property: pname.clone(), values,
        });
    }
    if commit_texts {
        let values = prop.elements.iter().filter_map(|el| match &el.value {
            IndiValue::Text(t) => Some((el.name.clone(), t.clone())),
            _ => None,
        }).collect();
        let _ = cmd_tx.send(IndiCmd::SetText {
            device: pdev.clone(), property: pname.clone(), values,
        });
    }
    if commit_switches {
        let _ = cmd_tx.send(IndiCmd::SetSwitch {
            device: pdev, property: pname, values: indi_switch_payload(prop),
        });
    }
}

/// All switch elements of a property as (name, on) pairs — INDI switch writes
/// send the full vector so the driver can apply its rule atomically.
#[cfg(feature = "indi")]
fn indi_switch_payload(prop: &indi_camera::IndiProperty) -> Vec<(String, bool)> {
    prop.elements.iter().filter_map(|el| match el.value {
        indi_camera::IndiValue::Switch(on) => Some((el.name.clone(), on)),
        _ => None,
    }).collect()
}

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

/// Semibold, letter-spaced uppercase eyebrow text — the section headers'
/// voice. Tracking gives the small caps room to breathe instead of reading
/// as a cramped run.
fn eyebrow_job(title: &str, pal: &widgets::Palette) -> egui::text::LayoutJob {
    let mut job = egui::text::LayoutJob::default();
    job.append(
        &title.to_uppercase(),
        0.0,
        egui::TextFormat {
            font_id: egui::FontId::new(11.0, strong_family()),
            color: pal.section_header_text,
            extra_letter_spacing: 1.2,
            ..Default::default()
        },
    );
    job
}

/// Eyebrow labeling one backend's group in the Connect dialog.
fn connect_group_header(ui: &mut egui::Ui, title: &str, pal: &widgets::Palette) {
    ui.label(eyebrow_job(title, pal));
}

fn section(ui: &mut egui::Ui, title: &str, pal: &widgets::Palette, content: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(pal.section_body_fill)
        .stroke(egui::Stroke::new(1.0, pal.section_border))
        .corner_radius(egui::CornerRadius::same(8))
        .inner_margin(egui::Margin { left: 1, right: 1, top: 1, bottom: 1 })
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            let header_h = 24.0;
            let header_rect = {
                let avail = ui.available_rect_before_wrap();
                let rect = egui::Rect::from_min_size(avail.min, egui::vec2(avail.width(), header_h));
                ui.painter().rect_filled(rect, egui::CornerRadius { nw: 7, ne: 7, sw: 0, se: 0 }, pal.section_header_fill);
                ui.painter().hline(rect.x_range(), rect.max.y, egui::Stroke::new(1.0, pal.section_border));
                rect
            };
            let galley = ui.painter().layout_job(eyebrow_job(title, pal));
            let gy = header_rect.center().y - galley.size().y / 2.0;
            ui.painter().galley(egui::pos2(header_rect.min.x + 10.0, gy), galley, pal.section_header_text);
            ui.allocate_space(egui::vec2(0.0, header_h));
            egui::Frame::new()
                .inner_margin(egui::Margin { left: 9, right: 9, top: 8, bottom: 8 })
                .show(ui, |ui| { ui.spacing_mut().item_spacing.y = 5.0; content(ui); });
        });
}

#[cfg(any(feature = "svbony", feature = "toupcam"))]
fn ctrl_label(ui: &mut egui::Ui, width: f32, text: &str) {
    ui.allocate_ui(egui::vec2(width, ui.spacing().interact_size.y), |ui| {
        ui.set_max_width(width);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center).with_main_wrap(true), |ui| {
            ui.label(egui::RichText::new(text).size(13.0));
        });
    });
}

/// One row of the Statistics panel: a dim, right-aligned label and a monospace
/// value — the same dim-label + mono-value register as the Plate Solve readout.
fn stat_row(ui: &mut egui::Ui, label_width: f32, label: &str, value: &str, pal: &widgets::Palette) {
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        ui.set_width(label_width);
        ui.label(egui::RichText::new(label).color(pal.text_secondary));
    });
    ui.label(egui::RichText::new(value).monospace().color(pal.text_primary));
    ui.end_row();
}

// ── Keyboard shortcuts ──────────────────────────────────────────────────────
// One definition each, shared by the handler in `ui()` and the menu hints, so
// the two can never disagree. `format_shortcut` renders COMMAND as ⌘ on macOS
// and Ctrl elsewhere.

const SC_OPEN: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::O);
const SC_CONNECT: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::K);
const SC_QUIT: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::COMMAND, egui::Key::Q);
const SC_PLAY: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::NONE, egui::Key::Space);
const SC_RECORD: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::NONE, egui::Key::R);
const SC_SIDE_PANEL: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::NONE, egui::Key::S);
const SC_BOTTOM_PANEL: egui::KeyboardShortcut = egui::KeyboardShortcut::new(egui::Modifiers::NONE, egui::Key::B);

// ── Main update loop ────────────────────────────────────────────────────────

impl eframe::App for ViewerApp {
    fn on_exit(&mut self) {
        // Finish any in-flight recording first: the writer thread is joined
        // so the FITS file is flushed and closed before the process exits.
        if self.rec_tx.is_some() || self.rec_join.is_some() {
            self.stop_recording();
        }
        self.stop_capture();
        #[cfg(feature = "starsolve")]
        self.save_config();
        self.ui_config().save();
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll_frame();
        self.poll_log();
        #[cfg(feature = "gev")]
        self.update_gev_rate();
        self.poll_fits_load();
        self.poll_bg();
        #[cfg(feature = "starsolve")]
        self.poll_solver_generation();
        self.apply_theme(&ctx);

        if QUIT_REQUESTED.load(Ordering::SeqCst) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        // Remember where the window is, for next launch. Read every frame
        // rather than at exit because the viewport info is gone by then.
        if let (Some(inner), Some(outer)) = ctx.input(|i| (i.viewport().inner_rect, i.viewport().outer_rect)) {
            if inner.width() > 100.0 && inner.height() > 100.0 && outer.min.x.is_finite() && outer.min.y.is_finite() {
                self.window_geometry = Some(([inner.width(), inner.height()], [outer.min.x, outer.min.y]));
            }
        }

        if self.pending_fits_load.is_some() || self.pending_bg.is_some() { ctx.request_repaint(); }
        // While capturing, frames wake the UI instantly via the pump thread;
        // this slow tick only keeps telemetry, logs, and INDI property updates
        // flowing between frames.
        if self.capture_running { ctx.request_repaint_after(std::time::Duration::from_millis(200)); }
        // Keep repainting while the database builds so the elapsed timer ticks
        // and completion is detected promptly.
        #[cfg(feature = "starsolve")]
        if self.gen_rx.is_some() { ctx.request_repaint(); }

        let pal = self.pal();

        // One-time prompt offering to build the default database on first run.
        #[cfg(feature = "starsolve")]
        if self.show_build_prompt {
            let (mut build, mut later) = (false, false);
            egui::Window::new("Star database")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .show(&ctx, |ui| {
                    ui.label("No star database found.");
                    ui.label("Build it now from the bundled catalog?");
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new("Takes ~10 s to a few minutes (one time). Cached afterward.")
                            .size(12.0)
                            .color(pal.text_secondary),
                    );
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if widgets::styled_button(ui, "Build now", &pal) { build = true; }
                        if widgets::styled_button(ui, "Later", &pal) { later = true; }
                    });
                });
            if build { self.start_solver_generation(); }
            if later { self.show_build_prompt = false; }
        }

        // Keyboard shortcuts. Consumed here, ahead of the widgets; the bare-key
        // ones stay quiet while any widget has keyboard focus (text fields,
        // drag values) so typing never trips them.
        let typing = ctx.egui_wants_keyboard_input();
        let (sc_open, sc_connect, sc_quit, sc_side, sc_bottom, sc_play, sc_record) = ctx.input_mut(|i| (
            i.consume_shortcut(&SC_OPEN),
            i.consume_shortcut(&SC_CONNECT),
            i.consume_shortcut(&SC_QUIT),
            !typing && i.consume_shortcut(&SC_SIDE_PANEL),
            !typing && i.consume_shortcut(&SC_BOTTOM_PANEL),
            !typing && i.consume_shortcut(&SC_PLAY),
            !typing && i.consume_shortcut(&SC_RECORD),
        ));
        if sc_side { self.side_panel_open = !self.side_panel_open; }
        if sc_bottom { self.bottom_panel_open = !self.bottom_panel_open; }
        if sc_open && self.pending_fits_path.is_none() { self.open_fits_dialog(); }
        if sc_connect { self.connect_dialog_open = true; }
        if sc_quit { ctx.send_viewport_cmd(egui::ViewportCommand::Close); }
        if sc_play {
            if self.capture_running {
                self.pause_or_stop();
            } else if !matches!(self.camera_source, CameraSource::None) {
                self.play_or_resume();
            }
        }
        if sc_record {
            if self.recording { self.stop_recording(); } else { self.start_recording(); }
        }

        // Menu bar
        egui::Panel::top("menu_bar")
            .frame(egui::Frame::new()
                .fill(pal.toolbar_fill)
                .inner_margin(egui::Margin { left: 4, right: 4, top: 2, bottom: 2 })
                .stroke(egui::Stroke::new(1.0, pal.toolbar_border))
            )
            .show(ui, |ui| {
                egui::MenuBar::new().ui(ui, |ui| {
                    // ── File ────────────────────────────────────────────
                    ui.menu_button("File", |ui| {
                        let dialog_pending = self.pending_fits_path.is_some();
                        if ui.add_enabled(!dialog_pending, egui::Button::new("Open FITS...").shortcut_text(ctx.format_shortcut(&SC_OPEN))).clicked() {
                            self.open_fits_dialog();
                            ui.close();
                        }
                        ui.separator();
                        if self.recording {
                            if ui.add(egui::Button::new("Stop Recording").shortcut_text(ctx.format_shortcut(&SC_RECORD))).clicked() {
                                self.stop_recording();
                                ui.close();
                            }
                        } else {
                            if ui.add(egui::Button::new("Start Recording").shortcut_text(ctx.format_shortcut(&SC_RECORD))).clicked() {
                                self.start_recording();
                                ui.close();
                            }
                        }
                        ui.separator();
                        if ui.add(egui::Button::new("Quit").shortcut_text(ctx.format_shortcut(&SC_QUIT))).clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });

                    // ── View ────────────────────────────────────────────
                    ui.menu_button("View", |ui| {
                        ui.menu_button("Colormap", |ui| {
                            for &kind in ColormapKind::ALL {
                                if menu_radio(ui, self.colormap.kind == kind, kind.name()) {
                                    self.colormap = Colormap::new(kind);
                                    ui.close();
                                }
                            }
                        });
                        ui.menu_button("Scale Mode", |ui| {
                            for &(mode, name) in ScaleMode::ALL {
                                if menu_radio(ui, self.scale_mode == mode, name) {
                                    self.scale_mode = mode;
                                    ui.close();
                                }
                            }
                        });
                        ui.menu_button("Transfer Function", |ui| {
                            for &(tf, name) in imageview::TransferFn::ALL {
                                if menu_radio(ui, self.display_params.transfer == tf, name) {
                                    self.display_params.transfer = tf;
                                    ui.close();
                                }
                            }
                        });
                        ui.separator();
                        if menu_check_sc(ui, self.side_panel_open, "Side Panel", ctx.format_shortcut(&SC_SIDE_PANEL)) {
                            self.side_panel_open = !self.side_panel_open;
                        }
                        if menu_check_sc(ui, self.bottom_panel_open, "Bottom Panel", ctx.format_shortcut(&SC_BOTTOM_PANEL)) {
                            self.bottom_panel_open = !self.bottom_panel_open;
                        }
                        ui.separator();
                        if menu_check(ui, self.display_params.show_axes, "Show Axes") {
                            self.display_params.show_axes = !self.display_params.show_axes;
                        }
                        if menu_check(ui, self.display_params.show_colorbar, "Show Colorbar") {
                            self.display_params.show_colorbar = !self.display_params.show_colorbar;
                        }
                        #[cfg(feature = "starsolve")]
                        {
                            ui.separator();
                            if menu_check(ui, self.show_centroids, "Show Centroids") {
                                self.show_centroids = !self.show_centroids;
                            }
                            if menu_check(ui, self.show_matched_stars, "Show Matched Stars") {
                                self.show_matched_stars = !self.show_matched_stars;
                            }
                            if menu_check(ui, self.show_star_names, "Show Star Names") {
                                self.show_star_names = !self.show_star_names;
                            }
                        }
                    });

                    // ── Source ──────────────────────────────────────────
                    // One flat, kind-tagged list across all camera backends;
                    // non-enumerable sources (FITS, GigE-by-IP) have their own
                    // entries. Everything opens through open_source().
                    ui.menu_button("Source", |ui| {
                        let play_sc = ctx.format_shortcut(&SC_PLAY);
                        if self.capture_running {
                            if ui.add(egui::Button::new("Stop").shortcut_text(play_sc)).clicked() {
                                self.stop_capture();
                                ui.close();
                            }
                        } else {
                            if ui.add(egui::Button::new("Play").shortcut_text(play_sc)).clicked() {
                                let source = self.camera_source.clone();
                                self.open_source(source);
                                ui.close();
                            }
                        }
                        ui.separator();
                        if ui.add(egui::Button::new("Connect\u{2026}").shortcut_text(ctx.format_shortcut(&SC_CONNECT))).clicked() {
                            self.connect_dialog_open = true;
                            ui.close();
                        }
                        ui.separator();
                        if ui.button("Refresh Sources").clicked() {
                            self.refresh_sources();
                        }
                        let sources = self.discovered_source_list();
                        if sources.is_empty() {
                            ui.add_enabled(false, egui::Button::new("No cameras found"));
                        }
                        for (source, label) in sources {
                            let is_this = self.camera_source == source;
                            if menu_radio(ui, is_this, &label) && !is_this {
                                self.open_source(source);
                                ui.close();
                            }
                        }
                        ui.separator();
                        let dialog_pending = self.pending_fits_path.is_some();
                        if ui.add_enabled(!dialog_pending, egui::Button::new("Open FITS...").shortcut_text(ctx.format_shortcut(&SC_OPEN))).clicked() {
                            self.open_fits_dialog();
                            ui.close();
                        }
                    });

                    // ── Theme ───────────────────────────────────────────
                    ui.menu_button("Theme", |ui| {
                        for &(theme, name) in widgets::UiTheme::ALL {
                            if menu_radio(ui, self.ui_theme == theme, name) {
                                self.ui_theme = theme;
                                ui.close();
                            }
                        }
                    });
                });
            });

        // Top toolbar
        egui::Panel::top("toolbar")
            .exact_size(38.0)
            .frame(egui::Frame::new()
                .fill(pal.toolbar_fill)
                .inner_margin(egui::Margin { left: 10, right: 10, top: 4, bottom: 6 })
                .stroke(egui::Stroke::new(1.0, pal.toolbar_border))
            )
            .show(ui, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 6.0;
                    // Toggle side panel
                    let sidebar_icon = if self.side_panel_open { "\u{25E8}" } else { "\u{25E7}" };
                    let sidebar_btn = ui.add(egui::Button::new(
                        egui::RichText::new(sidebar_icon).size(16.0)
                    ).min_size(egui::vec2(28.0, 0.0)));
                    if sidebar_btn.clicked() {
                        self.side_panel_open = !self.side_panel_open;
                    }
                    sidebar_btn.on_hover_text(format!("Toggle side panel ({})", ctx.format_shortcut(&SC_SIDE_PANEL)));
                    ui.separator();
                    let play_sc = ctx.format_shortcut(&SC_PLAY);
                    if self.capture_running {
                        if ui.button(egui::RichText::new("\u{23F9}  Stop").size(14.0))
                            .on_hover_text(format!("Stop capture ({play_sc})"))
                            .clicked()
                        {
                            self.pause_or_stop();
                        }
                    } else if widgets::tip(
                        ui,
                        &if matches!(self.camera_source, CameraSource::None) {
                            format!("No source selected: opens the Connect dialog ({play_sc})")
                        } else {
                            format!("Start capture from the selected source ({play_sc})")
                        },
                        |ui| widgets::primary_button(ui, "\u{25B6}  Play", &pal),
                    ) {
                        self.play_or_resume();
                    }
                    ui.separator();
                    // The record/stop control. Red record semantics (filled
                    // circle to start, filled square to stop) so idle vs.
                    // recording is unmistakable; sized to match the other
                    // transport buttons via the shared icon_button helper.
                    let rec_red = pal.status_err;
                    if self.recording {
                        // Blinking "armed" indicator to the left of the button.
                        // Honor reduced-motion: hold the dot solid if requested.
                        let reduced_motion = ui.style().animation_time <= 0.0;
                        let alpha = if reduced_motion {
                            255
                        } else {
                            let t = ui.input(|i| i.time);
                            (((t * 3.0).sin() * 0.5 + 0.5) * 200.0) as u8 + 55
                        };
                        let (dot_rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 26.0), egui::Sense::hover());
                        ui.painter().circle_filled(
                            dot_rect.center(),
                            4.0,
                            egui::Color32::from_rgba_unmultiplied(220, 40, 40, alpha),
                        );
                        let tip = format!("Stop recording ({})", ctx.format_shortcut(&SC_RECORD));
                        if widgets::tip(ui, &tip, |ui| widgets::icon_button(ui, "\u{25A0}", rec_red, "Stop", &pal)) {
                            self.stop_recording();
                        }
                    } else {
                        let tip = format!(
                            "Record incoming frames to a FITS file in {} ({})",
                            Self::recordings_dir().display(),
                            ctx.format_shortcut(&SC_RECORD),
                        );
                        if widgets::tip(ui, &tip, |ui| widgets::icon_button(ui, "\u{25CF}", rec_red, "Record", &pal)) {
                            self.start_recording();
                        }
                    }
                    // Display settings (colormap, scale, theme) live in the
                    // DISPLAY panel + View/Theme menus; the toolbar stays a
                    // transport bar. fps reads on the right.
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 12.0;
                        ui.label(egui::RichText::new(format!("{:.1} fps", self.fps)).monospace().size(13.0).color(pal.accent))
                            .on_hover_text("Frames per second reaching the display");
                    });
                });
            });

        // Status bar — rendered before side panel so it spans the full window width
        let unread_count = self.unread_log().count();
        let unread_err = self.unread_log().any(|e| matches!(e.level, LogLevel::Error));
        let unread_last = self.unread_log().last().map(|e| e.message.clone());
        let mut open_log = false;
        egui::Panel::bottom("status_bar")
            .exact_size(22.0)
            .frame(egui::Frame::new()
                .fill(pal.statusbar_fill)
                .inner_margin(egui::Margin { left: 10, right: 10, top: 2, bottom: 2 })
                .stroke(egui::Stroke::new(1.0, pal.statusbar_border))
            )
            .show(ui, |ui| {
                let dim = pal.text_secondary;
                let small = 11.0;
                let sep = |ui: &mut egui::Ui| {
                    ui.label(egui::RichText::new("·").size(small).color(dim));
                };
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 6.0;

                    if self.recording {
                        ui.label(egui::RichText::new(format!(
                            "\u{25CF} REC {} · {} frames",
                            self.rec_filename, self.rec_frame_count
                        )).size(small).monospace().color(pal.status_err));
                        sep(ui);
                    }

                    // Source · dimensions · bit depth
                    ui.label(egui::RichText::new(self.source_label()).size(small).monospace().color(dim));
                    if let Some(frame) = &self.current_frame {
                        sep(ui);
                        ui.label(egui::RichText::new(format!("{}×{}", frame.width, frame.height))
                            .size(small).monospace().color(pal.text_primary));
                        sep(ui);
                        ui.label(egui::RichText::new(format!("{}-bit", frame.bit_depth))
                            .size(small).monospace().color(pal.text_primary));
                    }

                    // Right group: solve status + cursor readout
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 6.0;
                        // Unread warnings and errors: count plus the latest
                        // message, clickable through to the Log tab.
                        if let Some(last) = unread_last {
                            let color = if unread_err { pal.status_err } else { pal.status_warn };
                            let mut msg = last.clone();
                            if msg.chars().count() > 60 {
                                msg = msg.chars().take(57).collect::<String>() + "…";
                            }
                            let text = format!("\u{26A0} {}  {}", unread_count, msg);
                            let r = ui.add(egui::Label::new(egui::RichText::new(text).size(small).color(color)).sense(egui::Sense::click()));
                            if r.on_hover_text("Open the Log tab").clicked() {
                                open_log = true;
                            }
                            sep(ui);
                        }
                        #[cfg(feature = "starsolve")]
                        {
                            // solve_busy also covers extraction-only jobs; only
                            // call it solving when the solver is actually on.
                            if self.solve_enabled && self.solve_busy {
                                ui.label(egui::RichText::new("Solving…").size(small).monospace()
                                    .color(pal.status_warn));
                            } else if self.last_solve.as_ref().is_some_and(|s| s.is_ok()) {
                                ui.label(egui::RichText::new("Solved").size(small).monospace()
                                    .color(pal.status_ok));
                            }
                        }
                        if let (Some((px, py)), Some(val)) = (self.cursor_pixel, self.cursor_value) {
                            ui.label(egui::RichText::new(format!("({}, {}) = {:.0}", px, py, val))
                                .size(small).monospace().color(pal.text_primary));
                        }
                    });
                });
            });

        if open_log {
            self.bottom_tab = BottomTab::Log;
            self.bottom_panel_open = true;
        }

        // Side panel — rendered before bottom panel so it extends full height to status bar
        if self.side_panel_open {
            let resp = egui::Panel::left("controls")
                .resizable(true).default_size(self.side_panel_width)
                .frame(egui::Frame::new()
                    .fill(pal.panel_fill)
                    .inner_margin(egui::Margin { left: 6, right: 6, top: 8, bottom: 6 })
                )
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| { self.side_panel(ui); });
                });
            self.side_panel_width = resp.response.rect.width();
        }

        // Bottom tabbed panel — rendered after side panel so it only spans the image area
        // Collapsed, only the tab strip remains. egui's switched panel keeps
        // the two sizes under separate ids, animates between them, and lets
        // a drag on the handle past the strip height collapse the panel.
        let frame = egui::Frame::new().fill(pal.panel_fill).inner_margin(egui::Margin::ZERO);
        let expanded = egui::Panel::bottom("bottom_panel")
            .resizable(true)
            .default_size(self.bottom_panel_height)
            .size_range(120.0..=900.0)
            .frame(frame);
        let collapsed = egui::Panel::bottom("bottom_panel_collapsed").resizable(false).exact_size(28.0).frame(frame);
        // The tab strip inside may flip the flag itself (chevron, tab click);
        // the panel's own drag-to-collapse reports through `open`. Whichever
        // changed this frame wins.
        let before = self.bottom_panel_open;
        let mut open = before;
        let resp = egui::Panel::show_switched(ui, &mut open, collapsed, expanded, |ui, show_expanded| {
                // egui stores a panel's height from its content each frame, so
                // a tab with little in it would shrink the panel and that size
                // would stick. Claim the full height whatever the tab holds.
                ui.set_min_height(ui.available_height());
                self.bottom_panel_tabs(ui);
                if !show_expanded {
                    return;
                }

                egui::Frame::new()
                    .fill(pal.panel_fill)
                    .inner_margin(egui::Margin { left: 4, right: 4, top: 0, bottom: 4 })
                    .show(ui, |ui| {
                        match self.bottom_tab {
                            BottomTab::Histogram => self.histogram_content(ui),
                            BottomTab::Controls => {
                                #[cfg(any(feature = "svbony", feature = "gev", feature = "toupcam", feature = "indi"))]
                                {
                                    egui::ScrollArea::vertical().show(ui, |ui| {
                                        self.controls_content(ui);
                                    });
                                }
                                #[cfg(not(any(feature = "svbony", feature = "gev", feature = "toupcam", feature = "indi")))]
                                ui.label("Camera support not compiled in");
                            }
                            #[cfg(feature = "starsolve")]
                            BottomTab::PlateSolve => self.plate_solve_content(ui),
                            #[cfg(feature = "focus")]
                            BottomTab::Focus => self.focus_content(ui),
                            BottomTab::Log => self.log_content(ui),
                        }
                    });
            });
        if self.bottom_panel_open == before {
            self.bottom_panel_open = open;
        }
        if self.bottom_panel_open && open {
            self.bottom_panel_height = resp.response.rect.height();
        }

        // Update display params with current palette colors
        self.display_params.axes_text_color = pal.axes_text;
        self.display_params.axes_stroke_color = pal.axes_stroke;
        self.display_params.overlay_colors = overlays::OverlayColors {
            dim: pal.overlay_dim,
            bright: pal.overlay_bright,
            matched: pal.overlay_matched,
            catalog: pal.overlay_catalog,
            label: pal.overlay_label,
        };
        self.display_params.roi_color = pal.roi_outline;

        // Central panel
        egui::CentralPanel::default()
            .frame(egui::Frame::new()
                .fill(pal.image_bg)
                .inner_margin(egui::Margin { left: 4, right: 4, top: 12, bottom: 4 })
            )
            .show(ui, |ui| {
            if let Some(frame) = &self.current_frame {
                let resp = self.image_viewer.show(ui, &frame.mono, self.frame_serial, frame.width, frame.height, &self.display_params, &self.colormap, &self.overlay_items);
                self.cursor_pixel = resp.hovered_pixel;
                self.cursor_value = resp.hovered_value;
            } else {
                let pal2 = self.pal();
                let source = self.source_label();
                let has_source = !matches!(self.camera_source, CameraSource::None);
                let cmap_name = self.colormap.kind.name();
                let dialog_pending = self.pending_fits_path.is_some();
                let mut open = false;
                ui.vertical_centered(|ui| {
                    ui.add_space((ui.available_height() * 0.30).max(24.0));
                    ui.label(egui::RichText::new("AstroViewer")
                        .font(egui::FontId::new(30.0, strong_family()))
                        .color(pal2.text_secondary));
                    ui.add_space(8.0);
                    let hint = if has_source {
                        format!("Press Play to start  {}", source)
                    } else {
                        "Open a FITS file or connect a camera to begin".to_string()
                    };
                    ui.label(egui::RichText::new(hint).size(13.0).color(pal2.text_secondary));
                    ui.add_space(18.0);
                    if !dialog_pending && widgets::primary_button(ui, "Open FITS\u{2026}", &pal2) {
                        open = true;
                    }
                    ui.add_space(12.0);
                    ui.label(egui::RichText::new(format!("Colormap  {}", cmap_name))
                        .size(11.0).monospace().color(pal2.text_secondary));
                });
                if open {
                    self.open_fits_dialog();
                }
            }
        });

        // Zoom popup window
        self.show_zoom_window(&ctx);

        // Connect-to-source window
        self.connect_dialog(&ctx);
    }
}

/// Wall-clock budget for a single live plate-solve attempt.
///
/// Measured against real captures: a solve that is going to succeed lands well
/// under this (0.5–6 ms), while one with no possible match runs until stopped —
/// pattern combinations grow as C(n,4), so a few thousand read-noise centroids
/// never terminate on their own. tetra3 honors this to the millisecond.
#[cfg(feature = "starsolve")]
const SOLVE_TIMEOUT_MS: u64 = 10;

/// Set by the signal handler; the next UI pass turns it into a window close.
static QUIT_REQUESTED: AtomicBool = AtomicBool::new(false);

/// One frame's work for the plate-solve worker: extract centroids, then solve
/// them if a database is loaded.
#[cfg(feature = "starsolve")]
struct SolveJob {
    mono: Arc<Vec<f32>>,
    width: u32,
    height: u32,
    centroid_config: tetra3::CentroidExtractionConfig,
    /// Extract with the single-pass fast path instead of the CCL path.
    tracking: bool,
    /// `None` when no database is loaded — the worker still extracts, so the
    /// centroid overlay and star count keep working without a solver.
    solve: Option<SolveParams>,
    /// Focus measurement to run on this frame's centroids.
    #[cfg(feature = "focus")]
    focus: Option<focus::FocusConfig>,
}

/// Solver inputs, snapshotted on the UI thread at dispatch time.
#[cfg(feature = "starsolve")]
struct SolveParams {
    db: Arc<tetra3::SolverDatabase>,
    fov_rad: f32,
    fov_max_error: Option<f32>,
    attitude_hint: Option<tetra3::Quaternion>,
    camera_model: Option<tetra3::CameraModel>,
}

/// One frame's results. Centroids travel with their solve so
/// `matched_centroid_indices` always indexes the centroids it was solved from.
#[cfg(feature = "starsolve")]
struct SolveOutput {
    centroids: Vec<tetra3::Centroid>,
    extract_ms: f32,
    solve: Option<tetra3::SolveResult>,
    #[cfg(feature = "focus")]
    focus: Option<focus::FocusSample>,
}

/// Long-lived worker running extraction and solving off the UI thread. One
/// thread for the life of the app rather than one per frame: at full sensor
/// resolution a single job is ~100 ms of work, and the UI must stay responsive
/// throughout.
#[cfg(feature = "starsolve")]
fn spawn_solve_worker() -> (Sender<SolveJob>, Receiver<SolveOutput>) {
    let (job_tx, job_rx) = bounded::<SolveJob>(1);
    let (out_tx, out_rx) = bounded::<SolveOutput>(1);
    thread::spawn(move || {
        // Ends when the app drops the job sender.
        while let Ok(job) = job_rx.recv() {
            let t0 = Instant::now();
            // Tracking mode swaps in the single-pass fast extractor. The shared
            // sliders map across directly; knobs the fast path doesn't have
            // (Blur σ) are simply ignored, and its own extras (sharpness gate,
            // saturation level) stay at their defaults.
            let centroids: Vec<tetra3::Centroid> = if job.tracking {
                let cfg = tetra3::FastCentroidConfig {
                    sigma_threshold: job.centroid_config.sigma_threshold,
                    // The fast path has no global-background option; fall back
                    // to its default grid when the CCL slider says "global".
                    bg_grid: job.centroid_config.local_bg_block_size.unwrap_or(64),
                    min_pixels: job.centroid_config.min_pixels,
                    max_pixels: job.centroid_config.max_pixels,
                    max_centroids: job.centroid_config.max_centroids,
                    max_elongation: job.centroid_config.max_elongation,
                    ..Default::default()
                };
                tetra3::extract_centroids_fast(&job.mono, job.width, job.height, &cfg)
            } else {
                tetra3::extract_centroids_from_raw(
                    &job.mono, job.width, job.height, &job.centroid_config,
                )
            }
            .map(|r| r.centroids)
            .unwrap_or_default();
            let extract_ms = t0.elapsed().as_secs_f32() * 1000.0;

            // Focus figures ride on the same frame and centroids. A few
            // thousand pixels of arithmetic; nothing next to extraction.
            #[cfg(feature = "focus")]
            let focus = job.focus.as_ref().map(|cfg| {
                let stars: Vec<focus::Star> = centroids
                    .iter()
                    .map(|c| focus::Star {
                        x: c.x,
                        y: c.y,
                        mass: c.mass.unwrap_or(0.0),
                        // The overlay ellipse axes are 3σ; undo that for the
                        // window size. The ratio is scale-free.
                        sigma: c.cov.map(|cov| overlays::cov_to_ellipse(cov).0 / 3.0),
                        elongation: c.cov.map(|cov| {
                            let (a, b, _) = overlays::cov_to_ellipse(cov);
                            if b > 0.0 { a / b } else { f32::INFINITY }
                        }),
                    })
                    .collect();
                focus::measure(&job.mono, job.width, job.height, &stars, cfg)
            });

            let solve = job.solve.map(|p| {
                let mut cfg = tetra3::SolveConfig::new(p.fov_rad, job.width, job.height);
                cfg.fov_max_error_rad = p.fov_max_error;
                // Live tracking: a solve that cannot finish in a few ms is not
                // worth finishing — the next frame is along shortly. Bounds the
                // pathological case (thousands of read-noise centroids never
                // matching, which otherwise burns a full core on every frame)
                // without capping the centroid count real solves depend on.
                cfg.solve_timeout_ms = Some(SOLVE_TIMEOUT_MS);
                if let Some(cam) = p.camera_model {
                    cfg.camera_model = cam;
                }
                if let Some(q) = p.attitude_hint {
                    cfg.attitude_hint = Some(q);
                    cfg.hint_uncertainty_rad = 2.0_f32.to_radians();
                }
                p.db.solve_from_centroids(&centroids, &cfg)
            });

            let out = SolveOutput {
                centroids,
                extract_ms,
                solve,
                #[cfg(feature = "focus")]
                focus,
            };
            if out_tx.send(out).is_err() {
                break;
            }
        }
    });
    (job_tx, out_rx)
}

impl ViewerApp {
    /// Hand this frame to the solve worker, unless it is still busy with the
    /// previous one — in which case this frame is skipped rather than queued, so
    /// the pipeline always works on recent data instead of falling behind.
    #[cfg(feature = "starsolve")]
    fn maybe_dispatch_solve(&mut self, mono: &Pixels, w: u32, h: u32, bit_depth: u8) {
        if self.solve_busy {
            return;
        }
        // The worker needs owned `f32`. A U16 frame is widened here, after the
        // busy check so a skipped frame costs nothing, and gets its own Arc so
        // it never pins the pooled u16 buffer.
        let mono = mono.to_f32_arc();

        // Solver inputs, if a database is loaded. Without one the worker still
        // extracts, so centroid overlays keep working.
        let solve = self.solver_db.as_ref().map(|db| {
            // Use previous solve's FOV if available, otherwise user estimate
            let prev_solution = self.last_solve.as_ref().and_then(|s| s.as_ref().ok());
            let prev_locked = prev_solution.is_some();
            SolveParams {
                db: db.clone(),
                fov_rad: prev_solution
                    .map(|s| s.fov_rad)
                    .unwrap_or_else(|| self.fov_estimate_deg.to_radians()),
                // Wide FOV tolerance for initial solve, tight once we have a lock
                fov_max_error: Some(if prev_locked { 2.0_f32 } else { 10.0_f32 }.to_radians()),
                // Seed tracking-mode solve with previous attitude when locked.
                attitude_hint: prev_solution.map(|s| s.qicrs2cam),
                camera_model: self.camera_model.clone(),
            }
        });

        // Focus: stars whose peak reaches ~95% of full scale are treated as
        // clipped. Judged on the frame the worker sees, so a background-
        // subtracted frame is slightly lenient; the cores it lets through
        // are already flat and drop out on the elongation cut or read wide.
        #[cfg(feature = "focus")]
        let focus_cfg = focus::FocusConfig {
            saturation: Some(0.95 * ((1u64 << bit_depth) - 1) as f32),
            roi: if self.focus_use_roi { self.image_viewer.roi_rect } else { None },
            ..Default::default()
        };
        #[cfg(not(feature = "focus"))]
        let _ = bit_depth;

        let job = SolveJob {
            mono,
            width: w,
            height: h,
            centroid_config: self.centroid_config.clone(),
            tracking: self.tracking_mode,
            solve,
            #[cfg(feature = "focus")]
            focus: Some(focus_cfg),
        };
        if self.solve_tx.try_send(job).is_ok() {
            self.solve_busy = true;
        }
    }



    /// The Connect window: every backend's discovered devices plus inline
    /// address entry for addressed backends (GigE by IP, INDI server), all
    /// grouped under one roof so the Source menu and side panel stay small.
    fn connect_dialog(&mut self, ctx: &egui::Context) {
        if !self.connect_dialog_open {
            return;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.connect_dialog_open = false;
            return;
        }
        let pal = self.pal();
        let mut keep_open = true;
        // Actions are collected during the UI pass and applied after it, so
        // the closure never re-enters &mut self.
        let mut connect: Option<CameraSource> = None;
        let mut manual_err: Option<String> = None;
        let mut refresh = false;
        let mut open_fits = false;

        egui::Window::new("Connect to Source")
            .open(&mut keep_open)
            .collapsible(false)
            .resizable(false)
            .default_width(300.0)
            .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
            .show(ctx, |ui| {
                ui.spacing_mut().item_spacing.y = 5.0;

                if let Some(err) = &self.camera_error {
                    ui.label(egui::RichText::new(err).color(pal.status_err).small());
                }

                for b in sources::backends() {
                    ui.add_space(2.0);
                    connect_group_header(ui, b.name, &pal);

                    let mut any = false;
                    for d in self.discovered.iter().filter(|d| d.backend == b.name) {
                        any = true;
                        let text = d.title();
                        let is_current = self.camera_source == d.source;
                        if ui.selectable_label(is_current, egui::RichText::new(text).size(13.0)).clicked() {
                            connect = Some(d.source.clone());
                        }
                    }
                    if !any && b.discover.is_some() {
                        ui.label(egui::RichText::new("none found").italics().size(12.0).color(pal.text_secondary));
                    }

                    if let Some(spec) = &b.manual {
                        ui.horizontal(|ui| {
                            ui.label(egui::RichText::new(spec.label).size(13.0));
                            let input = self.manual_inputs.entry(b.scheme).or_default();
                            let resp = ui.add(
                                egui::TextEdit::singleline(input)
                                    .hint_text(spec.hint)
                                    .desired_width(150.0),
                            );
                            let submitted =
                                resp.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                            if widgets::styled_button(ui, "Connect", &pal) || submitted {
                                match (spec.make_source)(input) {
                                    Ok(src) => connect = Some(src),
                                    Err(e) => manual_err = Some(e),
                                }
                            }
                        });
                    }
                }

                ui.add_space(2.0);
                connect_group_header(ui, "Files", &pal);
                let dialog_pending = self.pending_fits_path.is_some();
                if !dialog_pending && widgets::styled_button(ui, "Open FITS\u{2026}", &pal) {
                    open_fits = true;
                }

                ui.add_space(6.0);
                ui.separator();
                if widgets::styled_button(ui, "\u{27F3} Refresh", &pal) {
                    refresh = true;
                }
            });

        self.connect_dialog_open = keep_open;
        if let Some(e) = manual_err {
            self.camera_error = Some(e);
        }
        if refresh {
            self.refresh_sources();
        }
        if open_fits {
            self.open_fits_dialog();
            self.connect_dialog_open = false;
        }
        if let Some(src) = connect {
            self.camera_error = None;
            self.open_source(src);
            if self.camera_error.is_none() {
                self.connect_dialog_open = false;
            }
        }
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

        // Recolor and re-upload only when the frame, ROI, or display params
        // changed — repaints happen far more often than any of those.
        let key = ZoomKey {
            frame_serial: self.frame_serial,
            roi,
            scale_min: self.display_params.scale_min,
            scale_max: self.display_params.scale_max,
            gamma: self.display_params.gamma,
            asinh_offset: self.display_params.asinh_offset,
            transfer: self.display_params.transfer,
            colormap: self.colormap.kind,
        };
        if self.zoom_key != Some(key) || self.zoom_texture.is_none() {
            self.zoom_key = Some(key);

            // Build zoomed RGBA from the ROI sub-region
            let npix = roi_w * roi_h;
            self.zoom_rgba.resize(npix * 4, 255);

            let range = self.display_params.scale_max - self.display_params.scale_min;
            let inv_range = if range > 0.0 { 1.0 / range } else { 1.0 };
            let inv_gamma = if self.display_params.gamma != 0.0 { 1.0 / self.display_params.gamma } else { 1.0 };
            let apply_gamma = (self.display_params.gamma - 1.0).abs() > 1e-4;
            let asinh_alpha = self.display_params.gamma;
            // Mirrors the pivoted asinh in ImageViewer::update_rgba.
            let (asinh_o, asinh_lo, asinh_norm): (f32, f32, f32) = if matches!(self.display_params.transfer, imageview::TransferFn::Asinh) {
                let o = ((self.display_params.asinh_offset - self.display_params.scale_min) * inv_range).clamp(0.0, 1.0);
                let lo = (-asinh_alpha * o).asinh();
                let hi = (asinh_alpha * (1.0 - o)).asinh();
                let norm = if hi > lo { 1.0 / (hi - lo) } else { 1.0 };
                (o, lo, norm)
            } else { (0.0, 0.0, 1.0) };

            for ry in 0..roi_h {
                for rx in 0..roi_w {
                    let src_idx = ((y0 as usize + ry) * frame.width as usize) + (x0 as usize + rx);
                    let val = frame.mono.value_at(src_idx).unwrap_or(0.0);
                    let mut t = ((val - self.display_params.scale_min) * inv_range).clamp(0.0, 1.0);
                    match self.display_params.transfer {
                        imageview::TransferFn::Linear => { if apply_gamma { t = t.powf(inv_gamma); } }
                        imageview::TransferFn::Asinh => { t = (((asinh_alpha * (t - asinh_o)).asinh() - asinh_lo) * asinh_norm).clamp(0.0, 1.0); }
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
        }

        // Close on Escape or X key
        // X is a bare key: ignore it while a text field or drag value has focus.
        let typing = ctx.egui_wants_keyboard_input();
        let close_key = ctx.input(|i| {
            i.key_pressed(egui::Key::Escape) || (!typing && i.key_pressed(egui::Key::X))
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

                    overlays::draw_overlays(ui.painter(), &overlay_items, to_screen, scale_x, max_mass, 2.0, &self.display_params.overlay_colors);

                    // Pixel info on hover
                    if let Some(pos) = resp.hover_pos() {
                        let rx = (pos.x - img_rect.min.x) / scale_x;
                        let ry = (pos.y - img_rect.min.y) / scale_y;
                        let px = (x0 as f32 + rx) as u32;
                        let py = (y0 as f32 + ry) as u32;
                        if px < img_w as u32 && py < img_h as u32 {
                            let idx = (py * img_w as u32 + px) as usize;
                            if let Some(val) = frame.mono.value_at(idx) {
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

fn start_fits_capture(tx: Sender<FrameData>, stop_rx: Receiver<()>, mut source: fits_source::FitsSource, target_fps: Arc<AtomicU32>) {
    let bit_depth = source.bit_depth;
    thread::spawn(move || {
        loop {
            let fps = target_fps.load(Ordering::Relaxed).max(1);
            let frame_interval = std::time::Duration::from_secs_f64(1.0 / fps as f64);
            let t0 = Instant::now();
            match stop_rx.try_recv() {
                Ok(()) | Err(crossbeam_channel::TryRecvError::Disconnected) => break,
                Err(crossbeam_channel::TryRecvError::Empty) => {}
            }
            let mono = source.next_frame();
            let frame_data = FrameData::from_pixels(mono, source.width, source.height, bit_depth);
            if tx.try_send(frame_data).is_err() && tx.is_empty() { break; }
            let elapsed = t0.elapsed();
            if elapsed < frame_interval { thread::sleep(frame_interval - elapsed); }
        }
    });
}

#[cfg(any(feature = "svbony", feature = "toupcam"))]
fn process_image(img: DynamicImage, bit_depth: u8) -> FrameData {
    let width = img.width();
    let height = img.height();
    // Integer mono images stay `u16` (no widening copy); only the RGB→luma
    // weighting, which is fractional, needs `f32`.
    match &img {
        DynamicImage::ImageLuma8(g) => {
            let mono: Vec<u16> = g.as_raw().iter().map(|&v| v as u16).collect();
            FrameData::new_u16(mono, width, height, bit_depth)
        }
        DynamicImage::ImageLuma16(g) => {
            FrameData::new_u16(g.as_raw().clone(), width, height, bit_depth)
        }
        DynamicImage::ImageRgb8(rgb) => {
            let mono: Vec<f32> = rgb.pixels().map(|p| 0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32).collect();
            FrameData::new(mono, width, height, bit_depth)
        }
        _ => {
            let gray = img.to_luma8();
            let mono: Vec<u16> = gray.as_raw().iter().map(|&v| v as u16).collect();
            FrameData::new_u16(mono, width, height, bit_depth)
        }
    }
}

/// Menu item with checkmark prefix for toggles.
fn menu_check(ui: &mut egui::Ui, checked: bool, label: &str) -> bool {
    let prefix = if checked { "\u{2713}  " } else { "    " };
    ui.button(format!("{prefix}{label}")).clicked()
}

/// `menu_check` with a right-aligned shortcut hint.
fn menu_check_sc(ui: &mut egui::Ui, checked: bool, label: &str, shortcut: String) -> bool {
    let prefix = if checked { "\u{2713}  " } else { "    " };
    ui.add(egui::Button::new(format!("{prefix}{label}")).shortcut_text(shortcut)).clicked()
}

/// Menu item with dot prefix for radio-style selections.
fn menu_radio(ui: &mut egui::Ui, selected: bool, label: &str) -> bool {
    let prefix = if selected { "\u{2022}  " } else { "    " };
    ui.button(format!("{prefix}{label}")).clicked()
}

fn snap_floor(v: f32, step: f32) -> f32 { (v / step).floor() * step }

/// Step-line points for the histogram bins overlapping `[lo, hi]`.
///
/// The stored histogram has `histogram::NUM_BINS` bins over its full range;
/// bins are merged `k` at a time so the curve carries about 300 steps across
/// the visible window whatever the zoom. Merged y values are summed counts,
/// so the y scale is per drawn step, not per stored bin.
fn hist_step_line(h: &histogram::Histogram, lo: f64, hi: f64, log_y: bool) -> Vec<[f64; 2]> {
    const TARGET_STEPS: usize = 300;
    let n = h.counts.len();
    if n == 0 || h.edges.len() != n + 1 { return Vec::new(); }
    let e0 = h.edges[0] as f64;
    let bw = (h.edges[n] as f64 - e0) / n as f64;
    if !(bw > 0.0) { return Vec::new(); }
    let first = (((lo - e0) / bw).floor().max(0.0) as usize).min(n - 1);
    let last = (((hi - e0) / bw).ceil().max(0.0) as usize).min(n).max(first + 1);
    let k = ((last - first) / TARGET_STEPS).max(1);
    let mut pts = Vec::with_capacity(2 * (last - first) / k + 2);
    let mut i = first;
    while i < last {
        let j = (i + k).min(last);
        let c: u64 = h.counts[i..j].iter().sum();
        let y = if log_y { (c as f64 + 1.0).log10() } else { c as f64 };
        pts.push([e0 + i as f64 * bw, y]);
        pts.push([e0 + j as f64 * bw, y]);
        i = j;
    }
    pts
}
fn snap_ceil(v: f32, step: f32) -> f32 { (v / step).ceil() * step }

/// ZScale algorithm (simplified IRAF/DS9 style).
/// Samples pixels, sorts them, fits a line to the central portion,
/// and derives display min/max that rejects outliers.
fn zscale(data: &Pixels) -> (f64, f64) {
    if data.is_empty() { return (0.0, 1.0); }

    // Sample up to 10000 evenly spaced pixels, read straight from the frame's
    // native buffer: only the samples are widened, never the whole frame.
    let n_samples = data.len().min(10_000);
    let step = data.len() as f64 / n_samples as f64;
    let mut samples: Vec<f64> = (0..n_samples)
        .map(|i| data.value_at((i as f64 * step) as usize).unwrap_or(0.0) as f64)
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
    // Capture panics to a file so a GUI crash is diagnosable even when the
    // app wasn't launched from a terminal (Finder, `open`, crash-after-close).
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let report = format!("{}\n\n{}", info, std::backtrace::Backtrace::force_capture());
        let dir = dirs::config_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join("astroviewer");
        let _ = std::fs::create_dir_all(&dir);
        let _ = std::fs::write(dir.join("last_panic.txt"), &report);
        default_hook(info);
    }));

    if std::env::args().any(|a| a == "-h" || a == "--help") {
        println!(
            "usage: astroviewer [SOURCE]\n\n\
             SOURCE selects the input at startup: a FITS file path, or a descriptor\n\
             \x20 file:<path>      FITS file\n\
             \x20 toupcam:<id>     ToupTek camera (requires the `toupcam` feature)\n\
             \x20 svb:<id>         SVBony camera (requires the `svbony` feature)\n\
             \x20 gev:<ip-or-id>   GigE Vision camera (requires the `gev` feature)\n\
             \x20 indi:<host[:port]>  INDI/INDIGO server (requires the `indi` feature)\n\n\
             With no SOURCE, the viewer reconnects to the last source used\n\
             (remembered across runs); if there is none, it starts idle —\n\
             pick a source from the Source menu."
        );
        return Ok(());
    }
    // cameleon_genapi logs an ERROR every time it constructs an error, even for
    // ones we handle (e.g. probing chunk-backed features we then skip) — keep
    // its internal logging out; failures we care about are reported by our code.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"))
                .add_directive("cameleon_genapi=off".parse().expect("valid directive")),
        )
        .init();
    let options = eframe::NativeOptions {
        // Setting an explicit (empty) icon suppresses eframe's default behavior of
        // loading the bundled egui logo and calling setApplicationIconImage shortly
        // after launch — which would overwrite the macOS Dock icon (and Windows
        // taskbar icon) provided by the app bundle's .icns / the exe's embedded .ico.
        // With IconData::default(), eframe leaves the OS-provided icon untouched.
        viewport: {
            let saved = UiConfig::load();
            let mut vp = egui::ViewportBuilder::default()
                .with_inner_size(saved.window_size.filter(|s| s[0] >= 400.0 && s[1] >= 300.0).unwrap_or([1400.0, 1000.0]))
                .with_title("AstroViewer")
                .with_icon(egui::IconData::default());
            if let Some(pos) = saved.window_pos {
                vp = vp.with_position(pos);
            }
            vp
        },
        ..Default::default()
    };
    eframe::run_native("AstroViewer", options, Box::new(|cc| Ok(Box::new(ViewerApp::new(cc)))))
        .map_err(|e| anyhow::anyhow!("{}", e))?;
    Ok(())
}
