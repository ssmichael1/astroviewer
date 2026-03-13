mod colormaps;
mod histogram;
mod imageview;
mod sim;

use anyhow::Result;
use crossbeam_channel::{bounded, Receiver, TryRecvError};
use eframe::egui;
use image::DynamicImage;
use std::thread;
use std::time::Instant;

use colormaps::{Colormap, ColormapKind};
use histogram::{compute_histogram, compute_stats};
use imageview::{DisplayParams, ImageViewer};
use sim::SimCamera;

/// Extracted frame data ready for display.
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
enum ScaleMode {
    Full,
    Auto,
    Manual,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum CameraSource {
    Simulated,
}

struct ViewerApp {
    frame_rx: Receiver<FrameData>,
    current_frame: Option<FrameData>,

    display_params: DisplayParams,
    colormap: Colormap,
    scale_mode: ScaleMode,
    image_viewer: ImageViewer,

    // Cursor info
    cursor_pixel: Option<(u32, u32)>,
    cursor_value: Option<f64>,

    // Frame rate tracking
    frame_times: Vec<Instant>,
    fps: f64,

    // Camera source
    camera_source: CameraSource,
    capture_running: bool,
    _stop_tx: crossbeam_channel::Sender<()>,

    // Sim settings
    sim_width: u32,
    sim_height: u32,
    sim_bit_depth: u8,
    sim_fps: u32,
}

impl ViewerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Improve visual style
        let mut style = (*cc.egui_ctx.style()).clone();
        style.text_styles.insert(
            egui::TextStyle::Body,
            egui::FontId::new(13.0, egui::FontFamily::Proportional),
        );
        style.text_styles.insert(
            egui::TextStyle::Heading,
            egui::FontId::new(16.0, egui::FontFamily::Proportional),
        );
        style.text_styles.insert(
            egui::TextStyle::Button,
            egui::FontId::new(13.0, egui::FontFamily::Proportional),
        );
        style.text_styles.insert(
            egui::TextStyle::Monospace,
            egui::FontId::new(13.0, egui::FontFamily::Monospace),
        );
        style.text_styles.insert(
            egui::TextStyle::Small,
            egui::FontId::new(11.0, egui::FontFamily::Proportional),
        );
        style.spacing.item_spacing = egui::vec2(8.0, 4.0);
        style.spacing.button_padding = egui::vec2(6.0, 3.0);
        cc.egui_ctx.set_style(style);

        let (frame_tx, frame_rx) = bounded(2);
        let (stop_tx, stop_rx) = bounded(1);

        let sim_width = 1280u32;
        let sim_height = 960u32;
        let sim_bit_depth = 12u8;
        let sim_fps = 30u32;

        start_sim_capture(
            frame_tx,
            stop_rx,
            sim_width,
            sim_height,
            sim_bit_depth,
            sim_fps,
        );

        Self {
            frame_rx,
            current_frame: None,
            display_params: DisplayParams {
                scale_min: 0.0,
                scale_max: 4095.0,
                ..Default::default()
            },
            colormap: Colormap::new(ColormapKind::Viridis),
            scale_mode: ScaleMode::Auto,
            image_viewer: ImageViewer::new(),
            cursor_pixel: None,
            cursor_value: None,
            frame_times: Vec::new(),
            fps: 0.0,
            camera_source: CameraSource::Simulated,
            capture_running: true,
            _stop_tx: stop_tx,
            sim_width,
            sim_height,
            sim_bit_depth,
            sim_fps,
        }
    }

    fn poll_frame(&mut self) {
        let mut latest = None;
        loop {
            match self.frame_rx.try_recv() {
                Ok(frame) => latest = Some(frame),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    self.capture_running = false;
                    break;
                }
            }
        }
        if let Some(frame) = latest {
            match self.scale_mode {
                ScaleMode::Auto => {
                    self.display_params.scale_min = frame.hist.data_min;
                    self.display_params.scale_max = frame.hist.data_max;
                }
                ScaleMode::Full => {
                    self.display_params.scale_min = 0.0;
                    self.display_params.scale_max = ((1u64 << frame.bit_depth) - 1) as f64;
                }
                ScaleMode::Manual => {}
            }

            let now = Instant::now();
            self.frame_times.push(now);
            while self.frame_times.len() > 30 {
                self.frame_times.remove(0);
            }
            if self.frame_times.len() >= 2 {
                let dt = self.frame_times.last().unwrap().duration_since(self.frame_times[0]);
                self.fps = (self.frame_times.len() - 1) as f64 / dt.as_secs_f64();
            }

            self.current_frame = Some(frame);
        }
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Camera");
        ui.separator();

        ui.label("Source:");
        ui.radio_value(&mut self.camera_source, CameraSource::Simulated, "Simulated");
        ui.separator();

        ui.label("Resolution:");
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut self.sim_width).range(64..=4096).speed(8).prefix("W: "));
            ui.add(egui::DragValue::new(&mut self.sim_height).range(64..=4096).speed(8).prefix("H: "));
        });
        ui.horizontal(|ui| {
            ui.label("Bit depth:");
            egui::ComboBox::from_id_salt("bit_depth")
                .selected_text(format!("{}", self.sim_bit_depth))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.sim_bit_depth, 8, "8");
                    ui.selectable_value(&mut self.sim_bit_depth, 12, "12");
                    ui.selectable_value(&mut self.sim_bit_depth, 14, "14");
                    ui.selectable_value(&mut self.sim_bit_depth, 16, "16");
                });
        });
        ui.add(egui::Slider::new(&mut self.sim_fps, 1..=60).text("FPS"));
        ui.separator();

        ui.heading("Display");
        ui.separator();

        let current_name = self.colormap.kind.name();
        egui::ComboBox::from_label("Colormap")
            .selected_text(current_name)
            .show_ui(ui, |ui| {
                for &kind in ColormapKind::ALL {
                    if ui.selectable_value(&mut self.colormap.kind, kind, kind.name()).changed() {
                        self.colormap = Colormap::new(kind);
                    }
                }
            });

        ui.add(
            egui::Slider::new(&mut self.display_params.gamma, 0.1..=5.0)
                .logarithmic(true)
                .text("Gamma"),
        );
        if ui.button("Reset Gamma").clicked() {
            self.display_params.gamma = 1.0;
        }

        ui.separator();

        ui.label("Scale Mode:");
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.scale_mode, ScaleMode::Full, "Full");
            ui.selectable_value(&mut self.scale_mode, ScaleMode::Auto, "Auto");
            ui.selectable_value(&mut self.scale_mode, ScaleMode::Manual, "Manual");
        });

        if self.scale_mode == ScaleMode::Manual {
            let max_range = if let Some(f) = &self.current_frame {
                ((1u64 << f.bit_depth) - 1) as f64
            } else {
                65535.0
            };
            ui.add(
                egui::Slider::new(&mut self.display_params.scale_min, 0.0..=max_range)
                    .text("Min"),
            );
            ui.add(
                egui::Slider::new(&mut self.display_params.scale_max, 0.0..=max_range)
                    .text("Max"),
            );
        } else {
            ui.label(format!(
                "Range: {:.0} – {:.0}",
                self.display_params.scale_min, self.display_params.scale_max,
            ));
        }

        ui.separator();

        ui.checkbox(&mut self.display_params.show_axes, "Show Axes");
        ui.checkbox(&mut self.display_params.show_colorbar, "Show Colorbar");

        ui.separator();

        ui.heading("Statistics");
        ui.separator();
        ui.label(format!("FPS: {:.1}", self.fps));
        if let Some(frame) = &self.current_frame {
            ui.label(format!("Size: {} x {}", frame.width, frame.height));
            ui.label(format!("Bit depth: {}", frame.bit_depth));
            ui.label(format!("Mean: {:.1}", frame.mean));
            ui.label(format!("Std Dev: {:.1}", frame.stddev));
        }

        ui.separator();
        if let (Some((px, py)), Some(val)) = (self.cursor_pixel, self.cursor_value) {
            ui.label(
                egui::RichText::new(format!("({}, {}) = {:.0}", px, py, val))
                    .monospace(),
            );
        } else {
            ui.label(egui::RichText::new("---").monospace().weak());
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_frame();

        if self.capture_running {
            ctx.request_repaint();
        }

        egui::SidePanel::left("controls")
            .resizable(true)
            .default_width(220.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.side_panel(ui);
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(frame) = &self.current_frame {
                let mono = &frame.mono;
                let width = frame.width;
                let height = frame.height;

                let resp = self.image_viewer.show(
                    ui,
                    mono,
                    width,
                    height,
                    &self.display_params,
                    &self.colormap,
                );

                self.cursor_pixel = resp.hovered_pixel;
                self.cursor_value = resp.hovered_value;

                // Histogram at the bottom
                let hist = &frame.hist;
                let hist_height = 120.0;
                let avail = ui.available_size();
                if avail.y > hist_height + 10.0 {
                    ui.separator();
                    let centers = hist.centers();
                    let points: Vec<[f64; 2]> = centers
                        .iter()
                        .zip(hist.counts.iter())
                        .map(|(&x, &y)| [x, y as f64])
                        .collect();

                    egui_plot::Plot::new("histogram")
                        .height(hist_height)
                        .show_axes([true, false])
                        .allow_drag(false)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .show(ui, |plot_ui| {
                            let bars: Vec<egui_plot::Bar> = points
                                .iter()
                                .map(|&[x, y]| {
                                    let bin_width = if centers.len() > 1 {
                                        centers[1] - centers[0]
                                    } else {
                                        1.0
                                    };
                                    egui_plot::Bar::new(x, y).width(bin_width)
                                })
                                .collect();
                            plot_ui.bar_chart(egui_plot::BarChart::new(bars).color(
                                egui::Color32::from_rgb(100, 150, 255),
                            ));

                            plot_ui.vline(
                                egui_plot::VLine::new(self.display_params.scale_min)
                                    .color(egui::Color32::RED)
                                    .width(2.0),
                            );
                            plot_ui.vline(
                                egui_plot::VLine::new(self.display_params.scale_max)
                                    .color(egui::Color32::RED)
                                    .width(2.0),
                            );
                        });
                }
            } else {
                ui.vertical_centered(|ui| {
                    ui.label("Waiting for frames...");
                });
            }
        });
    }
}

fn start_sim_capture(
    tx: crossbeam_channel::Sender<FrameData>,
    stop_rx: Receiver<()>,
    width: u32,
    height: u32,
    bit_depth: u8,
    target_fps: u32,
) {
    thread::spawn(move || {
        let mut cam = SimCamera::new(width, height, bit_depth);
        let frame_interval = std::time::Duration::from_secs_f64(1.0 / target_fps as f64);

        loop {
            let t0 = Instant::now();

            if stop_rx.try_recv().is_ok() {
                break;
            }

            let img = cam.next_frame();
            let frame_data = process_image(img, bit_depth);

            if tx.try_send(frame_data).is_err() {
                if tx.is_empty() {
                    break;
                }
            }

            let elapsed = t0.elapsed();
            if elapsed < frame_interval {
                thread::sleep(frame_interval - elapsed);
            }
        }
    });
}

fn process_image(img: DynamicImage, bit_depth: u8) -> FrameData {
    let width = img.width();
    let height = img.height();

    let mono: Vec<f64> = match &img {
        DynamicImage::ImageLuma8(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageLuma16(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageRgb8(rgb) => {
            rgb.pixels()
                .map(|p| 0.299 * p[0] as f64 + 0.587 * p[1] as f64 + 0.114 * p[2] as f64)
                .collect()
        }
        _ => {
            let gray = img.to_luma8();
            gray.as_raw().iter().map(|&v| v as f64).collect()
        }
    };

    let hist = compute_histogram(&mono, 256);
    let (mean, stddev) = compute_stats(&mono);

    FrameData {
        mono,
        width,
        height,
        hist,
        mean,
        stddev,
        bit_depth,
    }
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("Viewer"),
        ..Default::default()
    };

    eframe::run_native(
        "Viewer",
        options,
        Box::new(|cc| Ok(Box::new(ViewerApp::new(cc)))),
    )
    .map_err(|e| anyhow::anyhow!("{}", e))?;

    Ok(())
}
