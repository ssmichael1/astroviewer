mod colormaps;
mod histogram;
mod sim;

use crossbeam_channel::{bounded, Receiver, TryRecvError};
use iced::widget::{
    button, canvas, checkbox, column, container, pick_list, row, rule, scrollable, slider, text,
    Column,
};
use iced::{
    mouse, Color, Element, Length, Point, Rectangle, Renderer, Size, Subscription, Theme,
};
use image::DynamicImage;
use std::thread;
use std::time::Instant;

use colormaps::{Colormap, ColormapKind};
use histogram::{compute_histogram, compute_stats};
use sim::SimCamera;

fn main() -> iced::Result {
    tracing_subscriber::fmt::init();

    iced::application(Viewer::new, Viewer::update, Viewer::view)
        .subscription(Viewer::subscription)
        .title("Viewer")
        .theme(Viewer::theme)
        .window_size(Size::new(1200.0, 800.0))
        .run()
}

// ---------------------------------------------------------------------------
// Frame data from camera thread
// ---------------------------------------------------------------------------

struct FrameData {
    mono: Vec<f64>,
    width: u32,
    height: u32,
    rgba: Vec<u8>,
    hist: histogram::Histogram,
    mean: f64,
    stddev: f64,
    bit_depth: u8,
}

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ScaleMode {
    Full,
    Auto,
    Manual,
}

struct Viewer {
    frame_rx: Receiver<FrameData>,
    current_frame: Option<FrameData>,

    colormap: Colormap,
    scale_mode: ScaleMode,
    scale_min: f64,
    scale_max: f64,
    gamma: f32,
    show_axes: bool,
    show_colorbar: bool,

    cursor_pixel: Option<(u32, u32)>,
    cursor_value: Option<f64>,

    frame_times: Vec<Instant>,
    fps: f64,

    /// Cached image handle — only rebuilt when frame data changes
    image_handle: Option<iced::widget::image::Handle>,
    frame_gen: u64,

    image_cache: canvas::Cache,
    hist_cache: canvas::Cache,

    _stop_tx: crossbeam_channel::Sender<()>,
}

// ---------------------------------------------------------------------------
// Messages
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
enum Message {
    Tick,
    ColormapSelected(ColormapKind),
    ScaleModeSelected(ScaleMode),
    ScaleMinChanged(f64),
    ScaleMaxChanged(f64),
    GammaChanged(f32),
    ResetGamma,
    ToggleAxes(bool),
    ToggleColorbar(bool),
    CursorMoved(Option<(u32, u32)>, Option<f64>),
}

// ---------------------------------------------------------------------------
// App logic
// ---------------------------------------------------------------------------

impl Viewer {
    fn new() -> (Self, iced::Task<Message>) {
        let (frame_tx, frame_rx) = bounded(2);
        let (stop_tx, stop_rx) = bounded(1);

        start_sim_capture(frame_tx, stop_rx, 1280, 960, 12, 30);

        (Self {
            frame_rx,
            current_frame: None,
            colormap: Colormap::new(ColormapKind::Viridis),
            scale_mode: ScaleMode::Auto,
            scale_min: 0.0,
            scale_max: 4095.0,
            gamma: 1.0,
            show_axes: true,
            show_colorbar: true,
            cursor_pixel: None,
            cursor_value: None,
            frame_times: Vec::new(),
            fps: 0.0,
            image_handle: None,
            frame_gen: 0,
            image_cache: canvas::Cache::default(),
            hist_cache: canvas::Cache::default(),
            _stop_tx: stop_tx,
        }, iced::Task::none())
    }

    fn theme(&self) -> Theme {
        Theme::Light
    }

    fn poll_frames(&mut self) -> bool {
        let mut got_new = false;
        loop {
            match self.frame_rx.try_recv() {
                Ok(frame) => {
                    self.current_frame = Some(frame);
                    got_new = true;
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
            }
        }
        if got_new {
            if let Some(frame) = &self.current_frame {
                match self.scale_mode {
                    ScaleMode::Auto => {
                        self.scale_min = frame.hist.data_min;
                        self.scale_max = frame.hist.data_max;
                    }
                    ScaleMode::Full => {
                        self.scale_min = 0.0;
                        self.scale_max = ((1u64 << frame.bit_depth) - 1) as f64;
                    }
                    ScaleMode::Manual => {}
                }
            }
            self.recolor_current_frame();

            let now = Instant::now();
            self.frame_times.push(now);
            while self.frame_times.len() > 30 {
                self.frame_times.remove(0);
            }
            if self.frame_times.len() >= 2 {
                let dt = self.frame_times.last().unwrap().duration_since(self.frame_times[0]);
                self.fps = (self.frame_times.len() - 1) as f64 / dt.as_secs_f64();
            }

            self.frame_gen += 1;
            self.image_cache.clear();
            self.hist_cache.clear();
        }
        got_new
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::Tick => {
                let _ = self.poll_frames();
            }
            Message::ColormapSelected(kind) => {
                self.colormap = Colormap::new(kind);
                self.recolor_current_frame();
                self.image_cache.clear();
            }
            Message::ScaleModeSelected(mode) => {
                self.scale_mode = mode;
                if let Some(frame) = &self.current_frame {
                    match mode {
                        ScaleMode::Auto => {
                            self.scale_min = frame.hist.data_min;
                            self.scale_max = frame.hist.data_max;
                        }
                        ScaleMode::Full => {
                            self.scale_min = 0.0;
                            self.scale_max = ((1u64 << frame.bit_depth) - 1) as f64;
                        }
                        ScaleMode::Manual => {}
                    }
                }
                self.recolor_current_frame();
                self.image_cache.clear();
                self.hist_cache.clear();
            }
            Message::ScaleMinChanged(v) => {
                self.scale_min = v;
                self.recolor_current_frame();
                self.image_cache.clear();
                self.hist_cache.clear();
            }
            Message::ScaleMaxChanged(v) => {
                self.scale_max = v;
                self.recolor_current_frame();
                self.image_cache.clear();
                self.hist_cache.clear();
            }
            Message::GammaChanged(g) => {
                self.gamma = g;
                self.recolor_current_frame();
                self.image_cache.clear();
            }
            Message::ResetGamma => {
                self.gamma = 1.0;
                self.recolor_current_frame();
                self.image_cache.clear();
            }
            Message::ToggleAxes(v) => {
                self.show_axes = v;
                self.image_cache.clear();
            }
            Message::ToggleColorbar(v) => {
                self.show_colorbar = v;
                self.image_cache.clear();
            }
            Message::CursorMoved(pixel, value) => {
                self.cursor_pixel = pixel;
                self.cursor_value = value;
            }
        }
    }

    fn recolor_current_frame(&mut self) {
        if let Some(frame) = &mut self.current_frame {
            frame.rgba = apply_colormap(
                &frame.mono,
                frame.width,
                frame.height,
                self.scale_min,
                self.scale_max,
                self.gamma,
                &self.colormap,
            );
            self.image_handle = Some(iced::widget::image::Handle::from_rgba(
                frame.width,
                frame.height,
                frame.rgba.clone(),
            ));
        }
    }

    fn subscription(&self) -> Subscription<Message> {
        iced::time::every(std::time::Duration::from_millis(33)).map(|_| Message::Tick)
    }

    fn view(&self) -> Element<Message> {
        let sidebar = self.sidebar_view();
        let main_content = self.main_view();

        row![
            container(scrollable(sidebar).spacing(4)).width(240).padding(10),
            main_content,
        ]
        .into()
    }

    fn sidebar_view(&self) -> Column<Message> {
        let mut col = column![].spacing(6);

        // Camera section
        col = col
            .push(text("Camera").size(16))
            .push(rule::horizontal(1))
            .push(text("Source: Simulated").size(13));

        col = col.push(rule::horizontal(1));

        // Display section
        col = col
            .push(text("Display").size(16))
            .push(rule::horizontal(1));

        // Colormap picker
        col = col.push(
            row![
                text("Colormap").size(13),
                pick_list(
                    ColormapKind::ALL.to_vec(),
                    Some(self.colormap.kind),
                    Message::ColormapSelected,
                )
                .text_size(13),
            ]
            .spacing(8)
            .align_y(iced::Alignment::Center),
        );

        // Gamma
        col = col.push(
            row![
                text("Gamma").size(13).width(55),
                slider(0.1..=5.0, self.gamma, Message::GammaChanged),
                text(format!("{:.2}", self.gamma)).size(12).width(40),
            ]
            .spacing(6)
            .align_y(iced::Alignment::Center),
        );
        col = col.push(button(text("Reset Gamma").size(12)).on_press(Message::ResetGamma));

        col = col.push(rule::horizontal(1));

        // Scale mode
        col = col.push(text("Scale Mode").size(13));
        col = col.push(
            row![
                scale_mode_button("Full", ScaleMode::Full, self.scale_mode),
                scale_mode_button("Auto", ScaleMode::Auto, self.scale_mode),
                scale_mode_button("Manual", ScaleMode::Manual, self.scale_mode),
            ]
            .spacing(4),
        );

        if self.scale_mode == ScaleMode::Manual {
            let max_range = if let Some(f) = &self.current_frame {
                ((1u64 << f.bit_depth) - 1) as f64
            } else {
                65535.0
            };
            col = col.push(
                row![
                    text("Min").size(12).width(30),
                    slider(0.0..=max_range, self.scale_min, |v| {
                        Message::ScaleMinChanged(v)
                    }),
                    text(format!("{:.0}", self.scale_min)).size(12).width(45),
                ]
                .spacing(4)
                .align_y(iced::Alignment::Center),
            );
            col = col.push(
                row![
                    text("Max").size(12).width(30),
                    slider(0.0..=max_range, self.scale_max, |v| {
                        Message::ScaleMaxChanged(v)
                    }),
                    text(format!("{:.0}", self.scale_max)).size(12).width(45),
                ]
                .spacing(4)
                .align_y(iced::Alignment::Center),
            );
        } else {
            col = col.push(
                text(format!(
                    "Range: {:.0} – {:.0}",
                    self.scale_min, self.scale_max
                ))
                .size(12),
            );
        }

        col = col.push(rule::horizontal(1));

        // Overlays
        col = col
            .push(
                checkbox(self.show_axes)
                    .label("Show Axes")
                    .on_toggle(Message::ToggleAxes)
                    .text_size(13),
            )
            .push(
                checkbox(self.show_colorbar)
                    .label("Show Colorbar")
                    .on_toggle(Message::ToggleColorbar)
                    .text_size(13),
            );

        col = col.push(rule::horizontal(1));

        // Statistics
        col = col
            .push(text("Statistics").size(16))
            .push(rule::horizontal(1))
            .push(text(format!("FPS: {:.1}", self.fps)).size(13));

        if let Some(frame) = &self.current_frame {
            col = col
                .push(text(format!("Size: {} x {}", frame.width, frame.height)).size(13))
                .push(text(format!("Bit depth: {}", frame.bit_depth)).size(13))
                .push(text(format!("Mean: {:.1}", frame.mean)).size(13))
                .push(text(format!("Std Dev: {:.1}", frame.stddev)).size(13));
        }

        col = col.push(rule::horizontal(1));

        // Cursor info
        if let (Some((px, py)), Some(val)) = (self.cursor_pixel, self.cursor_value) {
            col = col.push(text(format!("({}, {}) = {:.0}", px, py, val)).size(13));
        } else {
            col = col.push(text("---").size(13));
        }

        col
    }

    fn main_view(&self) -> Element<Message> {
        if self.current_frame.is_none() || self.image_handle.is_none() {
            return container(text("Waiting for frames..."))
                .center(Length::Fill)
                .into();
        }

        // Single canvas for everything — no cache (redraws every frame)
        let image_canvas = canvas(ImageCanvas {
            frame: self.current_frame.as_ref(),
            image_handle: self.image_handle.as_ref(),
            colormap: &self.colormap,
            scale_min: self.scale_min,
            scale_max: self.scale_max,
            show_axes: self.show_axes,
            show_colorbar: self.show_colorbar,
            cache: &self.image_cache,
        })
        .width(Length::Fill)
        .height(Length::FillPortion(3));

        // Histogram
        let hist_canvas = canvas(HistCanvas {
            frame: self.current_frame.as_ref(),
            scale_min: self.scale_min,
            scale_max: self.scale_max,
            cache: &self.hist_cache,
        })
        .width(Length::Fill)
        .height(Length::FillPortion(1));

        column![image_canvas, hist_canvas]
            .spacing(4)
            .padding(4)
            .into()
    }
}

impl Default for Viewer {
    fn default() -> Self {
        Self::new().0
    }
}

fn scale_mode_button(label: &str, mode: ScaleMode, current: ScaleMode) -> Element<'_, Message> {
    let btn = button(text(label).size(12));
    if mode == current {
        btn.into()
    } else {
        btn.on_press(Message::ScaleModeSelected(mode)).into()
    }
}

// ---------------------------------------------------------------------------
// ColormapKind display for pick_list
// ---------------------------------------------------------------------------

impl std::fmt::Display for ColormapKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

// ---------------------------------------------------------------------------
// Image canvas — draws directly each frame (no cache) to avoid flicker
// ---------------------------------------------------------------------------

use iced::widget::canvas::Image as CanvasImage;

struct ImageCanvas<'a> {
    frame: Option<&'a FrameData>,
    image_handle: Option<&'a iced::widget::image::Handle>,
    colormap: &'a Colormap,
    scale_min: f64,
    scale_max: f64,
    show_axes: bool,
    show_colorbar: bool,
    cache: &'a canvas::Cache,
}

impl<'a> canvas::Program<Message> for ImageCanvas<'a> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let geom = self.cache.draw(renderer, bounds.size(), |frame| {
        let Some(fd) = self.frame else { return };

        let axis_left = if self.show_axes { 55.0 } else { 0.0 };
        let axis_bottom = if self.show_axes { 35.0 } else { 0.0 };
        let cbar_right = if self.show_colorbar { 75.0 } else { 0.0 };

        let area_w = (bounds.width - axis_left - cbar_right).max(1.0);
        let area_h = (bounds.height - axis_bottom).max(1.0);

        let aspect = fd.width as f32 / fd.height as f32;
        let (img_w, img_h) = if area_w / area_h > aspect {
            (area_h * aspect, area_h)
        } else {
            (area_w, area_w / aspect)
        };

        let img_x = axis_left;
        let img_y = 0.0;

        // Draw the image
        if let Some(handle) = self.image_handle {
            let img_bounds =
                Rectangle::new(Point::new(img_x, img_y), Size::new(img_w, img_h));
            frame.draw_image(img_bounds, CanvasImage::new(handle.clone()));
        }

        let stroke = canvas::Stroke::default()
            .with_color(Color::from_rgb(0.4, 0.4, 0.4))
            .with_width(1.0);
        let text_color = Color::from_rgb(0.3, 0.3, 0.3);

        // Axes
        if self.show_axes {
            // Y-axis
            frame.stroke(
                &canvas::Path::line(
                    Point::new(img_x, img_y),
                    Point::new(img_x, img_y + img_h),
                ),
                stroke,
            );
            for i in 0..=5 {
                let frac = i as f32 / 5.0;
                let y = img_y + frac * img_h;
                let pval = (frac * fd.height as f32) as u32;
                frame.stroke(
                    &canvas::Path::line(
                        Point::new(img_x - 5.0, y),
                        Point::new(img_x, y),
                    ),
                    stroke,
                );
                frame.fill_text(canvas::Text {
                    content: format!("{}", pval),
                    position: Point::new(img_x - 8.0, y - 6.0),
                    color: text_color,
                    size: iced::Pixels(11.0),
                    align_x: iced::alignment::Horizontal::Right.into(),
                    ..canvas::Text::default()
                });
            }

            // X-axis
            frame.stroke(
                &canvas::Path::line(
                    Point::new(img_x, img_y + img_h),
                    Point::new(img_x + img_w, img_y + img_h),
                ),
                stroke,
            );
            for i in 0..=5 {
                let frac = i as f32 / 5.0;
                let x = img_x + frac * img_w;
                let pval = (frac * fd.width as f32) as u32;
                frame.stroke(
                    &canvas::Path::line(
                        Point::new(x, img_y + img_h),
                        Point::new(x, img_y + img_h + 5.0),
                    ),
                    stroke,
                );
                frame.fill_text(canvas::Text {
                    content: format!("{}", pval),
                    position: Point::new(x, img_y + img_h + 8.0),
                    color: text_color,
                    size: iced::Pixels(11.0),
                    align_x: iced::alignment::Horizontal::Center.into(),
                    ..canvas::Text::default()
                });
            }
        }

        // Colorbar
        if self.show_colorbar {
            let bar_x = img_x + img_w + 10.0;
            let bar_w = 15.0;
            let bar_h = img_h;
            let n_seg = 128;
            let seg_h = bar_h / n_seg as f32;

            for i in 0..n_seg {
                let t = 1.0 - i as f32 / n_seg as f32;
                let rgb = self.colormap.lookup(t);
                let color = Color::from_rgb8(rgb[0], rgb[1], rgb[2]);
                let rect = canvas::Path::rectangle(
                    Point::new(bar_x, img_y + i as f32 * seg_h),
                    Size::new(bar_w, seg_h + 0.5),
                );
                frame.fill(&rect, color);
            }

            let border = canvas::Path::rectangle(
                Point::new(bar_x, img_y),
                Size::new(bar_w, bar_h),
            );
            frame.stroke(
                &border,
                canvas::Stroke::default()
                    .with_color(Color::from_rgb(0.4, 0.4, 0.4))
                    .with_width(1.0),
            );

            let text_color = Color::from_rgb(0.3, 0.3, 0.3);
            for i in 0..=5 {
                let frac = i as f32 / 5.0;
                let val =
                    self.scale_max - frac as f64 * (self.scale_max - self.scale_min);
                let y = img_y + frac * bar_h;
                frame.fill_text(canvas::Text {
                    content: format!("{:.0}", val),
                    position: Point::new(bar_x + bar_w + 4.0, y - 6.0),
                    color: text_color,
                    size: iced::Pixels(11.0),
                    ..canvas::Text::default()
                });
            }
        }

        });
        vec![geom]
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        bounds: Rectangle,
        cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        if cursor.is_over(bounds) {
            mouse::Interaction::Crosshair
        } else {
            mouse::Interaction::default()
        }
    }
}

// ---------------------------------------------------------------------------
// Histogram canvas
// ---------------------------------------------------------------------------

struct HistCanvas<'a> {
    frame: Option<&'a FrameData>,
    scale_min: f64,
    scale_max: f64,
    cache: &'a canvas::Cache,
}

impl<'a> canvas::Program<Message> for HistCanvas<'a> {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<canvas::Geometry> {
        let geom = self.cache.draw(renderer, bounds.size(), |frame| {
            let Some(fd) = self.frame else { return };
            let hist = &fd.hist;
            if hist.counts.is_empty() {
                return;
            }

            let margin_left = 50.0f32;
            let margin_right = 10.0f32;
            let margin_top = 5.0f32;
            let margin_bottom = 25.0f32;

            let plot_w = (bounds.width - margin_left - margin_right).max(1.0);
            let plot_h = (bounds.height - margin_top - margin_bottom).max(1.0);

            let max_count = hist.counts.iter().copied().max().unwrap_or(1) as f32;
            let centers = hist.centers();
            let x_min = hist.data_min;
            let x_max = hist.data_max;
            let x_range = (x_max - x_min).max(1.0);

            // Background
            let bg = canvas::Path::rectangle(
                Point::new(margin_left, margin_top),
                Size::new(plot_w, plot_h),
            );
            frame.fill(&bg, Color::from_rgb(0.94, 0.94, 0.96));

            // Bars
            let bar_color = Color::from_rgb(0.4, 0.6, 1.0);
            let bin_w_data = if centers.len() > 1 {
                centers[1] - centers[0]
            } else {
                1.0
            };
            for (c, &count) in centers.iter().zip(hist.counts.iter()) {
                if count == 0 {
                    continue;
                }
                let x_frac = ((*c - x_min) / x_range) as f32;
                let w_frac = (bin_w_data / x_range) as f32;
                let h_frac = count as f32 / max_count;
                let bar = canvas::Path::rectangle(
                    Point::new(
                        margin_left + x_frac * plot_w - w_frac * plot_w * 0.5,
                        margin_top + plot_h * (1.0 - h_frac),
                    ),
                    Size::new((w_frac * plot_w).max(1.0), plot_h * h_frac),
                );
                frame.fill(&bar, bar_color);
            }

            // Scale range lines
            let line_color = Color::from_rgb(1.0, 0.3, 0.3);
            let line_stroke = canvas::Stroke::default()
                .with_color(line_color)
                .with_width(2.0);
            for &val in &[self.scale_min, self.scale_max] {
                let x_frac = ((val - x_min) / x_range) as f32;
                let x = margin_left + x_frac * plot_w;
                if x >= margin_left && x <= margin_left + plot_w {
                    frame.stroke(
                        &canvas::Path::line(
                            Point::new(x, margin_top),
                            Point::new(x, margin_top + plot_h),
                        ),
                        line_stroke,
                    );
                }
            }

            // X-axis with labels
            let axis_stroke = canvas::Stroke::default()
                .with_color(Color::from_rgb(0.4, 0.4, 0.4))
                .with_width(1.0);
            let text_color = Color::from_rgb(0.3, 0.3, 0.3);

            frame.stroke(
                &canvas::Path::line(
                    Point::new(margin_left, margin_top + plot_h),
                    Point::new(margin_left + plot_w, margin_top + plot_h),
                ),
                axis_stroke,
            );
            for i in 0..=4 {
                let frac = i as f32 / 4.0;
                let val = x_min + frac as f64 * x_range;
                let x = margin_left + frac * plot_w;
                frame.fill_text(canvas::Text {
                    content: format!("{:.0}", val),
                    position: Point::new(x, margin_top + plot_h + 4.0),
                    color: text_color,
                    size: iced::Pixels(11.0),
                    align_x: iced::alignment::Horizontal::Center.into(),
                    ..canvas::Text::default()
                });
            }

            // Y-axis line
            frame.stroke(
                &canvas::Path::line(
                    Point::new(margin_left, margin_top),
                    Point::new(margin_left, margin_top + plot_h),
                ),
                axis_stroke,
            );
        });

        vec![geom]
    }
}

// ---------------------------------------------------------------------------
// Colormap application
// ---------------------------------------------------------------------------

fn apply_colormap(
    mono: &[f64],
    width: u32,
    height: u32,
    scale_min: f64,
    scale_max: f64,
    gamma: f32,
    colormap: &Colormap,
) -> Vec<u8> {
    let npix = (width * height) as usize;
    let mut rgba = vec![255u8; npix * 4];

    let range = scale_max - scale_min;
    let inv_range = if range > 0.0 { 1.0 / range } else { 1.0 };
    let inv_gamma = if gamma != 0.0 { 1.0 / gamma } else { 1.0 };
    let apply_gamma = (gamma - 1.0).abs() > 1e-4;

    for (i, &val) in mono.iter().take(npix).enumerate() {
        let mut t = ((val - scale_min) * inv_range).clamp(0.0, 1.0) as f32;
        if apply_gamma {
            t = t.powf(inv_gamma);
        }
        let rgb = colormap.lookup(t);
        let off = i * 4;
        rgba[off] = rgb[0];
        rgba[off + 1] = rgb[1];
        rgba[off + 2] = rgb[2];
        rgba[off + 3] = 255;
    }
    rgba
}

// ---------------------------------------------------------------------------
// Camera thread
// ---------------------------------------------------------------------------

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
        let colormap = Colormap::new(ColormapKind::Viridis);

        loop {
            let t0 = Instant::now();
            if stop_rx.try_recv().is_ok() {
                break;
            }

            let img = cam.next_frame();
            let frame_data = process_image(img, bit_depth, &colormap);

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

fn process_image(img: DynamicImage, bit_depth: u8, colormap: &Colormap) -> FrameData {
    let width = img.width();
    let height = img.height();

    let mono: Vec<f64> = match &img {
        DynamicImage::ImageLuma8(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageLuma16(g) => g.as_raw().iter().map(|&v| v as f64).collect(),
        DynamicImage::ImageRgb8(rgb) => rgb
            .pixels()
            .map(|p| 0.299 * p[0] as f64 + 0.587 * p[1] as f64 + 0.114 * p[2] as f64)
            .collect(),
        _ => {
            let gray = img.to_luma8();
            gray.as_raw().iter().map(|&v| v as f64).collect()
        }
    };

    let hist = compute_histogram(&mono, 256);
    let (mean, stddev) = compute_stats(&mono);

    let rgba = apply_colormap(
        &mono,
        width,
        height,
        hist.data_min,
        hist.data_max,
        1.0,
        colormap,
    );

    FrameData {
        mono,
        width,
        height,
        rgba,
        hist,
        mean,
        stddev,
        bit_depth,
    }
}
