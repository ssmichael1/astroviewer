use std::sync::Arc;

use egui::{self, Color32, ColorImage, Pos2, Rect, Sense, Stroke, StrokeKind, TextureHandle, TextureOptions, Vec2};
use rayon::prelude::*;

use crate::colormaps::{Colormap, ColormapKind};
use crate::overlays::{self, OverlayItem};
use crate::pixels::Pixels;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TransferFn {
    Linear,
    Asinh,
}

impl TransferFn {
    pub const ALL: &'static [(TransferFn, &'static str)] = &[
        (TransferFn::Linear, "Linear"),
        (TransferFn::Asinh, "Asinh"),
    ];
}

/// Display parameters for the image viewer.
pub struct DisplayParams {
    pub scale_min: f32,
    pub scale_max: f32,
    pub gamma: f32,
    /// Asinh pivot in data units: the stretch is linear around this level
    /// (typically the sky background) and logarithmic above it.
    pub asinh_offset: f32,
    pub transfer: TransferFn,
    pub show_axes: bool,
    pub show_colorbar: bool,
    pub axes_text_color: Color32,
    pub axes_stroke_color: Color32,
    pub overlay_colors: overlays::OverlayColors,
    pub roi_color: Color32,
}

impl Default for DisplayParams {
    fn default() -> Self {
        Self {
            scale_min: 0.0,
            scale_max: 65535.0,
            gamma: 1.0,
            asinh_offset: 0.0,
            transfer: TransferFn::Linear,
            show_axes: true,
            show_colorbar: true,
            axes_text_color: Color32::from_rgb(51, 51, 51),
            axes_stroke_color: Color32::from_rgb(97, 97, 97),
            overlay_colors: overlays::OverlayColors::default(),
            roi_color: Color32::from_rgba_unmultiplied(255, 255, 0, 128),
        }
    }
}

/// Result of rendering the image widget — reports mouse interaction.
pub struct ImageViewResponse {
    pub hovered_pixel: Option<(u32, u32)>,
    pub hovered_value: Option<f32>,
}

/// Everything the recolored `image` is derived from. A repaint whose key equals the
/// cached one skips the O(pixels) rebuild and the GPU texture upload — the
/// egui repaint rate (mouse moves, 60 Hz during capture) is otherwise far
/// higher than the rate at which frames or display settings actually change.
#[derive(Clone, Copy, PartialEq)]
pub struct RgbaKey {
    /// Frame identity: the app bumps a serial whenever the displayed pixel
    /// buffer is replaced (dimensions alone can't distinguish frames).
    pub frame_serial: u64,
    pub width: u32,
    pub height: u32,
    /// Display bin factor (see [`display_bin_factor`]); depends on the
    /// viewport, so a resize can trigger a rebuild without a new frame.
    pub bin: u32,
    pub shading: Shading,
}

impl RgbaKey {
    pub fn new(frame_serial: u64, width: u32, height: u32, bin: u32, params: &DisplayParams, colormap: &Colormap) -> Self {
        Self { frame_serial, width, height, bin, shading: Shading::new(params, colormap) }
    }
}

/// Integer factor by which a `width`-pixel-wide frame is mean-binned before
/// recoloring when it is drawn `display_px` physical pixels wide.
///
/// Drawing a frame smaller than its pixel count is decimation; done by point
/// sampling the texture (what `TextureFilter::Nearest` does) it skips the
/// low-pass filter that sampling theory requires, so a 2-pixel-FWHM star is
/// picked up or dropped depending on where the sampling grid happens to
/// fall, and the sky keeps its full per-pixel noise. Averaging blocks of the
/// data first is that low-pass filter. The factor is the largest that leaves
/// the binned image at least as wide as the display, so the GPU's residual
/// minification is under 2×, a range in which bilinear filtering touches
/// every texel and nothing is dropped. At 1:1 or magnified the factor is 1.
pub fn display_bin_factor(width: u32, display_px: f32) -> u32 {
    if display_px <= 0.0 || width == 0 {
        return 1;
    }
    ((width as f32 / display_px).floor() as u32).max(1)
}

/// A sample type `bin_mean_shade` can accumulate: `u16` sums in `u32`
/// (integer adds vectorize; exact), `f32` in `f32`.
trait BinSample: Copy + Sync {
    type Acc: Copy + Default + Send + Sync;
    fn add(acc: Self::Acc, s: Self) -> Self::Acc;
    fn add_acc(a: Self::Acc, b: Self::Acc) -> Self::Acc;
    fn to_f32(acc: Self::Acc) -> f32;
}

impl BinSample for u16 {
    type Acc = u32;
    #[inline] fn add(acc: u32, s: u16) -> u32 { acc + s as u32 }
    #[inline] fn add_acc(a: u32, b: u32) -> u32 { a + b }
    #[inline] fn to_f32(acc: u32) -> f32 { acc as f32 }
}

impl BinSample for f32 {
    type Acc = f32;
    #[inline] fn add(acc: f32, s: f32) -> f32 { acc + s }
    #[inline] fn add_acc(a: f32, b: f32) -> f32 { a + b }
    #[inline] fn to_f32(acc: f32) -> f32 { acc }
}

/// Mean-bin `src` (row-major `w`×`h`) in `bin`×`bin` blocks and shade each
/// block mean into `dst`, row-major `ceil(w/bin)`×`ceil(h/bin)`. Blocks on
/// the right and bottom edges average only the pixels they cover.
///
/// Output rows go across the rayon pool. Each sums its `bin` source rows
/// column-wise first (one vectorized add per source pixel), then reduces the
/// column sums in runs of `bin`; the runtime block width only enters that
/// second, `bin`× smaller pass.
fn bin_mean_shade<T: BinSample>(
    src: &[T],
    w: usize,
    h: usize,
    bin: usize,
    dst: &mut [Color32],
    shade: impl Fn(f32) -> Color32 + Sync,
) {
    let bw = w.div_ceil(bin);
    let bh = h.div_ceil(bin);
    debug_assert_eq!(src.len(), w * h);
    debug_assert_eq!(dst.len(), bw * bh);
    dst.par_chunks_mut(bw).enumerate().for_each(|(oy, out)| {
        let y0 = oy * bin;
        let rows = &src[y0 * w..(y0 + bin).min(h) * w];
        let nrows = (rows.len() / w) as f32;
        let mut colsum = vec![T::Acc::default(); w];
        for row in rows.chunks_exact(w) {
            for (c, &s) in colsum.iter_mut().zip(row) {
                *c = T::add(*c, s);
            }
        }
        for (d, block) in out.iter_mut().zip(colsum.chunks(bin)) {
            let sum = block.iter().copied().fold(T::Acc::default(), T::add_acc);
            *d = shade(T::to_f32(sum) / (block.len() as f32 * nrows));
        }
    });
}

/// Everything that maps a pixel value to a color: display range, transfer
/// function (with its gamma/alpha and asinh pivot) and colormap. A pure
/// function of the value, so for integer frames it is evaluated once per
/// possible value into a table instead of once per pixel.
#[derive(Clone, Copy, PartialEq)]
pub struct Shading {
    scale_min: f32,
    scale_max: f32,
    gamma: f32,
    asinh_offset: f32,
    transfer: TransferFn,
    colormap: ColormapKind,
}

impl Shading {
    fn new(params: &DisplayParams, colormap: &Colormap) -> Self {
        Self {
            scale_min: params.scale_min,
            scale_max: params.scale_max,
            gamma: params.gamma,
            asinh_offset: params.asinh_offset,
            transfer: params.transfer,
            colormap: colormap.kind,
        }
    }

    /// The value → colormap-position mapping with its constants precomputed.
    fn transfer_map(&self) -> TransferMap {
        let range = self.scale_max - self.scale_min;
        let inv_range = if range > 0.0 { 1.0 / range } else { 1.0 };
        // Asinh with a pivot: s(t) = (asinh(α(t−o)) − asinh(−αo)) / (asinh(α(1−o)) − asinh(−αo))
        // where o is the offset normalized into the display range. asinh is odd,
        // so pixels below the pivot stay visible (compressed toward black) rather
        // than clipping; o = 0 reduces to the plain asinh(αt)/asinh(α) stretch.
        let asinh_alpha = self.gamma;
        let (asinh_o, asinh_lo, asinh_norm) = if matches!(self.transfer, TransferFn::Asinh) {
            let o = ((self.asinh_offset - self.scale_min) * inv_range).clamp(0.0, 1.0);
            let lo = (-asinh_alpha * o).asinh();
            let hi = (asinh_alpha * (1.0 - o)).asinh();
            let norm = if hi > lo { 1.0 / (hi - lo) } else { 1.0 };
            (o, lo, norm)
        } else {
            (0.0, 0.0, 1.0)
        };
        TransferMap {
            scale_min: self.scale_min,
            inv_range,
            inv_gamma: if self.gamma != 0.0 { 1.0 / self.gamma } else { 1.0 },
            apply_gamma: (self.gamma - 1.0).abs() > 1e-4,
            transfer: self.transfer,
            asinh_alpha,
            asinh_o,
            asinh_lo,
            asinh_norm,
        }
    }

    /// The per-value shader. `colormap` must be the one this shading was built from.
    fn shader<'a>(&self, colormap: &'a Colormap) -> impl Fn(f32) -> Color32 + Sync + 'a {
        let map = self.transfer_map();
        move |val: f32| {
            let rgb = colormap.lookup(map.t(val));
            Color32::from_rgb(rgb[0], rgb[1], rgb[2])
        }
    }
}

/// Pixel value → position along the colormap in [0, 1]: normalization into the
/// display range followed by the transfer function. Shared by the shader and
/// the colorbar, so the bar's labels sit exactly where the image maps them.
#[derive(Clone, Copy)]
struct TransferMap {
    scale_min: f32,
    inv_range: f32,
    inv_gamma: f32,
    apply_gamma: bool,
    transfer: TransferFn,
    asinh_alpha: f32,
    asinh_o: f32,
    asinh_lo: f32,
    asinh_norm: f32,
}

impl TransferMap {
    #[inline]
    fn t(&self, val: f32) -> f32 {
        let mut t = ((val - self.scale_min) * self.inv_range).clamp(0.0, 1.0);
        match self.transfer {
            TransferFn::Linear => {
                if self.apply_gamma { t = t.powf(self.inv_gamma); }
            }
            TransferFn::Asinh => {
                t = (((self.asinh_alpha * (t - self.asinh_o)).asinh() - self.asinh_lo) * self.asinh_norm).clamp(0.0, 1.0);
            }
        }
        t
    }
}

/// A "nice" tick step (1, 2 or 5 × 10ⁿ) giving about `target_ticks` ticks over `range`.
fn nice_step(range: f32, target_ticks: f32) -> f32 {
    let raw = (range / target_ticks.max(1.0)).max(f32::MIN_POSITIVE);
    let mag = 10f32.powf(raw.log10().floor());
    let norm = raw / mag;
    let nice = if norm < 1.5 { 1.0 } else if norm < 3.0 { 2.0 } else if norm < 7.0 { 5.0 } else { 10.0 };
    nice * mag
}

/// The next finer nice step below `step` (5→2→1→0.5…).
fn nice_step_below(step: f32) -> f32 {
    let mag = 10f32.powf(step.log10().floor());
    let norm = step / mag;
    if norm > 4.0 { 2.0 * mag } else if norm > 1.5 { mag } else { 0.5 * mag }
}

/// Format a tick value with just enough decimals for its step.
fn fmt_tick(v: f32, step: f32) -> String {
    let decimals = if step >= 1.0 { 0 } else { (-step.log10().floor()) as usize };
    format!("{:.*}", decimals, v)
}

/// Sampling for the main image texture. Magnification stays nearest so a
/// frame smaller than the viewport shows crisp pixels; minification is
/// bilinear, which with the bin factor keeping the residual ratio under 2×
/// weights every texel into some screen pixel instead of dropping most.
const DISPLAY_TEXTURE: TextureOptions = TextureOptions {
    magnification: egui::TextureFilter::Nearest,
    minification: egui::TextureFilter::Linear,
    wrap_mode: egui::TextureWrapMode::ClampToEdge,
    mipmap_mode: None,
};

/// Holds the texture and cached rendering state.
pub struct ImageViewer {
    texture: Option<TextureHandle>,
    /// The recolored frame. Shared with the texture manager rather than
    /// copied into it: uploading is a refcount bump, and by the next repaint
    /// the manager has consumed and dropped its handle, so `Arc::make_mut`
    /// writes the next frame in place. At 26 Mpix the copy this avoids was
    /// ~7 ms per frame.
    image: Arc<ColorImage>,
    /// Key the current `image`/texture were computed from.
    rgba_key: Option<RgbaKey>,
    /// Value → color table for `u16` frames (one entry per possible sample),
    /// and the shading it was built for. Rebuilt only when a display
    /// parameter changes (~1 ms); recoloring a frame is then a table lookup
    /// per pixel with no transcendental math.
    lut: Vec<Color32>,
    lut_shading: Option<Shading>,
    /// ROI drag state
    roi_start: Option<Pos2>,
    pub roi_rect: Option<[u32; 4]>,
}

impl ImageViewer {
    pub fn new() -> Self {
        Self {
            texture: None,
            image: Arc::new(ColorImage::filled([1, 1], Color32::BLACK)),
            rgba_key: None,
            lut: Vec::new(),
            lut_shading: None,
            roi_start: None,
            roi_rect: None,
        }
    }

    /// Render the image widget. `mono_data` is row-major pixel values;
    /// `frame_serial` identifies it so unchanged repaints skip recoloring.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        mono_data: &Pixels,
        frame_serial: u64,
        width: u32,
        height: u32,
        params: &DisplayParams,
        colormap: &Colormap,
        overlay_items: &[OverlayItem],
    ) -> ImageViewResponse {
        let mut response = ImageViewResponse {
            hovered_pixel: None,
            hovered_value: None,
        };

        if width == 0 || height == 0 || mono_data.is_empty() {
            ui.label("No image data");
            return response;
        }

        let available = ui.available_size();

        // Reserve space for axes and colorbar
        let axis_margin_left = if params.show_axes { 60.0 } else { 0.0 };
        let axis_margin_bottom = if params.show_axes { 40.0 } else { 0.0 };
        let colorbar_width = if params.show_colorbar { 80.0 } else { 0.0 };

        let image_area_w = (available.x - axis_margin_left - colorbar_width).max(1.0);
        let image_area_h = (available.y - axis_margin_bottom).max(1.0);

        // Fit image preserving aspect ratio
        let aspect = width as f32 / height as f32;
        let (display_w, display_h) = if image_area_w / image_area_h > aspect {
            (image_area_h * aspect, image_area_h)
        } else {
            (image_area_w, image_area_w / aspect)
        };

        let top_left = ui.cursor().min + Vec2::new(axis_margin_left, 0.0);
        let image_rect = Rect::from_min_size(top_left, Vec2::new(display_w, display_h));

        // Recolor and re-upload only when the frame, display params or bin
        // factor changed. The bin factor follows the drawn size in physical
        // pixels (points × scale factor on HiDPI), so a window resize that
        // crosses an integer ratio rebuilds; ordinary repaints do not.
        let bin = display_bin_factor(width, display_w * ui.ctx().pixels_per_point()).min(width).min(height).max(1);
        let key = RgbaKey::new(frame_serial, width, height, bin, params, colormap);
        if self.rgba_key != Some(key) || self.texture.is_none() {
            self.rgba_key = Some(key);
            self.update_rgba(mono_data, width, height, bin, params, colormap);
            match &mut self.texture {
                Some(tex) => tex.set(self.image.clone(), DISPLAY_TEXTURE),
                None => {
                    self.texture = Some(ui.ctx().load_texture(
                        "camera_image",
                        self.image.clone(),
                        DISPLAY_TEXTURE,
                    ));
                }
            }
        }
        // The binned texture covers ceil(w/bin)·bin source columns, up to
        // bin−1 more than the frame has. Map only the frame's extent onto
        // `image_rect` so hover, ROI and overlay geometry stay exact.
        let uv_max = {
            let [bw, bh] = self.image.size;
            Pos2::new(
                width as f32 / (bw as u32 * bin) as f32,
                height as f32 / (bh as u32 * bin) as f32,
            )
        };

        // Draw axes
        if params.show_axes {
            self.draw_axes(ui, image_rect, width, height, params);
        }

        // Draw the image with full interaction sense (hover + drag)
        if let Some(tex) = &self.texture {
            let resp = ui.allocate_rect(image_rect, Sense::click_and_drag());
            ui.painter().image(
                tex.id(),
                image_rect,
                Rect::from_min_max(Pos2::new(0.0, 0.0), uv_max),
                Color32::WHITE,
            );

            // Mouse interaction
            if let Some(pos) = resp.hover_pos() {
                let rel_x = (pos.x - image_rect.min.x) / display_w;
                let rel_y = (pos.y - image_rect.min.y) / display_h;
                if (0.0..=1.0).contains(&rel_x) && (0.0..=1.0).contains(&rel_y) {
                    let px = (rel_x * width as f32) as u32;
                    let py = (rel_y * height as f32) as u32;
                    let px = px.min(width - 1);
                    let py = py.min(height - 1);
                    response.hovered_pixel = Some((px, py));
                    let idx = (py * width + px) as usize;
                    response.hovered_value = mono_data.value_at(idx);
                }
            }

            // A stray left click used to clear the zoom ROI, which lost the
            // zoom too easily. The ROI now goes away only with Escape, the
            // zoom window's close button, or a new right-drag replacing it.

            // ROI selection (right-click drag)
            if resp.dragged_by(egui::PointerButton::Secondary) && self.roi_start.is_none() {
                self.roi_start = resp.hover_pos();
            }
            if resp.drag_stopped_by(egui::PointerButton::Secondary) {
                if let (Some(start), Some(end)) = (self.roi_start, resp.hover_pos()) {
                    let to_pixel = |pos: Pos2| -> (u32, u32) {
                        let rx = ((pos.x - image_rect.min.x) / display_w * width as f32) as u32;
                        let ry = ((pos.y - image_rect.min.y) / display_h * height as f32) as u32;
                        (rx.min(width - 1), ry.min(height - 1))
                    };
                    let (x0, y0) = to_pixel(start);
                    let (x1, y1) = to_pixel(end);
                    let roi = [x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1)];
                    self.roi_rect = Some(roi);
                }
                self.roi_start = None;
            }

            // Draw ROI rectangle overlay
            if let Some(roi) = self.roi_rect {
                let to_screen = |px: u32, py: u32| -> Pos2 {
                    Pos2::new(
                        image_rect.min.x + px as f32 / width as f32 * display_w,
                        image_rect.min.y + py as f32 / height as f32 * display_h,
                    )
                };
                let roi_screen = Rect::from_two_pos(
                    to_screen(roi[0], roi[1]),
                    to_screen(roi[2], roi[3]),
                );
                ui.painter().rect_stroke(
                    roi_screen,
                    0.0,
                    Stroke::new(2.0, Color32::YELLOW),
                    StrokeKind::Outside,
                );
            }

            // Draw active drag rectangle
            if let (Some(start), Some(current)) = (self.roi_start, ui.input(|i| i.pointer.hover_pos())) {
                let drag_rect = Rect::from_two_pos(start, current).intersect(image_rect);
                ui.painter().rect_stroke(
                    drag_rect,
                    0.0,
                    Stroke::new(1.0, params.roi_color),
                    StrokeKind::Outside,
                );
            }
        }

        // Draw overlays
        if !overlay_items.is_empty() {
            let img_cx = width as f32 / 2.0;
            let img_cy = height as f32 / 2.0;
            let scale_x = display_w / width as f32;
            let scale_y = display_h / height as f32;

            let to_screen = |ox: f32, oy: f32| -> Pos2 {
                // Convert from image-center origin to screen coords
                let px = ox + img_cx;
                let py = oy + img_cy;
                Pos2::new(
                    image_rect.min.x + px * scale_x,
                    image_rect.min.y + py * scale_y,
                )
            };

            let max_mass = overlay_items.iter().filter_map(|item| {
                if let OverlayItem::Centroid { mass, .. } = item { Some(*mass) } else { None }
            }).fold(0.0_f32, f32::max);

            overlays::draw_overlays(ui.painter(), overlay_items, to_screen, scale_x, max_mass, 1.0, &params.overlay_colors);
        }

        // Draw colorbar
        if params.show_colorbar {
            self.draw_colorbar(ui, image_rect, params, colormap);
        }

        // Advance cursor past the whole area we used
        let total_rect = Rect::from_min_size(
            ui.cursor().min,
            Vec2::new(
                axis_margin_left + display_w + colorbar_width,
                display_h + axis_margin_bottom,
            ),
        );
        ui.allocate_rect(total_rect, Sense::hover());

        response
    }

    /// Recolor the frame into `self.image`, mean-binned by `bin` (see
    /// [`display_bin_factor`]; 1 recolors every pixel).
    ///
    /// `u16` frames go through the per-value table; `f32` frames (float FITS,
    /// background-subtracted data) evaluate the shader per pixel, since their
    /// values are not indexable. Both paths use the same shader, so a `u16`
    /// frame and its `f32` widening produce byte-identical pixels at bin 1.
    /// Binned, a `u16` block mean is rounded to the nearest integer for the
    /// table lookup. Rows are distributed across the rayon pool: this runs on
    /// the UI thread and at full sensor resolution is the single largest cost
    /// per frame; binning shrinks the shaded and uploaded pixel count by
    /// `bin²`, while the pass over the source stays one read per pixel.
    fn update_rgba(
        &mut self,
        mono_data: &Pixels,
        width: u32,
        height: u32,
        bin: u32,
        params: &DisplayParams,
        colormap: &Colormap,
    ) {
        let (w, h) = (width as usize, height as usize);
        let bin = (bin.max(1) as usize).min(w.max(1)).min(h.max(1));
        let (bw, bh) = (w.div_ceil(bin), h.div_ceil(bin));
        let npix = bw * bh;
        let shading = Shading::new(params, colormap);
        let shade = shading.shader(colormap);

        if matches!(mono_data, Pixels::U16(_)) {
            self.ensure_lut(shading, &shade);
        }

        let img = Arc::make_mut(&mut self.image);
        if img.size != [bw, bh] || img.pixels.len() != npix {
            img.size = [bw, bh];
            img.source_size = Vec2::new(bw as f32, bh as f32);
            img.pixels.resize(npix, Color32::BLACK);
        }

        if bin > 1 {
            match mono_data {
                Pixels::U16(v) => {
                    let lut = &self.lut;
                    bin_mean_shade(v.as_slice(), w, h, bin, &mut img.pixels, |m| {
                        lut[((m + 0.5) as usize).min(u16::MAX as usize)]
                    });
                }
                Pixels::F32(v) => bin_mean_shade(v.as_slice(), w, h, bin, &mut img.pixels, &shade),
            }
            return;
        }

        match mono_data {
            Pixels::U16(v) => {
                let lut = &self.lut;
                img.pixels
                    .par_chunks_mut(w)
                    .zip(v.as_slice().par_chunks(w))
                    .for_each(|(dst, src)| {
                        for (d, &s) in dst.iter_mut().zip(src) {
                            *d = lut[s as usize];
                        }
                    });
            }
            Pixels::F32(v) => {
                img.pixels
                    .par_chunks_mut(w)
                    .zip(v.as_slice().par_chunks(w))
                    .for_each(|(dst, src)| {
                        for (d, &s) in dst.iter_mut().zip(src) {
                            *d = shade(s);
                        }
                    });
            }
        }
    }

    /// Rebuild the `u16` value → color table if `shading` differs from the one
    /// it was built for.
    fn ensure_lut(&mut self, shading: Shading, shade: &(impl Fn(f32) -> Color32 + Sync)) {
        if self.lut_shading != Some(shading) || self.lut.len() != 1 << 16 {
            self.lut = (0..=u16::MAX).map(|x| shade(x as f32)).collect();
            self.lut_shading = Some(shading);
        }
    }

    fn draw_axes(&self, ui: &mut egui::Ui, image_rect: Rect, width: u32, height: u32, params: &DisplayParams) {
        let painter = ui.painter();
        let stroke = Stroke::new(1.0, params.axes_stroke_color);
        let text_color = params.axes_text_color;
        let font = egui::FontId::monospace(13.0);

        // Ticks at round pixel coordinates (multiples of 1/2/5 × 10ⁿ), spaced
        // roughly 80 screen px apart, rather than at fixed fractions of the
        // edge that land on values like 1228 and 2457.
        let ticks = |extent: u32, screen_len: f32| -> (Vec<u32>, f32) {
            let target = (screen_len / 80.0).clamp(2.0, 12.0);
            let step = nice_step(extent as f32, target).max(1.0);
            let mut v = Vec::new();
            let mut k = 0.0f32;
            while k * step <= extent as f32 {
                v.push((k * step) as u32);
                k += 1.0;
            }
            (v, step)
        };

        // Y-axis (left side)
        let (y_ticks, _) = ticks(height, image_rect.height());
        for &pixel_val in &y_ticks {
            let y = image_rect.min.y + pixel_val as f32 / height as f32 * image_rect.height();
            painter.line_segment([Pos2::new(image_rect.min.x - 5.0, y), Pos2::new(image_rect.min.x, y)], stroke);
            painter.text(
                Pos2::new(image_rect.min.x - 8.0, y),
                egui::Align2::RIGHT_CENTER,
                format!("{}", pixel_val),
                font.clone(),
                text_color,
            );
        }

        // X-axis (bottom)
        let (x_ticks, _) = ticks(width, image_rect.width());
        for &pixel_val in &x_ticks {
            let x = image_rect.min.x + pixel_val as f32 / width as f32 * image_rect.width();
            painter.line_segment([Pos2::new(x, image_rect.max.y), Pos2::new(x, image_rect.max.y + 5.0)], stroke);
            painter.text(
                Pos2::new(x, image_rect.max.y + 8.0),
                egui::Align2::CENTER_TOP,
                format!("{}", pixel_val),
                font.clone(),
                text_color,
            );
        }

        // Axis lines
        painter.line_segment([image_rect.left_bottom(), image_rect.left_top()], stroke);
        painter.line_segment([image_rect.left_bottom(), image_rect.right_bottom()], stroke);
    }

    fn draw_colorbar(
        &self,
        ui: &mut egui::Ui,
        image_rect: Rect,
        params: &DisplayParams,
        colormap: &Colormap,
    ) {
        let painter = ui.painter();
        let bar_width = 15.0;
        let gap = 8.0;
        let bar_x = image_rect.max.x + gap;
        let bar_top = image_rect.min.y;
        let bar_height = image_rect.height();

        // The bar is the colormap itself, uniform in colormap position t; the
        // labels are placed by pushing round data values through the same
        // transfer the image uses, so under gamma or asinh they land exactly
        // where those values appear in the bar (dense where the stretch
        // expands the range, sparse where it compresses it).
        let n_segments = 128;
        let seg_height = bar_height / n_segments as f32;
        for i in 0..n_segments {
            let t = 1.0 - i as f32 / n_segments as f32;
            let rgb = colormap.lookup(t);
            let color = Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
            let y = bar_top + i as f32 * seg_height;
            let rect = Rect::from_min_size(Pos2::new(bar_x, y), Vec2::new(bar_width, seg_height));
            painter.rect_filled(rect, 0.0, color);
        }

        // Border
        let bar_rect = Rect::from_min_size(Pos2::new(bar_x, bar_top), Vec2::new(bar_width, bar_height));
        painter.rect_stroke(bar_rect, 0.0, Stroke::new(1.0, params.axes_stroke_color), StrokeKind::Outside);

        let font = egui::FontId::monospace(13.0);
        let text_color = params.axes_text_color;
        let tick_stroke = Stroke::new(1.0, params.axes_stroke_color);
        let label_x = bar_x + bar_width + 4.0;
        let (vmin, vmax) = (params.scale_min, params.scale_max);
        let range = vmax - vmin;
        if !(range > 0.0) || bar_height <= 0.0 {
            return;
        }
        let map = Shading::new(params, colormap).transfer_map();
        let y_of = |v: f32| bar_top + (1.0 - map.t(v)) * bar_height;

        // Ends first, then progressively finer round values wherever they fit
        // without crowding a label already placed.
        const MIN_GAP: f32 = 16.0;
        let mut labels: Vec<(f32, String)> = vec![
            (y_of(vmin), fmt_tick(vmin, 1.0)),
            (y_of(vmax), fmt_tick(vmax, 1.0)),
        ];
        let mut step = nice_step(range, 2.0);
        let finest = (range / (bar_height / MIN_GAP).max(1.0) / 4.0).max(range * 1e-4);
        while step >= finest {
            let k0 = (vmin / step).ceil() as i64;
            let k1 = (vmax / step).floor() as i64;
            for k in k0..=k1 {
                let v = k as f32 * step;
                if (v - vmin).abs() < step * 0.5 || (vmax - v).abs() < step * 0.5 {
                    continue;
                }
                let y = y_of(v);
                if labels.iter().all(|(ly, _)| (ly - y).abs() >= MIN_GAP) {
                    labels.push((y, fmt_tick(v, step)));
                }
            }
            step = nice_step_below(step);
            if step <= 0.0 { break; }
        }

        for (y, text) in labels {
            painter.line_segment([Pos2::new(bar_x + bar_width, y), Pos2::new(bar_x + bar_width + 3.0, y)], tick_stroke);
            painter.text(Pos2::new(label_x, y), egui::Align2::LEFT_CENTER, text, font.clone(), text_color);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::colormaps::ColormapKind;
    use std::sync::Arc;

    /// Recoloring a `u16` frame must produce byte-for-byte identical RGBA to the
    /// same pixels widened to `f32`. This is the display half of the dual-type
    /// equivalence guarantee: a `u16` GigE frame looks exactly as it did when
    /// every frame was stored as `f32`.
    fn assert_recolor_identical(raw: &[u16], w: u32, h: u32, params: &DisplayParams, kind: ColormapKind) {
        let cmap = Colormap::new(kind);
        let u = Pixels::U16(Arc::new(raw.to_vec()));
        let f = Pixels::F32(Arc::new(raw.iter().map(|&x| x as f32).collect()));

        let mut vu = ImageViewer::new();
        vu.update_rgba(&u, w, h, 1, params, &cmap);
        let px_u = vu.image.pixels.clone();

        let mut vf = ImageViewer::new();
        vf.update_rgba(&f, w, h, 1, params, &cmap);

        assert_eq!(px_u, vf.image.pixels, "u16 vs f32 pixels differ for {kind:?}");
        assert_eq!(vu.image.size, [w as usize, h as usize]);
    }

    /// Timing of the recolor paths at a 26 Mpix (SC571CC) frame. Ignored by
    /// default; run with
    /// `cargo test --release recolor_bench -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn recolor_bench_26mpix() {
        let (w, h) = (6224u32, 4168u32);
        let n = (w * h) as usize;
        let raw: Vec<u16> = (0..n).map(|i| ((i.wrapping_mul(2654435761) >> 7) & 0x0fff) as u16).collect();
        let u = Pixels::U16(Arc::new(raw.clone()));
        let f = Pixels::F32(Arc::new(raw.iter().map(|&x| x as f32).collect()));
        let cmap = Colormap::new(ColormapKind::Viridis);
        let cases = [
            ("linear g=1", DisplayParams { scale_min: 10.0, scale_max: 3000.0, ..Default::default() }),
            ("gamma 2.2", DisplayParams { scale_min: 10.0, scale_max: 3000.0, gamma: 2.2, ..Default::default() }),
            ("asinh a=100", DisplayParams { scale_min: 10.0, scale_max: 3000.0, gamma: 100.0, asinh_offset: 300.0, transfer: TransferFn::Asinh, ..Default::default() }),
        ];
        let mut v = ImageViewer::new();
        for (name, p) in &cases {
            for (label, px) in [("u16", &u), ("f32", &f)] {
                // First call includes the LUT build / buffer allocation; the
                // second is the steady state.
                for bin in [1u32, 3] {
                    v.update_rgba(px, w, h, bin, p, &cmap);
                    let t = std::time::Instant::now();
                    v.rgba_key = None;
                    v.update_rgba(px, w, h, bin, p, &cmap);
                    println!("{name:12} {label} bin {bin}: {:6.1} ms", t.elapsed().as_secs_f64() * 1e3);
                }
            }
        }
    }

    /// The bin factor keeps the binned image between 1× and 2× the drawn
    /// width and never bins a frame drawn at or above 1:1.
    #[test]
    fn bin_factor_brackets_display_width() {
        assert_eq!(display_bin_factor(6224, 6224.0), 1);
        assert_eq!(display_bin_factor(6224, 9000.0), 1);
        assert_eq!(display_bin_factor(6224, 2000.0), 3);
        assert_eq!(display_bin_factor(6224, 1556.0), 4);
        assert_eq!(display_bin_factor(6224, 0.0), 1);
        assert_eq!(display_bin_factor(0, 100.0), 1);
        for w in [640u32, 1920, 6224] {
            for px in [100.0f32, 333.0, 1000.0, 1919.0, 4000.0] {
                let b = display_bin_factor(w, px);
                let bw = w.div_ceil(b) as f32;
                assert!(bw >= px.min(w as f32), "w={w} px={px} bin={b} bw={bw}");
                assert!(bw < 2.0 * px || b == 1, "w={w} px={px} bin={b} bw={bw}");
            }
        }
    }

    /// Binned recoloring shades exact block means, with ragged right and
    /// bottom edges averaging only the pixels they cover; `u16` frames round
    /// the mean to the nearest table entry.
    #[test]
    fn binned_recolor_shades_block_means() {
        let (w, h, bin) = (7u32, 5u32, 3usize);
        let raw: Vec<u16> = (0..(w * h)).map(|i| (i * 977 % 4001) as u16).collect();
        let params = DisplayParams { scale_min: 0.0, scale_max: 4000.0, gamma: 1.7, ..Default::default() };
        let cmap = Colormap::new(ColormapKind::Inferno);
        let shade = Shading::new(&params, &cmap).shader(&cmap);

        let (bw, bh) = ((w as usize).div_ceil(bin), (h as usize).div_ceil(bin));
        let mut means = vec![0.0f32; bw * bh];
        for oy in 0..bh {
            for ox in 0..bw {
                let (mut sum, mut n) = (0.0f32, 0.0f32);
                for y in oy * bin..((oy + 1) * bin).min(h as usize) {
                    for x in ox * bin..((ox + 1) * bin).min(w as usize) {
                        sum += raw[y * w as usize + x] as f32;
                        n += 1.0;
                    }
                }
                means[oy * bw + ox] = sum / n;
            }
        }

        let mut vf = ImageViewer::new();
        vf.update_rgba(&Pixels::F32(Arc::new(raw.iter().map(|&x| x as f32).collect())), w, h, bin as u32, &params, &cmap);
        assert_eq!(vf.image.size, [bw, bh]);
        let expect_f: Vec<Color32> = means.iter().map(|&m| shade(m)).collect();
        assert_eq!(vf.image.pixels, expect_f);

        let mut vu = ImageViewer::new();
        vu.update_rgba(&Pixels::U16(Arc::new(raw.clone())), w, h, bin as u32, &params, &cmap);
        let expect_u: Vec<Color32> = means.iter().map(|&m| shade(m.round())).collect();
        assert_eq!(vu.image.pixels, expect_u);

        // Bin 1 is the unbinned path, byte for byte.
        let mut v1 = ImageViewer::new();
        v1.update_rgba(&Pixels::U16(Arc::new(raw.clone())), w, h, 1, &params, &cmap);
        let expect_1: Vec<Color32> = raw.iter().map(|&x| shade(x as f32)).collect();
        assert_eq!(v1.image.pixels, expect_1);
    }

    #[test]
    fn nice_steps_are_round() {
        assert_eq!(nice_step(6224.0, 8.0), 1000.0);
        assert_eq!(nice_step(1080.0, 5.0), 200.0);
        assert_eq!(nice_step(65535.0, 2.0), 50000.0);
        assert_eq!(nice_step_below(50000.0), 20000.0);
        assert_eq!(nice_step_below(20000.0), 10000.0);
        assert_eq!(nice_step_below(10000.0), 5000.0);
        assert_eq!(fmt_tick(1234.0, 1.0), "1234");
        assert_eq!(fmt_tick(0.25, 0.05), "0.25");
    }

    /// Under asinh the colorbar must place a value where the image maps it:
    /// the transfer map is monotonic and hits both ends exactly.
    #[test]
    fn transfer_map_is_monotonic_and_spans_unit_range() {
        let cmap = Colormap::new(ColormapKind::Grayscale);
        let p = DisplayParams { scale_min: 100.0, scale_max: 5000.0, gamma: 300.0, asinh_offset: 400.0, transfer: TransferFn::Asinh, ..Default::default() };
        let map = Shading::new(&p, &cmap).transfer_map();
        assert_eq!(map.t(100.0), 0.0);
        assert_eq!(map.t(5000.0), 1.0);
        let mut last = -1.0;
        for i in 0..=100 {
            let t = map.t(100.0 + i as f32 * 49.0);
            assert!(t >= last, "not monotonic at {i}");
            last = t;
        }
        // Strong asinh: the low half of the range covers most of the bar.
        assert!(map.t(600.0) > 0.5);
    }

    #[test]
    fn recolor_u16_matches_f32_all_transfers() {
        let (w, h) = (16u32, 16u32);
        let raw: Vec<u16> = (0..(w * h)).map(|i| ((i * 271) % 65536) as u16).collect();

        for kind in [ColormapKind::Grayscale, ColormapKind::Viridis, ColormapKind::Inferno] {
            // Linear, gamma == 1.
            let mut p = DisplayParams { scale_min: 0.0, scale_max: 65535.0, gamma: 1.0,
                transfer: TransferFn::Linear, ..Default::default() };
            assert_recolor_identical(&raw, w, h, &p, kind);

            // Linear with gamma.
            p.gamma = 2.2;
            assert_recolor_identical(&raw, w, h, &p, kind);

            // Asinh with a nonzero pivot and a tighter window.
            p.transfer = TransferFn::Asinh;
            p.gamma = 5.0;
            p.scale_min = 1000.0;
            p.scale_max = 40000.0;
            p.asinh_offset = 3000.0;
            assert_recolor_identical(&raw, w, h, &p, kind);
        }
    }
}
