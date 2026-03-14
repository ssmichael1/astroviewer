use egui::{self, Color32, Pos2, Rect, Sense, Stroke, StrokeKind, TextureHandle, TextureOptions, Vec2};

use crate::colormaps::Colormap;
use crate::overlays::{self, OverlayItem};

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
    pub transfer: TransferFn,
    pub show_axes: bool,
    pub show_colorbar: bool,
}

impl Default for DisplayParams {
    fn default() -> Self {
        Self {
            scale_min: 0.0,
            scale_max: 65535.0,
            gamma: 1.0,
            transfer: TransferFn::Linear,
            show_axes: true,
            show_colorbar: true,
        }
    }
}

/// Result of rendering the image widget — reports mouse interaction.
pub struct ImageViewResponse {
    pub hovered_pixel: Option<(u32, u32)>,
    pub hovered_value: Option<f32>,
}

/// Holds the texture and cached rendering state.
pub struct ImageViewer {
    texture: Option<TextureHandle>,
    rgba_buf: Vec<u8>,
    cached_width: u32,
    cached_height: u32,
    /// ROI drag state
    roi_start: Option<Pos2>,
    pub roi_rect: Option<[u32; 4]>,
}

impl ImageViewer {
    pub fn new() -> Self {
        Self {
            texture: None,
            rgba_buf: Vec::new(),
            cached_width: 0,
            cached_height: 0,
            roi_start: None,
            roi_rect: None,
        }
    }

    /// Render the image widget. `mono_data` is row-major pixel values.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        mono_data: &[f32],
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

        // Rebuild RGBA buffer
        self.update_rgba(mono_data, width, height, params, colormap);

        // Upload texture
        let color_image = egui::ColorImage::from_rgba_unmultiplied(
            [width as usize, height as usize],
            &self.rgba_buf,
        );
        match &mut self.texture {
            Some(tex) => tex.set(color_image, TextureOptions::NEAREST),
            None => {
                self.texture = Some(ui.ctx().load_texture(
                    "camera_image",
                    color_image,
                    TextureOptions::NEAREST,
                ));
            }
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

        // Draw axes
        if params.show_axes {
            self.draw_axes(ui, image_rect, width, height);
        }

        // Draw the image with full interaction sense (hover + drag)
        if let Some(tex) = &self.texture {
            let resp = ui.allocate_rect(image_rect, Sense::click_and_drag());
            ui.painter().image(
                tex.id(),
                image_rect,
                Rect::from_min_max(Pos2::new(0.0, 0.0), Pos2::new(1.0, 1.0)),
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
                    if idx < mono_data.len() {
                        response.hovered_value = Some(mono_data[idx]);
                    }
                }
            }

            // Left-click on image clears zoom ROI
            if resp.clicked_by(egui::PointerButton::Primary) && self.roi_rect.is_some() {
                self.roi_rect = None;
            }

            // ROI selection (right-click drag)
            if resp.dragged_by(egui::PointerButton::Secondary) {
                if self.roi_start.is_none() {
                    self.roi_start = resp.hover_pos();
                }
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
                    Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 0, 128)),
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

            overlays::draw_overlays(ui.painter(), overlay_items, to_screen, scale_x, max_mass, 1.0);
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

    fn update_rgba(
        &mut self,
        mono_data: &[f32],
        width: u32,
        height: u32,
        params: &DisplayParams,
        colormap: &Colormap,
    ) {
        let npix = (width * height) as usize;
        if self.rgba_buf.len() != npix * 4 || self.cached_width != width || self.cached_height != height
        {
            self.rgba_buf.resize(npix * 4, 255);
            self.cached_width = width;
            self.cached_height = height;
        }

        let range = params.scale_max - params.scale_min;
        let inv_range = if range > 0.0 { 1.0 / range } else { 1.0 };
        let inv_gamma = if params.gamma != 0.0 { 1.0 / params.gamma } else { 1.0 };
        let apply_gamma = (params.gamma - 1.0).abs() > 1e-4;

        let asinh_alpha = params.gamma;
        let asinh_norm = if matches!(params.transfer, TransferFn::Asinh) {
            let v = asinh_alpha.asinh();
            if v > 0.0 { 1.0 / v } else { 1.0 }
        } else {
            1.0
        };

        for (i, &val) in mono_data.iter().take(npix).enumerate() {
            let mut t = ((val - params.scale_min) * inv_range).clamp(0.0, 1.0);
            match params.transfer {
                TransferFn::Linear => {
                    if apply_gamma { t = t.powf(inv_gamma); }
                }
                TransferFn::Asinh => {
                    t = ((asinh_alpha * t).asinh() * asinh_norm).clamp(0.0, 1.0);
                }
            }
            let rgb = colormap.lookup(t);
            let off = i * 4;
            self.rgba_buf[off] = rgb[0];
            self.rgba_buf[off + 1] = rgb[1];
            self.rgba_buf[off + 2] = rgb[2];
            self.rgba_buf[off + 3] = 255;
        }
    }

    fn draw_axes(&self, ui: &mut egui::Ui, image_rect: Rect, width: u32, height: u32) {
        let painter = ui.painter();
        let stroke = Stroke::new(1.0, Color32::from_rgb(97, 97, 97));
        let text_color = Color32::from_rgb(51, 51, 51);
        let font = egui::FontId::monospace(13.0);

        // Y-axis (left side)
        let num_y_ticks = 5;
        for i in 0..=num_y_ticks {
            let frac = i as f32 / num_y_ticks as f32;
            let pixel_val = (frac * height as f32) as u32;
            let y = image_rect.min.y + frac * image_rect.height();
            let tick_start = Pos2::new(image_rect.min.x - 5.0, y);
            let tick_end = Pos2::new(image_rect.min.x, y);
            painter.line_segment([tick_start, tick_end], stroke);
            painter.text(
                Pos2::new(image_rect.min.x - 8.0, y),
                egui::Align2::RIGHT_CENTER,
                format!("{}", pixel_val),
                font.clone(),
                text_color,
            );
        }

        // X-axis (bottom)
        let num_x_ticks = 5;
        for i in 0..=num_x_ticks {
            let frac = i as f32 / num_x_ticks as f32;
            let pixel_val = (frac * width as f32) as u32;
            let x = image_rect.min.x + frac * image_rect.width();
            let tick_start = Pos2::new(x, image_rect.max.y);
            let tick_end = Pos2::new(x, image_rect.max.y + 5.0);
            painter.line_segment([tick_start, tick_end], stroke);
            painter.text(
                Pos2::new(x, image_rect.max.y + 8.0),
                egui::Align2::CENTER_TOP,
                format!("{}", pixel_val),
                font.clone(),
                text_color,
            );
        }

        // Axis lines
        painter.line_segment(
            [image_rect.left_bottom(), image_rect.left_top()],
            stroke,
        );
        painter.line_segment(
            [image_rect.left_bottom(), image_rect.right_bottom()],
            stroke,
        );
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

        // Draw gradient
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
        let bar_rect =
            Rect::from_min_size(Pos2::new(bar_x, bar_top), Vec2::new(bar_width, bar_height));
        painter.rect_stroke(bar_rect, 0.0, Stroke::new(1.0, Color32::from_rgb(97, 97, 97)), StrokeKind::Outside);

        // Scale labels
        let font = egui::FontId::monospace(13.0);
        let text_color = Color32::from_rgb(51, 51, 51);
        let label_x = bar_x + bar_width + 4.0;
        let num_labels = 5;
        for i in 0..=num_labels {
            let frac = i as f32 / num_labels as f32;
            let val = params.scale_max - frac * (params.scale_max - params.scale_min);
            let y = bar_top + frac * bar_height;
            painter.text(
                Pos2::new(label_x, y),
                egui::Align2::LEFT_CENTER,
                format!("{:.0}", val),
                font.clone(),
                text_color,
            );
        }
    }
}
