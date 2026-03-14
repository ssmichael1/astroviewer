use eframe::egui;
use super::*;

/// Mapping from slider position [0,1] to value and back.
enum SliderMapping {
    Linear,
    Logarithmic,
}

/// Core slider drawing with configurable mapping.
fn slider_core(
    ui: &mut egui::Ui,
    value: &mut f64,
    min: f64,
    max: f64,
    label: &str,
    fmt: &str,
    mapping: &SliderMapping,
) -> bool {
    let old = *value;

    ui.horizontal(|ui| {
        if !label.is_empty() {
            ui.label(label);
        }

        let avail = ui.available_width();
        let desired_width = if fmt == "none" {
            // No value text — fill available width
            avail.max(40.0)
        } else {
            avail.min(150.0).max(60.0) - 50.0
        };
        let handle_r = 7.0;
        let height = 20.0;
        let (rect, response) = ui.allocate_exact_size(
            egui::vec2(desired_width, height),
            egui::Sense::click_and_drag(),
        );

        let track_left = rect.min.x + handle_r;
        let track_right = rect.max.x - handle_r;
        let track_width = track_right - track_left;

        if response.dragged() || response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let t = ((pos.x - track_left) / track_width).clamp(0.0, 1.0) as f64;
                *value = match mapping {
                    SliderMapping::Linear => min + t * (max - min),
                    SliderMapping::Logarithmic => {
                        let log_min = min.max(1e-10).ln();
                        let log_max = max.ln();
                        (log_min + t * (log_max - log_min)).exp()
                    }
                };
            }
        }

        // Arrow key support when hovered
        if response.hovered() {
            let step = (max - min) * 0.01; // 1% per keypress
            if ui.input(|i| i.key_pressed(egui::Key::ArrowRight) || i.key_pressed(egui::Key::ArrowUp)) {
                *value = (*value + step).min(max);
            }
            if ui.input(|i| i.key_pressed(egui::Key::ArrowLeft) || i.key_pressed(egui::Key::ArrowDown)) {
                *value = (*value - step).max(min);
            }
        }

        let t = match mapping {
            SliderMapping::Linear => ((*value - min) / (max - min)).clamp(0.0, 1.0) as f32,
            SliderMapping::Logarithmic => {
                let log_min = min.max(1e-10).ln();
                let log_max = max.ln();
                (((*value).max(1e-10).ln() - log_min) / (log_max - log_min)).clamp(0.0, 1.0) as f32
            }
        };

        let painter = ui.painter_at(rect);
        let track_y = rect.center().y;
        let track_h = 4.0;
        let track_rect = egui::Rect::from_min_max(
            egui::pos2(track_left, track_y - track_h / 2.0),
            egui::pos2(track_right, track_y + track_h / 2.0),
        );
        painter.rect_filled(track_rect, egui::CornerRadius::same(2),
            egui::Color32::from_rgb(215, 216, 222));
        painter.hline(track_rect.x_range(), track_rect.min.y,
            egui::Stroke::new(1.0, egui::Color32::from_rgb(192, 193, 200)));

        let filled_rect = egui::Rect::from_min_max(
            track_rect.min,
            egui::pos2(track_left + t * track_width, track_rect.max.y),
        );
        if filled_rect.width() > 1.0 {
            painter.rect_filled(filled_rect, egui::CornerRadius::same(2), ACCENT);
        }

        let handle_x = track_left + t * track_width;
        let hc = egui::pos2(handle_x, track_y);
        let hovered = response.hovered() || response.dragged();

        painter.circle_filled(egui::pos2(hc.x, hc.y + 1.0), handle_r,
            egui::Color32::from_rgba_unmultiplied(0, 0, 0, 22));
        let hbg = if hovered { egui::Color32::from_rgb(252, 252, 254) } else { BG_RAISED };
        painter.circle_filled(hc, handle_r, hbg);
        let hborder = if response.dragged() { ACCENT_DARK }
            else if hovered { ACCENT }
            else { BORDER };
        painter.circle_stroke(hc, handle_r, egui::Stroke::new(1.5, hborder));
        painter.circle_filled(egui::pos2(hc.x - 1.0, hc.y - 2.0), 2.5,
            egui::Color32::from_rgba_unmultiplied(255, 255, 255, 120));

        if fmt != "none" {
            let display = match fmt {
                "d" => format!("{}", *value as i64),
                "1" => format!("{:.1}", *value),
                _ => format!("{:.2}", *value),
            };
            ui.label(egui::RichText::new(display).monospace().size(12.0));
        }
    });

    *value != old
}

/// Styled slider for f32 values (linear).
pub fn styled_slider(
    ui: &mut egui::Ui,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    label: &str,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, label, "2", &SliderMapping::Linear);
    *value = v as f32;
    changed
}

/// Styled logarithmic slider for f32 values.
pub fn styled_slider_log(
    ui: &mut egui::Ui,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    label: &str,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, label, "d", &SliderMapping::Logarithmic);
    *value = v.round() as f32;
    changed
}

/// Styled slider for f32 — no value text, fills available width.
pub fn styled_slider_bare(
    ui: &mut egui::Ui,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, "", "none", &SliderMapping::Linear);
    *value = v as f32;
    changed
}

/// Styled logarithmic slider — no value text, fills available width.
pub fn styled_slider_log_bare(
    ui: &mut egui::Ui,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, "", "none", &SliderMapping::Logarithmic);
    *value = v.round() as f32;
    changed
}

/// Styled slider for u32 values.
pub fn styled_slider_u32(
    ui: &mut egui::Ui,
    value: &mut u32,
    range: std::ops::RangeInclusive<u32>,
    label: &str,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, label, "d", &SliderMapping::Linear);
    *value = v.round() as u32;
    changed
}
