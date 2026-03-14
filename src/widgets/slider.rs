use eframe::egui;
use super::*;

/// Core slider drawing. Works on f64 internally, formats value with `fmt`.
fn slider_core(
    ui: &mut egui::Ui,
    value: &mut f64,
    min: f64,
    max: f64,
    label: &str,
    fmt: &str,
) -> bool {
    let old = *value;

    ui.horizontal(|ui| {
        if !label.is_empty() {
            ui.label(label);
        }

        let desired_width = ui.available_width().min(150.0).max(60.0) - 50.0;
        let handle_r = 7.0;
        let height = 20.0;
        let (rect, response) = ui.allocate_exact_size(
            egui::vec2(desired_width, height),
            egui::Sense::click_and_drag(),
        );

        // Inset the usable track so the handle circle doesn't clip at edges
        let track_left = rect.min.x + handle_r;
        let track_right = rect.max.x - handle_r;
        let track_width = track_right - track_left;

        if response.dragged() || response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let t = ((pos.x - track_left) / track_width).clamp(0.0, 1.0) as f64;
                *value = min + t * (max - min);
            }
        }

        let t = ((*value - min) / (max - min)).clamp(0.0, 1.0) as f32;
        let painter = ui.painter_at(rect);

        // Track
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

        // Filled portion
        let filled_rect = egui::Rect::from_min_max(
            track_rect.min,
            egui::pos2(track_left + t * track_width, track_rect.max.y),
        );
        if filled_rect.width() > 1.0 {
            painter.rect_filled(filled_rect, egui::CornerRadius::same(2), ACCENT);
        }

        // Handle
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

        // Value text
        let display = match fmt {
            "d" => format!("{}", *value as i64),
            "1" => format!("{:.1}", *value),
            _ => format!("{:.2}", *value),
        };
        ui.label(egui::RichText::new(display).monospace().size(12.0));
    });

    *value != old
}

/// Styled slider for f32 values.
pub fn styled_slider(
    ui: &mut egui::Ui,
    value: &mut f32,
    range: std::ops::RangeInclusive<f32>,
    label: &str,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, label, "2");
    *value = v as f32;
    changed
}

/// Styled slider for f64 values.
pub fn styled_slider_f64(
    ui: &mut egui::Ui,
    value: &mut f64,
    range: std::ops::RangeInclusive<f64>,
    label: &str,
) -> bool {
    slider_core(ui, value, *range.start(), *range.end(), label, "1")
}

/// Styled slider for u32 values.
pub fn styled_slider_u32(
    ui: &mut egui::Ui,
    value: &mut u32,
    range: std::ops::RangeInclusive<u32>,
    label: &str,
) -> bool {
    let mut v = *value as f64;
    let changed = slider_core(ui, &mut v, *range.start() as f64, *range.end() as f64, label, "d");
    *value = v.round() as u32;
    changed
}
