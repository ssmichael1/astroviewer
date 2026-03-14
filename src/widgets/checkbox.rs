use eframe::egui;
use super::*;

/// A contemporary styled checkbox with rounded toggle look.
pub fn styled_checkbox(ui: &mut egui::Ui, checked: &mut bool, label: &str) -> bool {
    let old = *checked;
    let height = 20.0;
    let box_size = 18.0;
    let r = egui::CornerRadius::same(5);
    let font = egui::FontId::proportional(13.0);

    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), TEXT_PRIMARY);
    let total_width = box_size + 8.0 + galley.size().x;

    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(total_width, height),
        egui::Sense::click(),
    );

    if response.clicked() {
        *checked = !*checked;
    }

    let painter = ui.painter();
    let hovered = response.hovered();

    let box_rect = egui::Rect::from_min_size(
        egui::pos2(rect.min.x, rect.center().y - box_size / 2.0),
        egui::vec2(box_size, box_size),
    );

    if *checked {
        painter.rect_filled(box_rect, r, ACCENT);
        painter.hline(
            egui::Rangef::new(box_rect.min.x + 2.0, box_rect.max.x - 2.0),
            box_rect.min.y + 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(0, 0, 0, 25)),
        );
        let cx = box_rect.center().x;
        let cy = box_rect.center().y;
        let check_stroke = egui::Stroke::new(2.2, egui::Color32::WHITE);
        painter.line_segment(
            [egui::pos2(cx - 4.0, cy), egui::pos2(cx - 1.0, cy + 3.5)],
            check_stroke,
        );
        painter.line_segment(
            [egui::pos2(cx - 1.0, cy + 3.5), egui::pos2(cx + 5.0, cy - 3.5)],
            check_stroke,
        );
    } else {
        let bg = if hovered { BG_HOVER } else { egui::Color32::WHITE };
        let bc = if hovered { ACCENT_LIGHT } else { BORDER };
        painter.rect_filled(box_rect, r, bg);
        painter.rect_stroke(box_rect, r, egui::Stroke::new(1.5, bc), egui::StrokeKind::Inside);
        painter.hline(
            egui::Rangef::new(box_rect.min.x + 2.0, box_rect.max.x - 2.0),
            box_rect.min.y + 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 160)),
        );
    }

    let text_color = if hovered { ACCENT } else { TEXT_PRIMARY };
    painter.text(
        egui::pos2(box_rect.max.x + 8.0, rect.center().y),
        egui::Align2::LEFT_CENTER,
        label,
        font,
        text_color,
    );

    *checked != old
}
