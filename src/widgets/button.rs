use eframe::egui;
use super::*;

/// A contemporary styled push button.
pub fn styled_button(ui: &mut egui::Ui, label: &str) -> bool {
    let height = 26.0;
    let font = egui::FontId::proportional(12.0);
    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), egui::Color32::BLACK);
    let width = galley.size().x + 24.0;

    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(width, height),
        egui::Sense::click(),
    );

    let painter = ui.painter();
    let hovered = response.hovered();
    let pressed = response.is_pointer_button_down_on();

    let bg = if pressed {
        egui::Color32::from_rgb(232, 234, 240)
    } else if hovered {
        BG_HOVER
    } else {
        BG_RAISED
    };

    let border_c = if hovered { BORDER_HOVER } else { BORDER };

    painter.rect_filled(rect, egui::CornerRadius::same(5), bg);
    painter.rect_stroke(rect, egui::CornerRadius::same(5), egui::Stroke::new(1.0, border_c), egui::StrokeKind::Inside);

    if !pressed {
        painter.hline(
            rect.shrink2(egui::vec2(2.0, 0.0)).x_range(),
            rect.max.y,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(0, 0, 0, 12)),
        );
        painter.hline(
            rect.shrink2(egui::vec2(2.0, 0.0)).x_range(),
            rect.min.y + 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 180)),
        );
    }

    let text_color = if hovered { ACCENT } else { TEXT_PRIMARY };
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, label, font, text_color);

    response.clicked()
}
