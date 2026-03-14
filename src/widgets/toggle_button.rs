use eframe::egui;
use super::*;

/// A modern toggle button with 3D-ish active/inactive states.
pub fn toggle_button(ui: &mut egui::Ui, label: &str, active: bool) -> bool {
    let height = 24.0;
    let font = egui::FontId::proportional(12.0);
    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), egui::Color32::BLACK);
    let width = galley.size().x + 22.0;

    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(width, height),
        egui::Sense::click(),
    );

    let painter = ui.painter();
    let hovered = response.hovered();
    let r = egui::CornerRadius::same(5);

    if active {
        painter.rect_filled(rect, r, ACCENT);
        painter.hline(rect.x_range(), rect.min.y + 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(0, 0, 0, 30)));
        painter.hline(rect.x_range(), rect.max.y - 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 40)));
        painter.text(rect.center(), egui::Align2::CENTER_CENTER, label, font, egui::Color32::WHITE);
    } else {
        let bg = if hovered { BG_HOVER } else { BG_RAISED };
        painter.rect_filled(rect, r, bg);
        let bc = if hovered { BORDER_HOVER } else { BORDER };
        painter.rect_stroke(rect, r, egui::Stroke::new(1.0, bc), egui::StrokeKind::Inside);
        painter.hline(rect.shrink2(egui::vec2(2.0, 0.0)).x_range(), rect.max.y,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(0, 0, 0, 12)));
        painter.hline(rect.shrink2(egui::vec2(2.0, 0.0)).x_range(), rect.min.y + 1.0,
            egui::Stroke::new(1.0, egui::Color32::from_rgba_unmultiplied(255, 255, 255, 180)));
        let tc = if hovered { ACCENT } else { TEXT_PRIMARY };
        painter.text(rect.center(), egui::Align2::CENTER_CENTER, label, font, tc);
    }

    response.clicked()
}
