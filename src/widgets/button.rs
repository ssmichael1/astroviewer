use eframe::egui;
use super::Palette;

/// A contemporary styled push button.
pub fn styled_button(ui: &mut egui::Ui, label: &str, pal: &Palette) -> bool {
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
        pal.button_bg_pressed
    } else if hovered {
        pal.button_bg_hover
    } else {
        pal.button_bg
    };

    let border_c = if hovered { pal.border_hover } else { pal.border };

    painter.rect_filled(rect, egui::CornerRadius::same(6), bg);
    painter.rect_stroke(rect, egui::CornerRadius::same(6), egui::Stroke::new(1.0, border_c), egui::StrokeKind::Inside);

    if !pressed {
        painter.hline(
            rect.shrink2(egui::vec2(2.0, 0.0)).x_range(),
            rect.max.y,
            egui::Stroke::new(1.0, pal.button_shadow),
        );
        painter.hline(
            rect.shrink2(egui::vec2(2.0, 0.0)).x_range(),
            rect.min.y + 1.0,
            egui::Stroke::new(1.0, pal.button_highlight),
        );
    }

    let text_color = if hovered { pal.accent } else { pal.text_primary };
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, label, font, text_color);

    response.clicked()
}

/// A filled, accent-colored button for the single primary action in a context
/// (e.g. Play). It is the one bright control so the eye lands on it first.
pub fn primary_button(ui: &mut egui::Ui, label: &str, pal: &Palette) -> bool {
    let height = 26.0;
    // Semibold "strong" family (installed in main) so the primary action reads
    // with weight, not just color.
    let font = egui::FontId::new(13.0, egui::FontFamily::Name("strong".into()));
    let galley = ui.painter().layout_no_wrap(label.to_string(), font.clone(), egui::Color32::BLACK);
    let width = galley.size().x + 28.0;

    let (rect, response) = ui.allocate_exact_size(egui::vec2(width, height), egui::Sense::click());

    let painter = ui.painter();
    let bg = if response.is_pointer_button_down_on() {
        pal.accent_dark
    } else if response.hovered() {
        pal.accent_light
    } else {
        pal.accent
    };

    painter.rect_filled(rect, egui::CornerRadius::same(6), bg);
    // Text sits on the accent fill — use the on-accent color (check_mark doubles
    // as "contrasting mark on accent" across all palettes).
    painter.text(rect.center(), egui::Align2::CENTER_CENTER, label, font, pal.check_mark);

    if response.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    response.clicked()
}
