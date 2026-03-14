use eframe::egui;

/// A modern styled combo box / dropdown.
pub fn combo_box<T: PartialEq + Copy>(
    ui: &mut egui::Ui,
    id: &str,
    label: &str,
    current: &mut T,
    options: &[(T, &str)],
) -> bool {
    let mut changed = false;

    let current_label = options
        .iter()
        .find(|(v, _)| *v == *current)
        .map(|(_, l)| *l)
        .unwrap_or("—");

    if !label.is_empty() {
        ui.horizontal(|ui| {
            ui.label(label);
            changed = combo_box_inner(ui, id, current, current_label, options);
        });
    } else {
        changed = combo_box_inner(ui, id, current, current_label, options);
    }

    changed
}

fn combo_box_inner<T: PartialEq + Copy>(
    ui: &mut egui::Ui,
    id: &str,
    current: &mut T,
    current_label: &str,
    options: &[(T, &str)],
) -> bool {
    let mut changed = false;
    let popup_id = ui.make_persistent_id(id);
    let is_open = ui.memory(|m| m.is_popup_open(popup_id));

    let desired_width = ui.available_width().min(160.0).max(80.0);
    let height = 28.0;

    let border_color = if is_open {
        egui::Color32::from_rgb(0, 122, 204)
    } else {
        egui::Color32::from_rgb(206, 206, 206)
    };

    let (rect, response) = ui.allocate_exact_size(
        egui::vec2(desired_width, height),
        egui::Sense::click(),
    );

    let bg = if response.hovered() || is_open {
        egui::Color32::from_rgb(248, 248, 248)
    } else {
        egui::Color32::WHITE
    };

    let painter = ui.painter();
    painter.rect(
        rect,
        egui::CornerRadius::same(4),
        bg,
        egui::Stroke::new(1.0, border_color),
        egui::StrokeKind::Inside,
    );

    let text_rect = rect.shrink2(egui::vec2(10.0, 0.0));
    painter.text(
        egui::pos2(text_rect.min.x, rect.center().y),
        egui::Align2::LEFT_CENTER,
        current_label,
        egui::FontId::proportional(13.0),
        egui::Color32::from_rgb(51, 51, 51),
    );

    // Chevron
    let arrow_x = rect.max.x - 14.0;
    let arrow_y = rect.center().y;
    let arrow_size = 4.0;
    let arrow_color = egui::Color32::from_rgb(120, 120, 120);
    painter.line_segment(
        [
            egui::pos2(arrow_x - arrow_size, arrow_y - arrow_size * 0.6),
            egui::pos2(arrow_x, arrow_y + arrow_size * 0.6),
        ],
        egui::Stroke::new(1.5, arrow_color),
    );
    painter.line_segment(
        [
            egui::pos2(arrow_x, arrow_y + arrow_size * 0.6),
            egui::pos2(arrow_x + arrow_size, arrow_y - arrow_size * 0.6),
        ],
        egui::Stroke::new(1.5, arrow_color),
    );

    if response.clicked() {
        ui.memory_mut(|m| m.toggle_popup(popup_id));
    }

    if is_open {
        let area_resp = egui::Area::new(popup_id)
            .order(egui::Order::Foreground)
            .fixed_pos(egui::pos2(rect.min.x, rect.max.y + 2.0))
            .show(ui.ctx(), |ui| {
                egui::Frame::new()
                    .fill(egui::Color32::WHITE)
                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(206, 206, 206)))
                    .corner_radius(egui::CornerRadius::same(4))
                    .shadow(egui::Shadow {
                        offset: [0, 2],
                        blur: 8,
                        spread: 0,
                        color: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 30),
                    })
                    .inner_margin(egui::Margin::symmetric(2, 2))
                    .show(ui, |ui| {
                        ui.set_width(desired_width - 4.0);
                        for &(value, label) in options {
                            let is_selected = value == *current;
                            let item_rect = ui.available_rect_before_wrap();
                            let item_rect = egui::Rect::from_min_size(
                                item_rect.min,
                                egui::vec2(desired_width - 4.0, 26.0),
                            );
                            let resp = ui.allocate_rect(item_rect, egui::Sense::click());

                            let item_bg = if resp.hovered() {
                                egui::Color32::from_rgb(0, 122, 204)
                            } else if is_selected {
                                egui::Color32::from_rgb(232, 240, 252)
                            } else {
                                egui::Color32::TRANSPARENT
                            };
                            let text_color = if resp.hovered() {
                                egui::Color32::WHITE
                            } else {
                                egui::Color32::from_rgb(51, 51, 51)
                            };

                            if item_bg != egui::Color32::TRANSPARENT {
                                ui.painter().rect_filled(item_rect, egui::CornerRadius::same(3), item_bg);
                            }
                            ui.painter().text(
                                egui::pos2(item_rect.min.x + 10.0, item_rect.center().y),
                                egui::Align2::LEFT_CENTER,
                                label,
                                egui::FontId::proportional(13.0),
                                text_color,
                            );

                            if resp.clicked() {
                                *current = value;
                                changed = true;
                                ui.memory_mut(|m| m.close_popup());
                            }
                        }
                    });
            });

        if ui.input(|i| i.pointer.any_click()) && !response.hovered() {
            let popup_rect = area_resp.response.rect;
            if let Some(pos) = ui.input(|i| i.pointer.interact_pos()) {
                if !popup_rect.contains(pos) && !rect.contains(pos) {
                    ui.memory_mut(|m| m.close_popup());
                }
            }
        }
    }

    changed
}
