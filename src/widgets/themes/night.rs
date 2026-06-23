use crate::widgets::Palette;
use eframe::egui;

impl Palette {
    pub fn night() -> Self {
        // Deep red/amber night-vision palette — nothing above ~620nm
        let bg_dark = egui::Color32::from_rgb(15, 5, 5);
        let bg_mid = egui::Color32::from_rgb(25, 8, 8);
        let bg_raised = egui::Color32::from_rgb(35, 12, 12);
        let bg_hover = egui::Color32::from_rgb(50, 18, 18);

        let red_accent = egui::Color32::from_rgb(160, 40, 40);
        let red_accent_dark = egui::Color32::from_rgb(130, 30, 30);
        let red_accent_light = egui::Color32::from_rgb(180, 55, 55);

        let text_dim = egui::Color32::from_rgb(140, 50, 50);
        let text_bright = egui::Color32::from_rgb(180, 70, 70);

        let border_dim = egui::Color32::from_rgb(70, 25, 25);
        let border_bright = egui::Color32::from_rgb(100, 35, 35);

        Self {
            accent: red_accent,
            accent_dark: red_accent_dark,
            accent_light: red_accent_light,
            border: border_dim,
            border_hover: border_bright,
            text_primary: text_bright,
            text_secondary: text_dim,
            bg_raised,
            bg_hover,
            bg_surface: bg_mid,

            extreme_bg: egui::Color32::from_rgb(20, 8, 8),
            faint_bg: egui::Color32::from_rgb(30, 12, 12),

            panel_fill: bg_mid,
            window_fill: bg_dark,
            toolbar_fill: bg_dark,
            toolbar_border: border_dim,
            statusbar_fill: bg_dark,
            statusbar_border: border_dim,
            section_header_fill: egui::Color32::from_rgb(40, 14, 14),
            section_header_text: egui::Color32::from_rgb(160, 60, 60),
            section_border: border_dim,
            section_body_fill: bg_mid,

            tab_bar: bg_dark,
            tab_active_bg: egui::Color32::from_rgb(30, 10, 10),
            tab_hover_bg: egui::Color32::from_rgb(45, 16, 16),
            tab_active_text: egui::Color32::from_rgb(180, 70, 70),
            tab_inactive_text: egui::Color32::from_rgb(100, 40, 40),

            plot_line: red_accent,
            plot_bg: bg_dark,

            image_bg: egui::Color32::from_rgb(5, 2, 2),

            check_mark: egui::Color32::from_rgb(200, 80, 80),

            combo_text: text_bright,
            combo_bg: bg_raised,
            combo_border: border_dim,
            combo_border_open: red_accent,
            combo_hover_bg: bg_hover,
            combo_popup_bg: bg_mid,
            combo_popup_border: border_dim,
            combo_item_selected_bg: egui::Color32::from_rgb(60, 20, 20),
            combo_item_hover_bg: red_accent,
            combo_item_hover_text: egui::Color32::from_rgb(220, 90, 90),
            combo_chevron: text_dim,

            slider_track: egui::Color32::from_rgb(50, 20, 20),
            slider_track_top: egui::Color32::from_rgb(40, 15, 15),
            slider_handle: bg_raised,
            slider_handle_hover: bg_hover,
            slider_handle_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 40),
            slider_handle_highlight: egui::Color32::from_rgba_unmultiplied(100, 30, 30, 60),

            button_bg: bg_raised,
            button_bg_hover: bg_hover,
            button_bg_pressed: egui::Color32::from_rgb(60, 22, 22),
            button_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 20),
            button_highlight: egui::Color32::from_rgba_unmultiplied(100, 30, 30, 40),

            axes_text: text_dim,
            axes_stroke: egui::Color32::from_rgb(80, 30, 30),
        }
    }
}
