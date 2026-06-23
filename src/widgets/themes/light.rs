use crate::widgets::Palette;
use eframe::egui;

impl Palette {
    pub fn light() -> Self {
        Self {
            accent: egui::Color32::from_rgb(79, 70, 229),        // indigo-600
            accent_dark: egui::Color32::from_rgb(67, 56, 202),   // indigo-700
            accent_light: egui::Color32::from_rgb(99, 102, 241), // indigo-500
            border: egui::Color32::from_rgb(209, 213, 219),       // gray-300
            border_hover: egui::Color32::from_rgb(156, 163, 175), // gray-400
            text_primary: egui::Color32::from_rgb(17, 24, 39),     // gray-900
            text_secondary: egui::Color32::from_rgb(107, 114, 128), // gray-500
            bg_raised: egui::Color32::from_rgb(255, 255, 255),
            bg_hover: egui::Color32::from_rgb(243, 244, 246),     // gray-100
            bg_surface: egui::Color32::from_rgb(249, 250, 251),   // gray-50

            extreme_bg: egui::Color32::from_rgb(248, 248, 248),
            faint_bg: egui::Color32::from_rgb(245, 245, 245),

            panel_fill: egui::Color32::from_rgb(249, 250, 251),
            window_fill: egui::Color32::WHITE,
            toolbar_fill: egui::Color32::from_rgb(243, 244, 246),
            toolbar_border: egui::Color32::from_rgb(229, 231, 235),
            statusbar_fill: egui::Color32::from_rgb(237, 238, 242),
            statusbar_border: egui::Color32::from_rgb(218, 220, 224),
            section_header_fill: egui::Color32::from_rgb(232, 234, 246),
            section_header_text: egui::Color32::from_rgb(75, 70, 110),
            section_border: egui::Color32::from_rgb(229, 231, 235),
            section_body_fill: egui::Color32::WHITE,

            // Tab bar lives in the light family — no charcoal island
            tab_bar: egui::Color32::from_rgb(237, 238, 242),
            tab_active_bg: egui::Color32::WHITE,
            tab_hover_bg: egui::Color32::from_rgb(245, 246, 249),
            tab_active_text: egui::Color32::from_rgb(17, 24, 39),    // gray-900
            tab_inactive_text: egui::Color32::from_rgb(107, 114, 128), // gray-500

            plot_line: egui::Color32::from_rgb(79, 70, 229),
            plot_bg: egui::Color32::from_rgb(249, 250, 251),

            image_bg: egui::Color32::from_rgb(240, 240, 240),

            check_mark: egui::Color32::WHITE,

            combo_text: egui::Color32::from_rgb(51, 51, 51),
            combo_bg: egui::Color32::WHITE,
            combo_border: egui::Color32::from_rgb(206, 206, 206),
            combo_border_open: egui::Color32::from_rgb(0, 122, 204),
            combo_hover_bg: egui::Color32::from_rgb(248, 248, 248),
            combo_popup_bg: egui::Color32::WHITE,
            combo_popup_border: egui::Color32::from_rgb(206, 206, 206),
            combo_item_selected_bg: egui::Color32::from_rgb(232, 240, 252),
            combo_item_hover_bg: egui::Color32::from_rgb(0, 122, 204),
            combo_item_hover_text: egui::Color32::WHITE,
            combo_chevron: egui::Color32::from_rgb(120, 120, 120),

            slider_track: egui::Color32::from_rgb(215, 216, 222),
            slider_track_top: egui::Color32::from_rgb(192, 193, 200),
            slider_handle: egui::Color32::from_rgb(255, 255, 255),
            slider_handle_hover: egui::Color32::from_rgb(252, 252, 254),
            slider_handle_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 22),
            slider_handle_highlight: egui::Color32::from_rgba_unmultiplied(255, 255, 255, 120),

            button_bg: egui::Color32::from_rgb(255, 255, 255),
            button_bg_hover: egui::Color32::from_rgb(243, 244, 246),
            button_bg_pressed: egui::Color32::from_rgb(232, 234, 240),
            button_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 12),
            button_highlight: egui::Color32::from_rgba_unmultiplied(255, 255, 255, 180),

            axes_text: egui::Color32::from_rgb(51, 51, 51),
            axes_stroke: egui::Color32::from_rgb(97, 97, 97),
        }
    }
}
