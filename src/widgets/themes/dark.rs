use crate::widgets::Palette;
use eframe::egui;

impl Palette {
    /// Neutral instrument dark theme — near-black cool chrome with a single warm
    /// amber accent reserved for interactive state. Amber bridges toward the
    /// red-light Night theme; saturated color otherwise belongs to the image's
    /// colormap and to status (amber/green), not to the UI frame.
    pub fn dark() -> Self {
        let window = egui::Color32::from_rgb(13, 15, 18);   // near-black, faintly cool
        let panel = egui::Color32::from_rgb(20, 23, 28);
        let raised = egui::Color32::from_rgb(27, 31, 38);
        let hover = egui::Color32::from_rgb(35, 40, 51);

        let accent = egui::Color32::from_rgb(224, 158, 58);       // warm amber
        let accent_dark = egui::Color32::from_rgb(190, 130, 40);
        let accent_light = egui::Color32::from_rgb(240, 190, 110);
        // Dark, warm "on-accent" ink — readable on the amber fill (Play button
        // text, checkbox marks, combo hover text).
        let on_accent = egui::Color32::from_rgb(28, 18, 5);

        let text_primary = egui::Color32::from_rgb(212, 217, 224);
        let text_secondary = egui::Color32::from_rgb(138, 147, 160);

        let border = egui::Color32::from_rgb(42, 48, 59);
        let border_hover = egui::Color32::from_rgb(58, 66, 80);

        Self {
            accent,
            accent_dark,
            accent_light,
            border,
            border_hover,
            text_primary,
            text_secondary,
            bg_raised: raised,
            bg_hover: hover,
            bg_surface: panel,

            extreme_bg: egui::Color32::from_rgb(10, 12, 15),
            faint_bg: egui::Color32::from_rgb(24, 28, 34),

            panel_fill: panel,
            window_fill: window,
            toolbar_fill: window,
            toolbar_border: border,
            statusbar_fill: window,
            statusbar_border: border,
            section_header_fill: raised,
            section_header_text: egui::Color32::from_rgb(154, 164, 178),
            section_border: border,
            section_body_fill: panel,

            tab_bar: window,
            tab_active_bg: panel,
            tab_hover_bg: raised,
            tab_active_text: text_primary,
            tab_inactive_text: egui::Color32::from_rgb(107, 114, 128),

            plot_line: accent,
            plot_bg: window,

            image_bg: egui::Color32::from_rgb(5, 6, 8),

            check_mark: on_accent,

            combo_text: text_primary,
            combo_bg: raised,
            combo_border: border,
            combo_border_open: accent,
            combo_hover_bg: hover,
            combo_popup_bg: panel,
            combo_popup_border: border,
            combo_item_selected_bg: egui::Color32::from_rgb(58, 42, 18),
            combo_item_hover_bg: accent,
            combo_item_hover_text: on_accent,
            combo_chevron: text_secondary,

            slider_track: egui::Color32::from_rgb(42, 48, 59),
            slider_track_top: egui::Color32::from_rgb(35, 40, 51),
            slider_handle: egui::Color32::from_rgb(196, 203, 212),
            slider_handle_hover: egui::Color32::WHITE,
            slider_handle_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 80),
            slider_handle_highlight: egui::Color32::from_rgba_unmultiplied(255, 255, 255, 30),

            button_bg: raised,
            button_bg_hover: hover,
            button_bg_pressed: egui::Color32::from_rgb(42, 48, 59),
            button_shadow: egui::Color32::from_rgba_unmultiplied(0, 0, 0, 60),
            button_highlight: egui::Color32::from_rgba_unmultiplied(255, 255, 255, 12),

            axes_text: text_secondary,
            axes_stroke: egui::Color32::from_rgb(58, 66, 80),
        }
    }
}
