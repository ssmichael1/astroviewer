mod button;
mod checkbox;
mod combo_box;
mod slider;

pub use button::styled_button;
pub use checkbox::styled_checkbox;
pub use combo_box::combo_box;
#[allow(unused_imports)]
pub use slider::{styled_slider, styled_slider_bare, styled_slider_log, styled_slider_log_bare, styled_slider_log_f, styled_slider_u32};

use eframe::egui;

// ── UI Theme / Palette System ──────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UiTheme {
    Light,
    Night,
}

impl UiTheme {
    pub const ALL: &'static [(UiTheme, &'static str)] = &[
        (UiTheme::Light, "Light"),
        (UiTheme::Night, "Night"),
    ];

    pub fn palette(self) -> Palette {
        match self {
            UiTheme::Light => Palette::light(),
            UiTheme::Night => Palette::night(),
        }
    }
}

/// Complete color palette for the UI.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub struct Palette {
    // Accent colors
    pub accent: egui::Color32,
    pub accent_dark: egui::Color32,
    pub accent_light: egui::Color32,

    // Borders
    pub border: egui::Color32,
    pub border_hover: egui::Color32,

    // Text
    pub text_primary: egui::Color32,
    pub text_secondary: egui::Color32,

    // Backgrounds
    pub bg_raised: egui::Color32,
    pub bg_hover: egui::Color32,
    pub bg_surface: egui::Color32,

    // Panel-specific
    pub panel_fill: egui::Color32,
    pub window_fill: egui::Color32,
    pub toolbar_fill: egui::Color32,
    pub toolbar_border: egui::Color32,
    pub statusbar_fill: egui::Color32,
    pub statusbar_border: egui::Color32,
    pub section_header_fill: egui::Color32,
    pub section_header_text: egui::Color32,
    pub section_border: egui::Color32,
    pub section_body_fill: egui::Color32,

    // Bottom panel tabs
    pub tab_bar: egui::Color32,
    pub tab_active_bg: egui::Color32,
    pub tab_hover_bg: egui::Color32,
    pub tab_active_text: egui::Color32,
    pub tab_inactive_text: egui::Color32,

    // Histogram plot
    pub plot_line: egui::Color32,
    pub plot_bg: egui::Color32,

    // Central panel background (behind image)
    pub image_bg: egui::Color32,

    // Checkbox check color
    pub check_mark: egui::Color32,

    // Combo box specific
    pub combo_text: egui::Color32,
    pub combo_bg: egui::Color32,
    pub combo_border: egui::Color32,
    pub combo_border_open: egui::Color32,
    pub combo_hover_bg: egui::Color32,
    pub combo_popup_bg: egui::Color32,
    pub combo_popup_border: egui::Color32,
    pub combo_item_selected_bg: egui::Color32,
    pub combo_item_hover_bg: egui::Color32,
    pub combo_item_hover_text: egui::Color32,
    pub combo_chevron: egui::Color32,

    // Slider specific
    pub slider_track: egui::Color32,
    pub slider_track_top: egui::Color32,
    pub slider_handle: egui::Color32,
    pub slider_handle_hover: egui::Color32,
    pub slider_handle_shadow: egui::Color32,
    pub slider_handle_highlight: egui::Color32,

    // Button specific
    pub button_bg: egui::Color32,
    pub button_bg_hover: egui::Color32,
    pub button_bg_pressed: egui::Color32,
    pub button_shadow: egui::Color32,
    pub button_highlight: egui::Color32,

    // Axes/colorbar text
    pub axes_text: egui::Color32,
    pub axes_stroke: egui::Color32,
}

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

            tab_bar: egui::Color32::from_rgb(37, 37, 38),
            tab_active_bg: egui::Color32::from_rgb(30, 30, 30),
            tab_hover_bg: egui::Color32::from_rgb(45, 45, 46),
            tab_active_text: egui::Color32::WHITE,
            tab_inactive_text: egui::Color32::from_rgb(150, 150, 150),

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

// Legacy constants — kept for non-palette code paths
#[allow(dead_code)]
pub const ACCENT: egui::Color32 = egui::Color32::from_rgb(79, 70, 229);
#[allow(dead_code)]
pub const TEXT_SECONDARY: egui::Color32 = egui::Color32::from_rgb(107, 114, 128);
#[allow(dead_code)]
pub const BG_SURFACE: egui::Color32 = egui::Color32::from_rgb(249, 250, 251);
