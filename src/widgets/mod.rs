mod button;
mod checkbox;
mod combo_box;
mod slider;
mod themes;

pub use button::{primary_button, styled_button};
pub use checkbox::styled_checkbox;
pub use combo_box::combo_box;
#[allow(unused_imports)]
pub use slider::{styled_slider, styled_slider_bare, styled_slider_log, styled_slider_log_bare, styled_slider_log_f, styled_slider_u32};

use eframe::egui;

// ── UI Theme / Palette System ──────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UiTheme {
    Dark,
    Light,
    Night,
}

impl UiTheme {
    pub const ALL: &'static [(UiTheme, &'static str)] = &[
        (UiTheme::Dark, "Dark"),
        (UiTheme::Light, "Light"),
        (UiTheme::Night, "Night"),
    ];

    pub fn palette(self) -> Palette {
        match self {
            UiTheme::Dark => Palette::dark(),
            UiTheme::Light => Palette::light(),
            UiTheme::Night => Palette::night(),
        }
    }

    /// Whether this theme uses a dark surround (everything except Light).
    pub fn is_dark(self) -> bool {
        !matches!(self, UiTheme::Light)
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

    // Text-input / DragValue backgrounds (egui extreme/faint bg)
    pub extreme_bg: egui::Color32,
    pub faint_bg: egui::Color32,

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

// Palette constructors live in `themes/{light,night,dark}.rs`.

// Legacy constants — kept for non-palette code paths
#[allow(dead_code)]
pub const ACCENT: egui::Color32 = egui::Color32::from_rgb(79, 70, 229);
#[allow(dead_code)]
pub const TEXT_SECONDARY: egui::Color32 = egui::Color32::from_rgb(107, 114, 128);
#[allow(dead_code)]
pub const BG_SURFACE: egui::Color32 = egui::Color32::from_rgb(249, 250, 251);
