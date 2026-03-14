mod button;
mod checkbox;
mod combo_box;
mod slider;

pub use button::styled_button;
pub use checkbox::styled_checkbox;
pub use combo_box::combo_box;
pub use slider::{styled_slider, styled_slider_bare, styled_slider_log, styled_slider_log_bare, styled_slider_u32};

use eframe::egui;

// Contemporary palette — indigo accent, warm neutrals
pub const ACCENT: egui::Color32 = egui::Color32::from_rgb(79, 70, 229);        // indigo-600
pub const ACCENT_DARK: egui::Color32 = egui::Color32::from_rgb(67, 56, 202);   // indigo-700
pub const ACCENT_LIGHT: egui::Color32 = egui::Color32::from_rgb(99, 102, 241); // indigo-500
pub const BORDER: egui::Color32 = egui::Color32::from_rgb(209, 213, 219);       // gray-300 warm
pub const BORDER_HOVER: egui::Color32 = egui::Color32::from_rgb(156, 163, 175); // gray-400
pub const TEXT_PRIMARY: egui::Color32 = egui::Color32::from_rgb(17, 24, 39);     // gray-900
pub const TEXT_SECONDARY: egui::Color32 = egui::Color32::from_rgb(107, 114, 128); // gray-500
pub const BG_RAISED: egui::Color32 = egui::Color32::from_rgb(255, 255, 255);
pub const BG_HOVER: egui::Color32 = egui::Color32::from_rgb(243, 244, 246);     // gray-100
pub const BG_SURFACE: egui::Color32 = egui::Color32::from_rgb(249, 250, 251);   // gray-50
