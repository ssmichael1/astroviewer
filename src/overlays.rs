use egui::{Color32, Pos2, Stroke};

/// Overlay items drawn on top of the image.
#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum OverlayItem {
    /// Detected star centroid with covariance ellipse
    Centroid {
        /// Pixel x (image-center origin, +X right)
        x: f32,
        /// Pixel y (image-center origin, +Y down)
        y: f32,
        /// Brightness (sum of pixel values in blob)
        mass: f32,
        /// Semi-major axis (pixels), from covariance eigenvalue
        semi_major: f32,
        /// Semi-minor axis (pixels), from covariance eigenvalue
        semi_minor: f32,
        /// Rotation angle of major axis (radians, from +X axis)
        angle: f32,
    },
    /// Catalog star projected onto image (from plate solve)
    CatalogStar {
        x: f32,
        y: f32,
        name: Option<String>,
        mag: f32,
    },
    /// Generic marker
    Marker {
        x: f32,
        y: f32,
        kind: MarkerKind,
        label: Option<String>,
    },
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub enum MarkerKind {
    Crosshair,
    Circle(f32), // radius in pixels
    Diamond(f32),
}

/// Convert tetra3rs centroid to overlay item.
#[cfg(feature = "starsolve")]
pub fn centroid_to_overlay(c: &tetra3::Centroid) -> OverlayItem {
    let (semi_major, semi_minor, angle) = if let Some(cov) = c.cov {
        cov_to_ellipse(cov)
    } else {
        (2.0, 2.0, 0.0)
    };

    OverlayItem::Centroid {
        x: c.x,
        y: c.y,
        mass: c.mass.unwrap_or(0.0),
        semi_major,
        semi_minor,
        angle,
    }
}

/// Extract ellipse parameters from a 2x2 covariance matrix.
/// Returns (semi_major, semi_minor, angle_radians).
#[cfg(feature = "starsolve")]
fn cov_to_ellipse(cov: tetra3::Matrix2) -> (f32, f32, f32) {
    // Eigenvalues of 2x2 symmetric matrix [[a, b], [b, c]]:
    // λ = ((a+c) ± sqrt((a-c)² + 4b²)) / 2
    let a = cov[(0, 0)];
    let b = cov[(0, 1)];
    let c = cov[(1, 1)];

    let trace = a + c;
    let det = a * c - b * b;
    let disc = ((trace * trace - 4.0 * det).max(0.0)).sqrt();

    let lambda1 = (trace + disc) / 2.0;
    let lambda2 = (trace - disc) / 2.0;

    // Semi-axes are sqrt of eigenvalues (std dev), scaled for visibility
    let scale = 3.0; // ~3-sigma ellipse
    let semi_major = lambda1.max(0.0).sqrt() * scale;
    let semi_minor = lambda2.max(0.0).sqrt() * scale;

    // Angle of major axis eigenvector
    let angle = if b.abs() > 1e-10 {
        (lambda1 - a).atan2(b)
    } else if a >= c {
        0.0
    } else {
        std::f32::consts::FRAC_PI_2
    };

    (semi_major.max(1.5), semi_minor.max(1.5), angle)
}

/// Draw overlay items onto the image area.
/// `to_screen` converts pixel coords (image-center origin) to screen coords.
pub fn draw_overlays(
    painter: &egui::Painter,
    items: &[OverlayItem],
    to_screen: impl Fn(f32, f32) -> Pos2,
    pixels_to_screen_scale: f32,
    max_mass: f32,
    stroke_scale: f32,
) {
    for item in items {
        match item {
            OverlayItem::Centroid { x, y, mass, semi_major, semi_minor, angle } => {
                let center = to_screen(*x, *y);

                // Color based on brightness: dim=cyan → bright=yellow
                let t = if max_mass > 0.0 {
                    (mass / max_mass).sqrt().clamp(0.0, 1.0)
                } else {
                    0.5
                };
                let color = brightness_color(t);

                // Scale ellipse axes from image pixels to screen pixels
                let smaj = *semi_major * pixels_to_screen_scale;
                let smin = *semi_minor * pixels_to_screen_scale;

                draw_ellipse(painter, center, smaj.max(2.0), smin.max(2.0), *angle, color, stroke_scale);
            }
            OverlayItem::CatalogStar { x, y, name, mag } => {
                let center = to_screen(*x, *y);
                let radius = (6.0 - mag).clamp(2.0, 8.0);
                painter.circle_stroke(center, radius, Stroke::new(1.5, Color32::from_rgb(50, 205, 50)));
                if let Some(name) = name {
                    painter.text(
                        Pos2::new(center.x + radius + 3.0, center.y),
                        egui::Align2::LEFT_CENTER,
                        name,
                        egui::FontId::proportional(10.0),
                        Color32::from_rgb(50, 205, 50),
                    );
                }
            }
            OverlayItem::Marker { x, y, kind, label } => {
                let center = to_screen(*x, *y);
                match kind {
                    MarkerKind::Crosshair => {
                        let s = 6.0 * stroke_scale;
                        let stroke = Stroke::new(1.0 * stroke_scale, Color32::from_rgb(50, 255, 50));
                        painter.line_segment([Pos2::new(center.x - s, center.y), Pos2::new(center.x + s, center.y)], stroke);
                        painter.line_segment([Pos2::new(center.x, center.y - s), Pos2::new(center.x, center.y + s)], stroke);
                    }
                    MarkerKind::Circle(r) => {
                        painter.circle_stroke(center, *r, Stroke::new(1.0, Color32::from_rgb(255, 255, 0)));
                    }
                    MarkerKind::Diamond(s) => {
                        let s = *s;
                        let stroke = Stroke::new(1.0, Color32::from_rgb(255, 255, 0));
                        let pts = [
                            Pos2::new(center.x, center.y - s),
                            Pos2::new(center.x + s, center.y),
                            Pos2::new(center.x, center.y + s),
                            Pos2::new(center.x - s, center.y),
                            Pos2::new(center.x, center.y - s),
                        ];
                        for w in pts.windows(2) {
                            painter.line_segment([w[0], w[1]], stroke);
                        }
                    }
                }
                if let Some(label) = label {
                    painter.text(
                        Pos2::new(center.x + 10.0, center.y),
                        egui::Align2::LEFT_CENTER,
                        label,
                        egui::FontId::proportional(10.0),
                        Color32::from_rgb(255, 255, 0),
                    );
                }
            }
        }
    }
}

/// Map brightness fraction [0,1] to a color: dim cyan → bright yellow
fn brightness_color(t: f32) -> Color32 {
    let r = (t * 255.0) as u8;
    let g = (200.0 + t * 55.0) as u8;
    let b = ((1.0 - t) * 255.0) as u8;
    Color32::from_rgb(r, g, b)
}

/// Draw an axis-aligned ellipse rotated by `angle` radians.
fn draw_ellipse(painter: &egui::Painter, center: Pos2, semi_major: f32, semi_minor: f32, angle: f32, color: Color32, stroke_scale: f32) {
    let n_segments = 24;
    let cos_a = angle.cos();
    let sin_a = angle.sin();

    let mut points = Vec::with_capacity(n_segments + 1);
    for i in 0..=n_segments {
        let theta = 2.0 * std::f32::consts::PI * i as f32 / n_segments as f32;
        let ex = semi_major * theta.cos();
        let ey = semi_minor * theta.sin();
        // Rotate by angle
        let rx = ex * cos_a - ey * sin_a;
        let ry = ex * sin_a + ey * cos_a;
        points.push(Pos2::new(center.x + rx, center.y + ry));
    }

    for w in points.windows(2) {
        painter.line_segment([w[0], w[1]], Stroke::new(1.5 * stroke_scale, color));
    }
}
