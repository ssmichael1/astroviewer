/// A colormap maps a normalized value in [0, 1] to an RGB color.
///
/// Each colormap is stored as a 256-entry lookup table (LUT).
/// To apply: `let rgb = cmap.lookup(normalized_value)`

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColormapKind {
    Grayscale,
    Hot,
    Viridis,
    Inferno,
    Magma,
    Red,
}

impl ColormapKind {
    pub const ALL: &[ColormapKind] = &[
        ColormapKind::Grayscale,
        ColormapKind::Hot,
        ColormapKind::Viridis,
        ColormapKind::Inferno,
        ColormapKind::Magma,
        ColormapKind::Red,
    ];

    pub fn name(&self) -> &'static str {
        match self {
            ColormapKind::Grayscale => "Grayscale",
            ColormapKind::Hot => "Hot",
            ColormapKind::Viridis => "Viridis",
            ColormapKind::Inferno => "Inferno",
            ColormapKind::Magma => "Magma",
            ColormapKind::Red => "Red",
        }
    }

    pub fn build_lut(&self) -> [[u8; 3]; 256] {
        match self {
            ColormapKind::Grayscale => build_grayscale(),
            ColormapKind::Hot => build_hot(),
            ColormapKind::Viridis => build_from_anchors(&VIRIDIS_ANCHORS),
            ColormapKind::Inferno => build_from_anchors(&INFERNO_ANCHORS),
            ColormapKind::Magma => build_from_anchors(&MAGMA_ANCHORS),
            ColormapKind::Red => build_red(),
        }
    }
}

pub struct Colormap {
    pub kind: ColormapKind,
    pub lut: [[u8; 3]; 256],
}

impl Colormap {
    pub fn new(kind: ColormapKind) -> Self {
        Self {
            lut: kind.build_lut(),
            kind,
        }
    }

    /// Look up an RGB color for a normalized value in [0, 1].
    #[inline]
    pub fn lookup(&self, t: f32) -> [u8; 3] {
        let idx = (t.clamp(0.0, 1.0) * 255.0) as usize;
        self.lut[idx]
    }
}

fn build_grayscale() -> [[u8; 3]; 256] {
    let mut lut = [[0u8; 3]; 256];
    for (i, entry) in lut.iter_mut().enumerate() {
        let v = i as u8;
        *entry = [v, v, v];
    }
    lut
}

fn build_hot() -> [[u8; 3]; 256] {
    // Black → Red → Yellow → White
    let mut lut = [[0u8; 3]; 256];
    for (i, entry) in lut.iter_mut().enumerate() {
        let t = i as f32 / 255.0;
        let r = (t * 3.0).min(1.0);
        let g = ((t - 1.0 / 3.0) * 3.0).clamp(0.0, 1.0);
        let b = ((t - 2.0 / 3.0) * 3.0).clamp(0.0, 1.0);
        *entry = [(r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8];
    }
    lut
}

fn build_red() -> [[u8; 3]; 256] {
    let mut lut = [[0u8; 3]; 256];
    for (i, entry) in lut.iter_mut().enumerate() {
        *entry = [i as u8, 0, 0];
    }
    lut
}

/// Linearly interpolate a colormap from anchor points.
/// Each anchor is (position_0_to_1, r, g, b) with values in [0, 1].
fn build_from_anchors(anchors: &[[f32; 4]]) -> [[u8; 3]; 256] {
    let mut lut = [[0u8; 3]; 256];
    for (i, entry) in lut.iter_mut().enumerate() {
        let t = i as f32 / 255.0;
        // Find the two anchors surrounding t
        let mut lo = 0;
        for (j, anchor) in anchors.iter().enumerate() {
            if anchor[0] <= t {
                lo = j;
            }
        }
        let hi = (lo + 1).min(anchors.len() - 1);
        let a = &anchors[lo];
        let b = &anchors[hi];
        let span = b[0] - a[0];
        let frac = if span > 0.0 { (t - a[0]) / span } else { 0.0 };
        let r = a[1] + frac * (b[1] - a[1]);
        let g = a[2] + frac * (b[2] - a[2]);
        let bl = a[3] + frac * (b[3] - a[3]);
        *entry = [
            (r * 255.0) as u8,
            (g * 255.0) as u8,
            (bl * 255.0) as u8,
        ];
    }
    lut
}

// Viridis anchor points (sampled from matplotlib)
const VIRIDIS_ANCHORS: [[f32; 4]; 9] = [
    [0.000, 0.267, 0.004, 0.329],
    [0.125, 0.283, 0.141, 0.458],
    [0.250, 0.254, 0.265, 0.530],
    [0.375, 0.206, 0.372, 0.553],
    [0.500, 0.163, 0.471, 0.558],
    [0.625, 0.128, 0.567, 0.551],
    [0.750, 0.267, 0.678, 0.480],
    [0.875, 0.578, 0.773, 0.322],
    [1.000, 0.993, 0.906, 0.144],
];

// Inferno anchor points
const INFERNO_ANCHORS: [[f32; 4]; 9] = [
    [0.000, 0.001, 0.000, 0.014],
    [0.125, 0.090, 0.027, 0.282],
    [0.250, 0.258, 0.039, 0.406],
    [0.375, 0.416, 0.055, 0.365],
    [0.500, 0.578, 0.148, 0.240],
    [0.625, 0.735, 0.271, 0.108],
    [0.750, 0.865, 0.435, 0.010],
    [0.875, 0.955, 0.640, 0.040],
    [1.000, 0.988, 1.000, 0.644],
];

// Magma anchor points
const MAGMA_ANCHORS: [[f32; 4]; 9] = [
    [0.000, 0.001, 0.000, 0.014],
    [0.125, 0.095, 0.030, 0.250],
    [0.250, 0.240, 0.040, 0.420],
    [0.375, 0.400, 0.060, 0.480],
    [0.500, 0.550, 0.110, 0.450],
    [0.625, 0.720, 0.200, 0.380],
    [0.750, 0.870, 0.360, 0.400],
    [0.875, 0.950, 0.570, 0.550],
    [1.000, 0.987, 0.991, 0.750],
];
