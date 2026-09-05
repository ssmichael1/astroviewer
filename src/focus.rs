//! Focus assist: a per-frame focus figure of merit measured off the UI thread.
//!
//! Two numbers come out of every frame:
//!
//! * **HFR** — the half flux radius of the brightest well-behaved stars,
//!   flux-weighted mean radius from the sub-pixel centroid, in the convention
//!   NINA and PHD2 use (a Gaussian of width σ reads `σ·√(π/2) ≈ 1.25σ`).
//!   Measured from the raw pixels in a small window around each centroid
//!   after subtracting the local background, so it does not move when the
//!   detection threshold moves. Smaller is better.
//! * **Sharpness** — a whole-frame contrast figure for the far-out-of-focus
//!   regime where stars are donuts the extractor no longer detects. Horizontal
//!   gradient energy divided by the frame variance: uncorrelated noise reads
//!   2 (less on real sensors, where noise is correlated), sharp stars push it
//!   up, defocus pulls it down. Larger is better, and only relative to its own
//!   recent values. Coarse only; once stars are detected HFR is the number
//!   to drive.
//!
//! This module knows nothing about the star extractor: it takes plain pixel
//! slices and positions, so it can sit under any detector later.

use std::collections::VecDeque;

/// A detected star handed in for measurement, image-center-origin pixel
/// coordinates as the extractor reports them.
#[derive(Clone, Copy, Debug)]
pub struct Star {
    pub x: f32,
    pub y: f32,
    /// Brightness used to pick the brightest `max_stars`; larger is brighter.
    pub mass: f32,
    /// Gaussian-equivalent width along the major axis, pixels, when the
    /// extractor provides second moments. Sizes the measurement window so a
    /// defocused star is measured whole rather than truncated.
    pub sigma: Option<f32>,
    /// Semi-major over semi-minor axis of the detection's second moments,
    /// when the extractor provides them. Elongated blobs (pairs, trails,
    /// coma) are skipped.
    pub elongation: Option<f32>,
}

#[derive(Clone, Debug)]
pub struct FocusConfig {
    /// Measure at most this many of the brightest candidates.
    pub max_stars: usize,
    /// Half-width of the square measurement window, pixels: `4σ` of the
    /// star's own width, clamped to `[window_radius, max_window_radius]`,
    /// so the border ring used for background sits clear of the star. A
    /// window of `2r+1` on a side must fit inside the frame for a star to
    /// count.
    pub window_radius: usize,
    pub max_window_radius: usize,
    /// Raw pixel value at or above which a star is treated as saturated and
    /// skipped: a clipped core has a flat top and no meaningful width.
    pub saturation: Option<f32>,
    /// Skip candidates whose elongation exceeds this ratio.
    pub max_elongation: f32,
    /// Only measure stars inside this inclusive `[x0, y0, x1, y1]` region,
    /// top-left-origin pixel coordinates. `None` uses the whole frame.
    pub roi: Option<[u32; 4]>,
}

impl Default for FocusConfig {
    fn default() -> Self {
        FocusConfig {
            max_stars: 30,
            window_radius: 7,
            max_window_radius: 20,
            saturation: None,
            max_elongation: 1.5,
            roi: None,
        }
    }
}

/// One star's measurement, image-center-origin coordinates.
#[derive(Clone, Copy, Debug)]
pub struct StarHfr {
    pub x: f32,
    pub y: f32,
    pub hfr: f32,
}

/// The focus figures for one frame.
#[derive(Clone, Debug, Default)]
pub struct FocusSample {
    /// Median HFR over `stars`, pixels. `None` when no star qualified.
    pub hfr_px: Option<f32>,
    pub stars: Vec<StarHfr>,
    /// Number of candidates offered before quality cuts, for the readout.
    pub candidates: usize,
    pub sharpness: f32,
}

/// Measure the frame. `pixels` is row-major, `width × height`; stars are in
/// image-center-origin coordinates with the origin at `((W-1)/2, (H-1)/2)`.
pub fn measure(pixels: &[f32], width: u32, height: u32, stars: &[Star], cfg: &FocusConfig) -> FocusSample {
    let sharpness = sharpness(pixels, width, height);
    let (w, h) = (width as usize, height as usize);
    if pixels.len() < w * h || w == 0 || h == 0 {
        return FocusSample { sharpness, ..Default::default() };
    }

    // Brightest first. Candidates are the extractor's detections; the
    // quality cuts below decide which of them are measured.
    let mut order: Vec<&Star> = stars.iter().collect();
    order.sort_by(|a, b| b.mass.partial_cmp(&a.mass).unwrap_or(std::cmp::Ordering::Equal));

    let cx0 = (w as f32 - 1.0) / 2.0;
    let cy0 = (h as f32 - 1.0) / 2.0;
    let mut out = Vec::with_capacity(cfg.max_stars);
    let mut border = Vec::with_capacity(8 * cfg.max_window_radius + 4);

    for s in order {
        if out.len() >= cfg.max_stars {
            break;
        }
        if s.elongation.is_some_and(|e| e > cfg.max_elongation) {
            continue;
        }
        // Sub-pixel position in top-left-origin pixel coordinates.
        let px = s.x + cx0;
        let py = s.y + cy0;
        if !px.is_finite() || !py.is_finite() {
            continue;
        }
        let (ic, ir) = (px.round() as i64, py.round() as i64);
        let r = s
            .sigma
            .filter(|sg| sg.is_finite())
            .map_or(cfg.window_radius, |sg| (4.0 * sg).ceil() as usize)
            .clamp(cfg.window_radius, cfg.max_window_radius.max(cfg.window_radius));
        if let Some([x0, y0, x1, y1]) = cfg.roi {
            if ic < x0 as i64 || ic > x1 as i64 || ir < y0 as i64 || ir > y1 as i64 {
                continue;
            }
        }
        // The window must lie fully inside the frame.
        if ic < r as i64 || ir < r as i64 || ic + r as i64 >= w as i64 || ir + r as i64 >= h as i64 {
            continue;
        }
        let (ic, ir) = (ic as usize, ir as usize);
        let (c0, r0) = (ic - r, ir - r);
        let side = 2 * r + 1;

        // Local background: median of the window's border ring.
        border.clear();
        let mut peak = f32::NEG_INFINITY;
        for j in 0..side {
            let row = &pixels[(r0 + j) * w + c0..(r0 + j) * w + c0 + side];
            for (i, &v) in row.iter().enumerate() {
                peak = peak.max(v);
                if j == 0 || j == side - 1 || i == 0 || i == side - 1 {
                    border.push(v);
                }
            }
        }
        if cfg.saturation.is_some_and(|sat| peak >= sat) {
            continue;
        }
        let bg = median(&mut border);

        // Flux-weighted mean radius from the sub-pixel centroid.
        let mut wsum = 0.0f64;
        let mut wr = 0.0f64;
        for j in 0..side {
            let y = (r0 + j) as f32 - py;
            let row = &pixels[(r0 + j) * w + c0..(r0 + j) * w + c0 + side];
            for (i, &v) in row.iter().enumerate() {
                let f = (v - bg).max(0.0) as f64;
                if f > 0.0 {
                    let x = (c0 + i) as f32 - px;
                    wsum += f;
                    wr += f * ((x * x + y * y) as f64).sqrt();
                }
            }
        }
        if wsum <= 0.0 {
            continue;
        }
        out.push(StarHfr { x: s.x, y: s.y, hfr: (wr / wsum) as f32 });
    }

    let mut hfrs: Vec<f32> = out.iter().map(|s| s.hfr).collect();
    let hfr_px = (!hfrs.is_empty()).then(|| median(&mut hfrs));
    FocusSample { hfr_px, stars: out, candidates: stars.len(), sharpness }
}

/// Horizontal gradient energy over frame variance, sampled on up to ~1024
/// evenly spaced rows so the cost stays bounded at any sensor size. Pure
/// noise reads 2; sharp detail raises it, defocus lowers it. Returns 0 when
/// the frame is flat.
pub fn sharpness(pixels: &[f32], width: u32, height: u32) -> f32 {
    let (w, h) = (width as usize, height as usize);
    if w < 2 || h == 0 || pixels.len() < w * h {
        return 0.0;
    }
    let step = (h / 1024).max(1);
    let mut n = 0usize;
    let mut sum = 0.0f64;
    let mut sum2 = 0.0f64;
    let mut grad2 = 0.0f64;
    let mut row = 0;
    while row < h {
        let r = &pixels[row * w..row * w + w];
        let mut prev = r[0];
        sum += prev as f64;
        sum2 += (prev as f64) * (prev as f64);
        for &v in &r[1..] {
            let d = (v - prev) as f64;
            grad2 += d * d;
            sum += v as f64;
            sum2 += (v as f64) * (v as f64);
            prev = v;
        }
        n += w;
        row += step;
    }
    let mean = sum / n as f64;
    let var = (sum2 / n as f64 - mean * mean).max(0.0);
    if var <= 0.0 {
        return 0.0;
    }
    let grads = n - (n / w); // one fewer difference than pixels per row
    (grad2 / grads as f64 / var) as f32
}

/// Median of a small buffer; sorts it in place.
fn median(v: &mut [f32]) -> f32 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = v.len();
    if n == 0 {
        0.0
    } else if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// One point on the focus trend.
#[derive(Clone, Copy, Debug)]
pub struct FocusPoint {
    /// Seconds since the history was created.
    pub t: f64,
    pub hfr_px: Option<f32>,
    pub sharpness: f32,
    /// Focuser step position at the time of the frame, when a focuser is attached.
    pub focuser_pos: Option<i32>,
}

/// Ring of recent samples plus the best HFR seen since the last reset. The
/// history is what a person focusing actually watches: the number moving as
/// the knob turns, and how far it is from the best it has been.
pub struct FocusHistory {
    samples: VecDeque<FocusPoint>,
    cap: usize,
    best: Option<FocusPoint>,
    start: std::time::Instant,
}

impl FocusHistory {
    pub fn new(cap: usize) -> Self {
        FocusHistory { samples: VecDeque::with_capacity(cap), cap: cap.max(1), best: None, start: std::time::Instant::now() }
    }

    pub fn push(&mut self, sample: &FocusSample, focuser_pos: Option<i32>) -> FocusPoint {
        let p = FocusPoint {
            t: self.start.elapsed().as_secs_f64(),
            hfr_px: sample.hfr_px,
            sharpness: sample.sharpness,
            focuser_pos,
        };
        if let Some(h) = p.hfr_px {
            if self.best.is_none_or(|b| b.hfr_px.is_none_or(|bh| h < bh)) {
                self.best = Some(p);
            }
        }
        if self.samples.len() >= self.cap {
            self.samples.pop_front();
        }
        self.samples.push_back(p);
        p
    }

    pub fn latest(&self) -> Option<&FocusPoint> {
        self.samples.back()
    }

    pub fn best(&self) -> Option<&FocusPoint> {
        self.best.as_ref()
    }

    pub fn iter(&self) -> impl Iterator<Item = &FocusPoint> {
        self.samples.iter()
    }

    /// Forget the trend and the best-so-far; the clock keeps running.
    pub fn reset(&mut self) {
        self.samples.clear();
        self.best = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Frame with Gaussian stars at top-left-origin positions.
    fn frame(w: usize, h: usize, bg: f32, stars: &[(f32, f32, f32, f32)]) -> Vec<f32> {
        let mut px = vec![bg; w * h];
        for &(cx, cy, sigma, amp) in stars {
            for r in 0..h {
                for c in 0..w {
                    let dx = c as f32 - cx;
                    let dy = r as f32 - cy;
                    px[r * w + c] += amp * (-(dx * dx + dy * dy) / (2.0 * sigma * sigma)).exp();
                }
            }
        }
        px
    }

    fn star(w: usize, h: usize, cx: f32, cy: f32, mass: f32) -> Star {
        Star { x: cx - (w as f32 - 1.0) / 2.0, y: cy - (h as f32 - 1.0) / 2.0, mass, sigma: None, elongation: Some(1.0) }
    }

    fn sized(mut s: Star, sigma: f32) -> Star {
        s.sigma = Some(sigma);
        s
    }

    #[test]
    fn gaussian_hfr_matches_closed_form() {
        // Flux-weighted mean radius of a 2-D Gaussian is σ·√(π/2).
        let (w, h) = (120, 100);
        let sigma = 2.0;
        let px = frame(w, h, 100.0, &[(50.3, 40.7, sigma, 1000.0)]);
        let s = measure(&px, w as u32, h as u32, &[star(w, h, 50.3, 40.7, 1.0)], &FocusConfig::default());
        let expect = sigma * (std::f32::consts::PI / 2.0).sqrt();
        let got = s.hfr_px.expect("one star measured");
        assert!((got - expect).abs() < 0.05, "hfr {got} vs {expect}");
        assert_eq!(s.stars.len(), 1);
        assert_eq!(s.candidates, 1);
    }

    #[test]
    fn wider_star_reads_larger() {
        let (w, h) = (100, 100);
        let sharp = frame(w, h, 10.0, &[(50.0, 50.0, 1.5, 500.0)]);
        let soft = frame(w, h, 10.0, &[(50.0, 50.0, 3.0, 500.0)]);
        let st = [star(w, h, 50.0, 50.0, 1.0)];
        let cfg = FocusConfig::default();
        let a = measure(&sharp, 100, 100, &st, &cfg).hfr_px.unwrap();
        let b = measure(&soft, 100, 100, &st, &cfg).hfr_px.unwrap();
        assert!(a < b);
        // Sharpness moves the other way.
        assert!(sharpness(&sharp, 100, 100) > sharpness(&soft, 100, 100));
    }

    #[test]
    fn median_over_stars_and_brightest_first() {
        let (w, h) = (200, 100);
        let px = frame(w, h, 50.0, &[(30.0, 50.0, 1.5, 800.0), (100.0, 50.0, 2.5, 600.0), (170.0, 50.0, 3.5, 400.0)]);
        let st = [
            sized(star(w, h, 30.0, 50.0, 3.0), 1.5),
            sized(star(w, h, 100.0, 50.0, 2.0), 2.5),
            sized(star(w, h, 170.0, 50.0, 1.0), 3.5),
        ];
        let all = measure(&px, 200, 100, &st, &FocusConfig::default());
        assert_eq!(all.stars.len(), 3);
        let mid = 2.5 * (std::f32::consts::PI / 2.0).sqrt();
        assert!((all.hfr_px.unwrap() - mid).abs() < 0.1);
        // Cap at one star: the brightest by mass wins, not the first listed.
        let one = measure(&px, 200, 100, &[st[2], st[0], st[1]], &FocusConfig { max_stars: 1, ..Default::default() });
        assert_eq!(one.stars.len(), 1);
        assert!((one.stars[0].x - st[0].x).abs() < 1e-6);
    }

    #[test]
    fn quality_cuts() {
        let (w, h) = (100, 100);
        let px = frame(w, h, 10.0, &[(50.0, 50.0, 2.0, 66000.0), (5.0, 5.0, 2.0, 500.0), (80.0, 20.0, 2.0, 500.0)]);
        let saturated = star(w, h, 50.0, 50.0, 3.0);
        let edge = star(w, h, 5.0, 5.0, 2.0); // window would run off the frame
        let mut elongated = star(w, h, 80.0, 20.0, 1.0);
        elongated.elongation = Some(3.0);
        let s = measure(
            &px,
            100,
            100,
            &[saturated, edge, elongated],
            &FocusConfig { saturation: Some(65000.0), ..Default::default() },
        );
        assert!(s.hfr_px.is_none());
        assert_eq!(s.candidates, 3);
        // Without the saturation level the bright star is measured.
        let s = measure(&px, 100, 100, &[saturated], &FocusConfig::default());
        assert_eq!(s.stars.len(), 1);
    }

    #[test]
    fn window_grows_with_star_size() {
        // A σ=4 star is 5 px HFR; a fixed 15 px window would clip it and
        // read low. With the width supplied the window follows the star.
        let (w, h) = (120, 120);
        let px = frame(w, h, 20.0, &[(60.0, 60.0, 4.0, 300.0)]);
        let expect = 4.0 * (std::f32::consts::PI / 2.0).sqrt();
        let cfg = FocusConfig::default();
        let fixed = measure(&px, 120, 120, &[star(w, h, 60.0, 60.0, 1.0)], &cfg).hfr_px.unwrap();
        let sized_ = measure(&px, 120, 120, &[sized(star(w, h, 60.0, 60.0, 1.0), 4.0)], &cfg).hfr_px.unwrap();
        assert!(fixed < expect - 0.3, "fixed window should read low: {fixed}");
        assert!((sized_ - expect).abs() < 0.08, "sized {sized_} vs {expect}");
    }

    #[test]
    fn roi_restricts_measurement() {
        let (w, h) = (200, 100);
        let px = frame(w, h, 50.0, &[(30.0, 50.0, 1.5, 800.0), (170.0, 50.0, 3.5, 400.0)]);
        let st = [star(w, h, 30.0, 50.0, 2.0), star(w, h, 170.0, 50.0, 1.0)];
        let s = measure(&px, 200, 100, &st, &FocusConfig { roi: Some([150, 0, 199, 99]), ..Default::default() });
        assert_eq!(s.stars.len(), 1);
        assert!((s.stars[0].x - st[1].x).abs() < 1e-6);
    }

    #[test]
    fn sharpness_noise_floor() {
        // Uncorrelated noise: gradient energy is twice the variance.
        let mut seed = 12345u32;
        let mut px = Vec::with_capacity(512 * 64);
        for _ in 0..512 * 64 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            px.push((seed >> 8) as f32 / (1u32 << 24) as f32);
        }
        let s = sharpness(&px, 512, 64);
        assert!((s - 2.0).abs() < 0.1, "sharpness {s}");
        assert_eq!(sharpness(&vec![7.0; 64], 8, 8), 0.0);
    }

    #[test]
    fn history_tracks_best_and_caps() {
        let mut hist = FocusHistory::new(3);
        let mk = |h: Option<f32>| FocusSample { hfr_px: h, stars: Vec::new(), candidates: 0, sharpness: 0.0 };
        hist.push(&mk(Some(3.0)), None);
        hist.push(&mk(Some(2.0)), Some(100));
        hist.push(&mk(None), None);
        hist.push(&mk(Some(2.5)), None);
        assert_eq!(hist.iter().count(), 3);
        assert_eq!(hist.best().unwrap().hfr_px, Some(2.0));
        assert_eq!(hist.best().unwrap().focuser_pos, Some(100));
        assert_eq!(hist.latest().unwrap().hfr_px, Some(2.5));
        hist.reset();
        assert_eq!(hist.iter().count(), 0);
        assert!(hist.best().is_none());
    }
}
