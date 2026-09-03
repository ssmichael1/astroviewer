/// Histogram computation for mono image data.
pub struct Histogram {
    pub edges: Vec<f32>,
    pub counts: Vec<u64>,
    pub data_min: f32,
    pub data_max: f32,
}

impl Histogram {
    #[allow(dead_code)]
    pub fn num_bins(&self) -> usize {
        self.counts.len()
    }

    pub fn centers(&self) -> Vec<f32> {
        self.edges
            .windows(2)
            .map(|w| (w[0] + w[1]) * 0.5)
            .collect()
    }

    /// Value at cumulative fraction `frac` in [0,1] (0.5 = median),
    /// linearly interpolated within the containing bin.
    pub fn percentile(&self, frac: f32) -> f32 {
        let total: u64 = self.counts.iter().sum();
        if total == 0 {
            return self.data_min;
        }
        let target = frac.clamp(0.0, 1.0) as f64 * total as f64;
        let mut cum = 0u64;
        for (i, &c) in self.counts.iter().enumerate() {
            if c > 0 && (cum + c) as f64 >= target {
                let within = ((target - cum as f64) / c as f64) as f32;
                return self.edges[i] + within * (self.edges[i + 1] - self.edges[i]);
            }
            cum += c;
        }
        self.data_max
    }
}

/// Compute histogram over the given fixed range `[range_min, range_max]`.
/// Values outside the range are clamped into the first/last bin.
pub fn compute_histogram(data: &[f32], num_bins: usize, range_min: f32, range_max: f32) -> Histogram {
    let mut data_min = f32::INFINITY;
    let mut data_max = f32::NEG_INFINITY;
    for &v in data {
        if v < data_min { data_min = v; }
        if v > data_max { data_max = v; }
    }

    let lo = if range_min < range_max { range_min } else { data_min };
    let hi = if range_min < range_max { range_max } else { data_max };
    let (lo, hi) = if hi <= lo { (lo, lo + 1.0) } else { (lo, hi) };

    let bin_width = (hi - lo) / num_bins as f32;
    let mut edges = Vec::with_capacity(num_bins + 1);
    for i in 0..=num_bins {
        edges.push(lo + i as f32 * bin_width);
    }

    let mut counts = vec![0u64; num_bins];
    for &v in data {
        let idx = ((v - lo) / bin_width) as usize;
        let idx = idx.min(num_bins - 1);
        counts[idx] += 1;
    }

    Histogram { edges, counts, data_min, data_max }
}

pub fn compute_stats(data: &[f32]) -> (f32, f32) {
    if data.is_empty() {
        return (0.0, 0.0);
    }
    let n = data.len() as f32;
    let mean = data.iter().sum::<f32>() / n;
    let var = data.iter().map(|&v| (v - mean) * (v - mean)).sum::<f32>() / n;
    (mean, var.sqrt())
}

/// Fused single-pass equivalent of `compute_histogram(data, num_bins,
/// range_min, range_max)` followed by `compute_stats(data)`.
///
/// The histogram range is fixed by the caller (not derived from the data), so
/// the bin edges are known up front and the min/max, sum, sum-of-squares, and
/// per-bin counts can all be accumulated in a single pass over the pixels
/// instead of the four passes the two functions make separately. At full sensor
/// resolution and high frame rates that is the difference between one and four
/// walks over ~10 MB of `f32` per frame.
///
/// Results match the separate functions: identical histogram (same edges, same
/// `data_min`/`data_max`, same bin clamp) and the same population-variance
/// definition for the standard deviation. The mean/variance are accumulated in
/// `f64` (sum and sum-of-squares) so the single pass stays numerically stable;
/// this can differ from the two-pass `f32` `compute_stats` only in the last few
/// ulps.
///
/// Only the common `range_min < range_max` case is fused; a degenerate or empty
/// range would make the edges depend on the data (forcing a min/max pass first),
/// so those fall back to the original functions to preserve their exact
/// behavior.
pub fn compute_histogram_and_stats(
    data: &[f32],
    num_bins: usize,
    range_min: f32,
    range_max: f32,
) -> (Histogram, f32, f32) {
    // Degenerate range or no data: defer to the originals so edge behavior is
    // byte-for-byte identical (and these paths are not the hot path). The
    // fast path needs a proper range so the bin edges don't depend on the data.
    let usable_range = range_min < range_max;
    if data.is_empty() || num_bins == 0 || !usable_range {
        let hist = compute_histogram(data, num_bins, range_min, range_max);
        let (mean, stddev) = compute_stats(data);
        return (hist, mean, stddev);
    }

    // range_min < range_max, so lo/hi are fixed and hi > lo (no degenerate
    // adjustment needed) — exactly what compute_histogram derives for this case.
    let lo = range_min;
    let hi = range_max;
    let bin_width = (hi - lo) / num_bins as f32;
    let mut edges = Vec::with_capacity(num_bins + 1);
    for i in 0..=num_bins {
        edges.push(lo + i as f32 * bin_width);
    }

    let mut counts = vec![0u64; num_bins];
    let mut data_min = f32::INFINITY;
    let mut data_max = f32::NEG_INFINITY;
    let mut sum = 0.0f64;
    let mut sum_sq = 0.0f64;
    for &v in data {
        if v < data_min {
            data_min = v;
        }
        if v > data_max {
            data_max = v;
        }
        // Same binning as compute_histogram: negative offsets saturate to 0 on
        // the `as usize` cast, large ones clamp to the last bin.
        let idx = ((v - lo) / bin_width) as usize;
        let idx = idx.min(num_bins - 1);
        counts[idx] += 1;
        let vd = v as f64;
        sum += vd;
        sum_sq += vd * vd;
    }

    let n = data.len() as f64;
    let mean = sum / n;
    // Population variance via E[x^2] - E[x]^2; clamp tiny negative results from
    // floating-point cancellation to zero before the sqrt.
    let var = (sum_sq / n - mean * mean).max(0.0);
    let hist = Histogram { edges, counts, data_min, data_max };
    (hist, mean as f32, var.sqrt() as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fused pass must reproduce the separate histogram/stats functions.
    fn assert_matches(data: &[f32], num_bins: usize, lo: f32, hi: f32) {
        let hist_ref = compute_histogram(data, num_bins, lo, hi);
        let (mean_ref, std_ref) = compute_stats(data);
        let (hist, mean, stddev) = compute_histogram_and_stats(data, num_bins, lo, hi);

        assert_eq!(hist.counts, hist_ref.counts, "bin counts differ");
        assert_eq!(hist.edges.len(), hist_ref.edges.len(), "edge count differs");
        for (a, b) in hist.edges.iter().zip(hist_ref.edges.iter()) {
            assert!((a - b).abs() <= 1e-3, "edge {a} vs {b}");
        }
        assert_eq!(hist.data_min, hist_ref.data_min, "data_min differs");
        assert_eq!(hist.data_max, hist_ref.data_max, "data_max differs");

        // f64 accumulation vs the two-pass f32 originals: allow a small
        // tolerance scaled to the magnitude of the values.
        let scale = 1.0 + mean_ref.abs() + std_ref.abs();
        assert!((mean - mean_ref).abs() <= 1e-3 * scale, "mean {mean} vs {mean_ref}");
        assert!((stddev - std_ref).abs() <= 1e-2 * scale, "stddev {stddev} vs {std_ref}");
    }

    #[test]
    fn fused_matches_gradient() {
        let data: Vec<f32> = (0..4096).map(|i| i as f32).collect();
        assert_matches(&data, 256, 0.0, 4095.0);
    }

    #[test]
    fn fused_matches_16bit_range() {
        // A noisy-ish pattern spanning a 16-bit range.
        let data: Vec<f32> = (0..10_000)
            .map(|i| ((i * 6151) % 65536) as f32)
            .collect();
        assert_matches(&data, 256, 0.0, 65535.0);
    }

    #[test]
    fn fused_matches_constant() {
        let data = vec![1234.0f32; 5000];
        assert_matches(&data, 256, 0.0, 4095.0);
    }

    #[test]
    fn fused_matches_out_of_range_values() {
        // Values below lo and above hi must clamp into the first/last bins,
        // exactly as compute_histogram does.
        let mut data: Vec<f32> = vec![-50.0, -1.0, 0.0, 2047.0, 4094.0, 4095.0, 9000.0];
        data.extend((0..1000).map(|i| (i % 4096) as f32));
        assert_matches(&data, 256, 0.0, 4095.0);
    }

    #[test]
    fn fused_empty_falls_back() {
        let data: [f32; 0] = [];
        let (_, mean, stddev) = compute_histogram_and_stats(&data, 256, 0.0, 4095.0);
        assert_eq!((mean, stddev), (0.0, 0.0));
    }

    #[test]
    fn fused_degenerate_range_falls_back() {
        // range_min >= range_max: must match the data-derived-range original.
        let data: Vec<f32> = (0..2000).map(|i| (i % 500) as f32).collect();
        assert_matches(&data, 256, 5.0, 5.0);
        assert_matches(&data, 256, 100.0, 10.0);
    }
}
