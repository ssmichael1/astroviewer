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
