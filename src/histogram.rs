/// Histogram computation for mono image data.

pub struct Histogram {
    /// Bin edges (length = num_bins + 1)
    pub edges: Vec<f64>,
    /// Bin counts (length = num_bins)
    pub counts: Vec<u64>,
    /// Data min value seen
    pub data_min: f64,
    /// Data max value seen
    pub data_max: f64,
}

impl Histogram {
    pub fn num_bins(&self) -> usize {
        self.counts.len()
    }

    /// Bin centers for plotting
    pub fn centers(&self) -> Vec<f64> {
        self.edges
            .windows(2)
            .map(|w| (w[0] + w[1]) * 0.5)
            .collect()
    }
}

/// Compute a histogram with `num_bins` linearly-spaced bins.
/// Works on any pixel data convertible to f64.
pub fn compute_histogram(data: &[f64], num_bins: usize) -> Histogram {
    if data.is_empty() {
        return Histogram {
            edges: vec![0.0; num_bins + 1],
            counts: vec![0; num_bins],
            data_min: 0.0,
            data_max: 0.0,
        };
    }

    let mut data_min = f64::INFINITY;
    let mut data_max = f64::NEG_INFINITY;
    for &v in data {
        if v < data_min {
            data_min = v;
        }
        if v > data_max {
            data_max = v;
        }
    }

    // Avoid zero-width bins
    if data_max <= data_min {
        data_max = data_min + 1.0;
    }

    let bin_width = (data_max - data_min) / num_bins as f64;
    let mut edges = Vec::with_capacity(num_bins + 1);
    for i in 0..=num_bins {
        edges.push(data_min + i as f64 * bin_width);
    }

    let mut counts = vec![0u64; num_bins];
    for &v in data {
        let idx = ((v - data_min) / bin_width) as usize;
        let idx = idx.min(num_bins - 1);
        counts[idx] += 1;
    }

    Histogram {
        edges,
        counts,
        data_min,
        data_max,
    }
}

/// Compute mean and standard deviation from f64 data.
pub fn compute_stats(data: &[f64]) -> (f64, f64) {
    if data.is_empty() {
        return (0.0, 0.0);
    }
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    let var = data.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>() / n;
    (mean, var.sqrt())
}
