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

pub fn compute_histogram(data: &[f32], num_bins: usize) -> Histogram {
    if data.is_empty() {
        return Histogram {
            edges: vec![0.0; num_bins + 1],
            counts: vec![0; num_bins],
            data_min: 0.0,
            data_max: 0.0,
        };
    }

    let mut data_min = f32::INFINITY;
    let mut data_max = f32::NEG_INFINITY;
    for &v in data {
        if v < data_min { data_min = v; }
        if v > data_max { data_max = v; }
    }

    if data_max <= data_min {
        data_max = data_min + 1.0;
    }

    let bin_width = (data_max - data_min) / num_bins as f32;
    let mut edges = Vec::with_capacity(num_bins + 1);
    for i in 0..=num_bins {
        edges.push(data_min + i as f32 * bin_width);
    }

    let mut counts = vec![0u64; num_bins];
    for &v in data {
        let idx = ((v - data_min) / bin_width) as usize;
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
