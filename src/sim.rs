use image::{DynamicImage, GrayImage, Luma};
use std::time::Instant;

/// Simulated camera source that generates synthetic frames.
pub struct SimCamera {
    width: u32,
    height: u32,
    frame_count: u64,
    start_time: Instant,
    bit_depth: u8,
}

impl SimCamera {
    pub fn new(width: u32, height: u32, bit_depth: u8) -> Self {
        Self {
            width,
            height,
            frame_count: 0,
            start_time: Instant::now(),
            bit_depth,
        }
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn bit_depth(&self) -> u8 {
        self.bit_depth
    }

    /// Generate the next frame as a DynamicImage.
    pub fn next_frame(&mut self) -> DynamicImage {
        let t = self.start_time.elapsed().as_secs_f64();
        self.frame_count += 1;

        let max_val = ((1u32 << self.bit_depth) - 1) as f64;
        let base_level = max_val * 0.15;
        let blob_amplitude = max_val * 0.7;

        let cx = self.width as f64 * (0.5 + 0.3 * (t * 0.5).sin());
        let cy = self.height as f64 * (0.5 + 0.3 * (t * 0.37).cos());
        let sigma = self.width as f64 * 0.08;
        let inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

        if self.bit_depth <= 8 {
            let mut img = GrayImage::new(self.width, self.height);
            // Simple LCG for fast pseudo-random noise
            let mut rng_state: u64 = self.frame_count.wrapping_mul(6364136223846793005).wrapping_add(1);
            for y in 0..self.height {
                for x in 0..self.width {
                    let dx = x as f64 - cx;
                    let dy = y as f64 - cy;
                    let blob = blob_amplitude * (-((dx * dx + dy * dy) * inv_2sigma2)).exp();
                    rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let noise = ((rng_state >> 33) as f64 / u32::MAX as f64 - 0.5) * max_val * 0.05;
                    let val = (base_level + blob + noise).clamp(0.0, max_val) as u8;
                    img.put_pixel(x, y, Luma([val]));
                }
            }
            DynamicImage::ImageLuma8(img)
        } else {
            let mut img = image::ImageBuffer::<Luma<u16>, Vec<u16>>::new(self.width, self.height);
            let mut rng_state: u64 = self.frame_count.wrapping_mul(6364136223846793005).wrapping_add(1);
            for y in 0..self.height {
                for x in 0..self.width {
                    let dx = x as f64 - cx;
                    let dy = y as f64 - cy;
                    let blob = blob_amplitude * (-((dx * dx + dy * dy) * inv_2sigma2)).exp();
                    rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let noise = ((rng_state >> 33) as f64 / u32::MAX as f64 - 0.5) * max_val * 0.05;
                    let val = (base_level + blob + noise).clamp(0.0, max_val) as u16;
                    img.put_pixel(x, y, Luma([val]));
                }
            }
            DynamicImage::ImageLuma16(img)
        }
    }
}
