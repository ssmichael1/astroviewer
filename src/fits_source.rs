use anyhow::{anyhow, Result};
use fits4::{FitsFile, HduData};
use image::{DynamicImage, ImageBuffer, Luma};

/// A FITS-file-based image source that cycles through frames.
pub struct FitsSource {
    frames: Vec<Vec<f64>>,
    pub width: u32,
    pub height: u32,
    pub bit_depth: u8,
    current: usize,
}

impl FitsSource {
    /// Load a FITS file. Supports:
    /// - 2D image (NAXIS=2): single frame, repeated
    /// - 3D cube (NAXIS=3): multiple frames along axis 3
    /// - Multi-HDU: each image HDU becomes a frame
    pub fn from_file(path: &str) -> Result<Self> {
        let fits = FitsFile::from_file(path)?;

        let mut frames = Vec::new();
        let mut width = 0u32;
        let mut height = 0u32;

        for hdu in fits.iter() {
            let img = match &hdu.data {
                HduData::Image(im) if im.axes.len() >= 2 => im,
                _ => continue,
            };

            let w = img.axes[0] as u32;
            let h = img.axes[1] as u32;

            if frames.is_empty() {
                width = w;
                height = h;
            } else if w != width || h != height {
                continue;
            }

            let pixels_per_frame = (w * h) as usize;
            let nslices = if img.axes.len() >= 3 { img.axes[2] } else { 1 };

            // scaled_values applies BZERO/BSCALE automatically
            let bscale = hdu.header.get_float("BSCALE").unwrap_or(1.0);
            let bzero = hdu.header.get_float("BZERO").unwrap_or(0.0);
            let all_pixels = img.scaled_values(bscale, bzero);

            for s in 0..nslices {
                let start = s * pixels_per_frame;
                let end = start + pixels_per_frame;
                if end <= all_pixels.len() {
                    frames.push(all_pixels[start..end].to_vec());
                }
            }
        }

        if frames.is_empty() {
            return Err(anyhow!("No image data found in FITS file"));
        }

        // Infer actual bit depth from maximum pixel value
        let max_val = frames.iter()
            .flat_map(|f| f.iter())
            .copied()
            .fold(0.0_f64, f64::max);
        let inferred_depth = if max_val <= 255.0 { 8 }
            else if max_val <= 4095.0 { 12 }
            else if max_val <= 16383.0 { 14 }
            else if max_val <= 65535.0 { 16 }
            else { 32 };

        Ok(Self {
            frames,
            width,
            height,
            bit_depth: inferred_depth,
            current: 0,
        })
    }

    pub fn num_frames(&self) -> usize {
        self.frames.len()
    }

    /// Compute per-pixel percentile across all frames.
    /// `percentile` should be in 0.0..1.0 (e.g. 0.35 for 35th percentile).
    pub fn compute_background(&self, percentile: f32) -> Vec<f32> {
        let npix = (self.width as usize) * (self.height as usize);
        let nframes = self.frames.len();
        let mut bg = vec![0.0f32; npix];
        let mut col = vec![0.0f64; nframes];
        let p = (percentile.clamp(0.0, 1.0) as f64) * (nframes - 1) as f64;
        let lo = p.floor() as usize;
        let hi = (lo + 1).min(nframes - 1);
        let frac = p - lo as f64;
        for i in 0..npix {
            for (j, frame) in self.frames.iter().enumerate() {
                col[j] = frame[i];
            }
            col.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            bg[i] = (col[lo] * (1.0 - frac) + col[hi] * frac) as f32;
        }
        bg
    }

    /// Return the next frame as a DynamicImage, cycling back to the start.
    pub fn next_frame(&mut self) -> DynamicImage {
        let mono = &self.frames[self.current];
        self.current = (self.current + 1) % self.frames.len();

        if self.bit_depth <= 8 {
            let pixels: Vec<u8> = mono.iter().map(|&v| v.clamp(0.0, 255.0) as u8).collect();
            let buf = ImageBuffer::<Luma<u8>, _>::from_raw(self.width, self.height, pixels).unwrap();
            DynamicImage::ImageLuma8(buf)
        } else {
            let pixels: Vec<u16> = mono.iter().map(|&v| v.clamp(0.0, 65535.0) as u16).collect();
            let buf = ImageBuffer::<Luma<u16>, _>::from_raw(self.width, self.height, pixels).unwrap();
            DynamicImage::ImageLuma16(buf)
        }
    }
}
