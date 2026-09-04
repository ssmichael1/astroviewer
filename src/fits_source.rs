//! FITS file playback source.
//!
//! Frames stay in their native integer width where the file allows it: BITPIX 8,
//! and BITPIX 16 with the conventional unsigned offset (BZERO 32768) or with no
//! offset and no negative samples, become `u16` frames — the same type the
//! integer cameras produce, so playback shares their copy-free histogram and
//! colormap paths. Everything else (floats, 32/64-bit integers, arbitrary
//! BSCALE/BZERO) becomes `f32`. Either way the frames are `Arc`-shared: playback
//! hands out the same buffers every loop without copying, and the background
//! estimator works from the same store instead of re-reading the file.

use std::cmp::Ordering;
use std::sync::Arc;

use anyhow::{anyhow, Result};
use fitskit::{FitsFile, HduData, PixelData};
use rayon::prelude::*;

use crate::pixels::Pixels;

/// Every frame of a loaded file, all in one storage type.
pub enum FitsFrames {
    U16(Vec<Arc<Vec<u16>>>),
    F32(Vec<Arc<Vec<f32>>>),
}

impl FitsFrames {
    pub fn num_frames(&self) -> usize {
        match self {
            FitsFrames::U16(v) => v.len(),
            FitsFrames::F32(v) => v.len(),
        }
    }

    /// Frame `i` as a shared pixel buffer (a refcount bump, no copy).
    pub fn frame(&self, i: usize) -> Pixels {
        match self {
            FitsFrames::U16(v) => Pixels::U16(v[i].clone()),
            FitsFrames::F32(v) => Pixels::F32(v[i].clone()),
        }
    }

    /// Largest sample anywhere in the file, for bit-depth inference.
    fn max_value(&self) -> f64 {
        match self {
            FitsFrames::U16(v) => v
                .par_iter()
                .map(|f| f.iter().copied().max().unwrap_or(0))
                .max()
                .unwrap_or(0) as f64,
            FitsFrames::F32(v) => v
                .par_iter()
                .map(|f| f.iter().copied().fold(f32::NEG_INFINITY, f32::max))
                .reduce(|| f32::NEG_INFINITY, f32::max) as f64,
        }
    }

    /// Per-pixel percentile across all frames — the temporal background.
    /// `percentile` in 0.0..=1.0 (e.g. 0.35 for the 35th percentile), linearly
    /// interpolated between the two neighbouring order statistics.
    pub fn compute_background(&self, percentile: f32) -> Vec<f32> {
        match self {
            FitsFrames::U16(v) => percentile_background(v, percentile),
            FitsFrames::F32(v) => percentile_background(v, percentile),
        }
    }
}

/// Temporal percentile per pixel.
///
/// Works a tile of pixels at a time: the tile's columns (one per pixel, one
/// entry per frame) are gathered into a contiguous scratch buffer so every
/// frame is read sequentially, then each column is resolved with a linear-time
/// selection rather than a sort. Tiles run in parallel. On a 219-frame
/// 1920×1080 cube this is ~20× faster than the per-pixel sort it replaces.
fn percentile_background<T>(frames: &[Arc<Vec<T>>], percentile: f32) -> Vec<f32>
where
    T: Copy + PartialOrd + Into<f64> + Send + Sync,
{
    let nframes = frames.len();
    let npix = frames.first().map_or(0, |f| f.len());
    if nframes == 0 || npix == 0 {
        return Vec::new();
    }
    let p = (percentile.clamp(0.0, 1.0) as f64) * (nframes - 1) as f64;
    let lo = p.floor() as usize;
    let hi = (lo + 1).min(nframes - 1);
    let frac = p - lo as f64;
    let cmp = |a: &T, b: &T| a.partial_cmp(b).unwrap_or(Ordering::Equal);

    const TILE: usize = 2048;
    let mut bg = vec![0.0f32; npix];
    bg.par_chunks_mut(TILE).enumerate().for_each(|(ti, out)| {
        let start = ti * TILE;
        let len = out.len();
        // col[j * nframes + i] = frame i, pixel start + j.
        let mut col = vec![frames[0][0]; len * nframes];
        for (i, f) in frames.iter().enumerate() {
            for (j, &v) in f[start..start + len].iter().enumerate() {
                col[j * nframes + i] = v;
            }
        }
        for (j, out_px) in out.iter_mut().enumerate() {
            let vals = &mut col[j * nframes..(j + 1) * nframes];
            let (_, v_lo, rest) = vals.select_nth_unstable_by(lo, cmp);
            let v_lo: f64 = (*v_lo).into();
            let v_hi: f64 = if hi > lo {
                // Everything after the pivot is >= it, so the smallest of the
                // rest is the next order statistic.
                rest.iter()
                    .copied()
                    .reduce(|m, x| if cmp(&x, &m) == Ordering::Less { x } else { m })
                    .map_or(v_lo, Into::into)
            } else {
                v_lo
            };
            *out_px = (v_lo * (1.0 - frac) + v_hi * frac) as f32;
        }
    });
    bg
}

/// One HDU's pixels after BSCALE/BZERO, before slicing into frames.
enum Raw {
    U16(Vec<u16>),
    F32(Vec<f32>),
}

/// Apply BSCALE/BZERO, keeping integer data as `u16` whenever the result is
/// exactly representable there; otherwise widen to `f32` (the precision the
/// viewer displayed before, when every frame went through `f64` then `f32`).
fn convert(px: PixelData, bscale: f64, bzero: f64) -> Raw {
    let integral = |x: f64| x == x.trunc();
    let scale = |x: f64| (bzero + bscale * x) as f32;
    match px {
        PixelData::U8(v) => {
            if bscale == 1.0 && integral(bzero) && bzero >= 0.0 && bzero + 255.0 <= 65535.0 {
                let off = bzero as u16;
                Raw::U16(v.into_iter().map(|x| x as u16 + off).collect())
            } else {
                Raw::F32(v.into_iter().map(|x| scale(x as f64)).collect())
            }
        }
        PixelData::I16(v) => {
            if bscale == 1.0 && bzero == 32768.0 {
                // The standard unsigned-16 encoding.
                Raw::U16(v.into_iter().map(|x| (x as i32 + 32768) as u16).collect())
            } else if bscale == 1.0 && bzero == 0.0 && v.iter().all(|&x| x >= 0) {
                Raw::U16(v.into_iter().map(|x| x as u16).collect())
            } else {
                Raw::F32(v.into_iter().map(|x| scale(x as f64)).collect())
            }
        }
        PixelData::I32(v) => Raw::F32(v.into_iter().map(|x| scale(x as f64)).collect()),
        PixelData::I64(v) => Raw::F32(v.into_iter().map(|x| scale(x as f64)).collect()),
        PixelData::F32(v) => Raw::F32(v.into_iter().map(|x| scale(x as f64)).collect()),
        PixelData::F64(v) => Raw::F32(v.into_iter().map(scale).collect()),
    }
}

/// Split an HDU's pixel run into `npix`-sized frames. A single-frame HDU is
/// moved, not copied; a cube's slices are copied out once.
fn split_frames<T: Copy>(v: Vec<T>, npix: usize) -> Vec<Arc<Vec<T>>> {
    if v.len() == npix {
        vec![Arc::new(v)]
    } else {
        v.chunks_exact(npix).map(|c| Arc::new(c.to_vec())).collect()
    }
}

/// A FITS-file-based image source that cycles through frames.
pub struct FitsSource {
    frames: Arc<FitsFrames>,
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

        let mut raw_frames: Vec<Raw> = Vec::new();
        let mut width = 0u32;
        let mut height = 0u32;

        for hdu in fits.hdus {
            let img = match hdu.data {
                HduData::Image(im) if im.axes.len() >= 2 => im,
                _ => continue,
            };

            let w = img.axes[0] as u32;
            let h = img.axes[1] as u32;
            if raw_frames.is_empty() {
                width = w;
                height = h;
            } else if w != width || h != height {
                continue;
            }
            let npix = (w as usize) * (h as usize);

            let bscale = hdu.header.get_float("BSCALE").unwrap_or(1.0);
            let bzero = hdu.header.get_float("BZERO").unwrap_or(0.0);
            match convert(img.pixels, bscale, bzero) {
                Raw::U16(v) => raw_frames.extend(split_frames(v, npix).into_iter().map(|f| {
                    Raw::U16(Arc::try_unwrap(f).unwrap_or_else(|a| (*a).clone()))
                })),
                Raw::F32(v) => raw_frames.extend(split_frames(v, npix).into_iter().map(|f| {
                    Raw::F32(Arc::try_unwrap(f).unwrap_or_else(|a| (*a).clone()))
                })),
            }
        }

        if raw_frames.is_empty() {
            return Err(anyhow!("No image data found in FITS file"));
        }

        // One storage type for the whole file: if any HDU needed f32, widen the
        // rest so frame order is preserved and every frame plays the same way.
        let frames = if raw_frames.iter().all(|r| matches!(r, Raw::U16(_))) {
            FitsFrames::U16(
                raw_frames
                    .into_iter()
                    .map(|r| match r { Raw::U16(v) => Arc::new(v), Raw::F32(_) => unreachable!() })
                    .collect(),
            )
        } else {
            FitsFrames::F32(
                raw_frames
                    .into_par_iter()
                    .map(|r| match r {
                        Raw::F32(v) => Arc::new(v),
                        Raw::U16(v) => Arc::new(v.iter().map(|&x| x as f32).collect()),
                    })
                    .collect(),
            )
        };

        // Infer the sensor bit depth from the largest sample, so a 12-bit
        // camera recorded into 16-bit words still scales as 12-bit.
        let max_val = frames.max_value();
        let inferred_depth = if max_val <= 255.0 { 8 }
            else if max_val <= 4095.0 { 12 }
            else if max_val <= 16383.0 { 14 }
            else if max_val <= 65535.0 { 16 }
            else { 32 };

        Ok(Self {
            frames: Arc::new(frames),
            width,
            height,
            bit_depth: inferred_depth,
            current: 0,
        })
    }

    pub fn num_frames(&self) -> usize {
        self.frames.num_frames()
    }

    /// The shared frame store, for the background estimator.
    pub fn frames(&self) -> Arc<FitsFrames> {
        Arc::clone(&self.frames)
    }

    /// The next frame, cycling back to the start. A shared handle, not a copy.
    ///
    /// Values are the BSCALE/BZERO-corrected physical values; they are *not*
    /// clamped to a display bit depth here — display scaling happens downstream.
    pub fn next_frame(&mut self) -> Pixels {
        let p = self.frames.frame(self.current);
        self.current = (self.current + 1) % self.frames.num_frames();
        p
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference implementation: per-pixel sort, as the viewer used to do.
    fn reference_background<T: Copy + Into<f64>>(frames: &[Arc<Vec<T>>], percentile: f32) -> Vec<f32> {
        let n = frames.len();
        let npix = frames[0].len();
        let p = (percentile as f64) * (n - 1) as f64;
        let lo = p.floor() as usize;
        let hi = (lo + 1).min(n - 1);
        let frac = p - lo as f64;
        (0..npix)
            .map(|i| {
                let mut col: Vec<f64> = frames.iter().map(|f| f[i].into()).collect();
                col.sort_by(|a, b| a.partial_cmp(b).unwrap());
                (col[lo] * (1.0 - frac) + col[hi] * frac) as f32
            })
            .collect()
    }

    #[test]
    fn percentile_background_matches_sort_reference() {
        // Odd sizes so tiles are partial and interpolation is exercised.
        let (npix, nframes) = (2048 * 2 + 37, 13);
        let frames: Vec<Arc<Vec<u16>>> = (0..nframes)
            .map(|k| Arc::new((0..npix).map(|i| ((i * 7919 + k * 104729) % 4096) as u16).collect()))
            .collect();
        for pct in [0.0, 0.1, 0.35, 0.5, 0.9, 1.0] {
            let got = percentile_background(&frames, pct);
            let want = reference_background(&frames, pct);
            assert_eq!(got.len(), want.len());
            for (i, (g, w)) in got.iter().zip(&want).enumerate() {
                assert!((g - w).abs() < 1e-3, "pct {pct} pixel {i}: {g} vs {w}");
            }
        }
        // f32 frames too, including negatives.
        let ff: Vec<Arc<Vec<f32>>> = frames
            .iter()
            .map(|f| Arc::new(f.iter().map(|&x| x as f32 - 2000.0).collect()))
            .collect();
        let got = percentile_background(&ff, 0.35);
        let want = reference_background(&ff, 0.35);
        for (g, w) in got.iter().zip(&want) {
            assert!((g - w).abs() < 1e-3);
        }
    }

    #[test]
    fn i16_with_unsigned_offset_stays_u16() {
        let px = PixelData::I16(vec![-32768, -32767, 0, 32767]);
        match convert(px, 1.0, 32768.0) {
            Raw::U16(v) => assert_eq!(v, vec![0, 1, 32768, 65535]),
            Raw::F32(_) => panic!("expected u16"),
        }
    }

    #[test]
    fn i16_signed_or_scaled_widens() {
        match convert(PixelData::I16(vec![-5, 10]), 1.0, 0.0) {
            Raw::F32(v) => assert_eq!(v, vec![-5.0, 10.0]),
            Raw::U16(_) => panic!("negative samples must widen"),
        }
        match convert(PixelData::I16(vec![2, 4]), 0.5, 100.0) {
            Raw::F32(v) => assert_eq!(v, vec![101.0, 102.0]),
            Raw::U16(_) => panic!("fractional scale must widen"),
        }
        match convert(PixelData::I16(vec![2, 4]), 1.0, 0.0) {
            Raw::U16(v) => assert_eq!(v, vec![2, 4]),
            Raw::F32(_) => panic!("non-negative unscaled i16 fits u16"),
        }
    }
}
