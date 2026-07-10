//! Color filter array (Bayer mosaic) utilities for RAW color sensors.
//!
//! The viewer streams color cameras as RAW mono-layout frames (see
//! `toupcam_camera` module docs), so the Bayer mosaic is present in the pixel
//! data. Everything here is exact arithmetic on true ADUs — no demosaicking
//! interpolation:
//!
//! - [`compute_cfa_histograms`] subsamples the CFA by channel (R and B are ¼
//!   of pixels each, G the remaining ½) for per-channel histogram overlays.
//! - [`superpixel_bin`] averages each 2×2 cell — (R + G₁ + G₂ + B) / 4 — into
//!   one mono pixel at half resolution, removing the mosaic checkerboard
//!   without inventing data.
//!
//! Both assume the CFA phase is intact: hardware ROI offsets are always even
//! (SDK constraint) so ROI never flips the phase, but hardware binning other
//! than 1×1 scrambles the pattern — callers must skip CFA processing then.

use crate::histogram::Histogram;

/// Bayer pattern order of the top-left 2×2 cell of the frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CfaPattern {
    Rggb,
    Bggr,
    Grbg,
    Gbrg,
}

impl CfaPattern {
    /// Parse a sensor-format FourCC (e.g. from `Toupcam_get_RawFormat`).
    pub fn from_fourcc(fourcc: u32) -> Option<CfaPattern> {
        match &fourcc.to_le_bytes() {
            b"RGGB" => Some(CfaPattern::Rggb),
            b"BGGR" => Some(CfaPattern::Bggr),
            b"GRBG" => Some(CfaPattern::Grbg),
            b"GBRG" => Some(CfaPattern::Gbrg),
            _ => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            CfaPattern::Rggb => "RGGB",
            CfaPattern::Bggr => "BGGR",
            CfaPattern::Grbg => "GRBG",
            CfaPattern::Gbrg => "GBRG",
        }
    }

    /// Color channel (0 = R, 1 = G, 2 = B) of the pixel at `(x, y)`.
    #[inline]
    fn channel_at(self, x: u32, y: u32) -> usize {
        // The 2×2 cell in row-major order: [ (0,0), (1,0), (0,1), (1,1) ].
        const CELLS: [[usize; 4]; 4] = [
            [0, 1, 1, 2], // RGGB
            [2, 1, 1, 0], // BGGR
            [1, 0, 2, 1], // GRBG
            [1, 2, 0, 1], // GBRG
        ];
        CELLS[self as usize][((y & 1) * 2 + (x & 1)) as usize]
    }
}

/// Per-channel (R, G, B) histograms of a raw CFA frame, binned identically to
/// the frame's main histogram (256 bins over the full bit-depth range) so the
/// curves overlay directly. Returns `None` for non-mono-layout images.
///
/// Counts are raw pixel counts: the G curve sits ~2× higher than R/B because
/// green covers half the mosaic.
pub fn compute_cfa_histograms(
    img: &image::DynamicImage,
    pattern: CfaPattern,
    bit_depth: u8,
) -> Option<[Histogram; 3]> {
    const NUM_BINS: usize = 256;
    let (width, height) = (img.width(), img.height());
    let hi = ((1u64 << bit_depth) - 1) as f32;
    let bin_width = hi.max(1.0) / NUM_BINS as f32;

    let mut counts = [[0u64; NUM_BINS]; 3];
    let mut mins = [f32::INFINITY; 3];
    let mut maxs = [f32::NEG_INFINITY; 3];
    let mut accumulate = |x: u32, y: u32, v: f32| {
        let ch = pattern.channel_at(x, y);
        let idx = ((v / bin_width) as usize).min(NUM_BINS - 1);
        counts[ch][idx] += 1;
        if v < mins[ch] { mins[ch] = v; }
        if v > maxs[ch] { maxs[ch] = v; }
    };

    match img {
        image::DynamicImage::ImageLuma8(buf) => {
            for (row, y) in buf.as_raw().chunks_exact(width as usize).zip(0..height) {
                for (x, &v) in (0..width).zip(row) {
                    accumulate(x, y, v as f32);
                }
            }
        }
        image::DynamicImage::ImageLuma16(buf) => {
            for (row, y) in buf.as_raw().chunks_exact(width as usize).zip(0..height) {
                for (x, &v) in (0..width).zip(row) {
                    accumulate(x, y, v as f32);
                }
            }
        }
        _ => return None,
    }

    let edges: Vec<f32> = (0..=NUM_BINS).map(|i| i as f32 * bin_width).collect();
    Some(std::array::from_fn(|ch| Histogram {
        edges: edges.clone(),
        counts: counts[ch].to_vec(),
        data_min: mins[ch],
        data_max: maxs[ch],
    }))
}

/// Average each 2×2 CFA cell into one mono pixel: exact (R + G₁ + G₂ + B) / 4
/// at half resolution, same bit depth. Odd trailing row/column is dropped.
/// Non-mono-layout images pass through unchanged.
pub fn superpixel_bin(img: image::DynamicImage) -> image::DynamicImage {
    use image::DynamicImage;
    match img {
        DynamicImage::ImageLuma16(buf) => {
            let (w2, h2) = (buf.width() / 2, buf.height() / 2);
            if w2 == 0 || h2 == 0 {
                return DynamicImage::ImageLuma16(buf);
            }
            let src = buf.as_raw();
            let stride = buf.width() as usize;
            let mut out = Vec::with_capacity((w2 * h2) as usize);
            for y in 0..h2 as usize {
                let (top, bot) = (2 * y * stride, (2 * y + 1) * stride);
                for x in 0..w2 as usize {
                    let sum = src[top + 2 * x] as u32
                        + src[top + 2 * x + 1] as u32
                        + src[bot + 2 * x] as u32
                        + src[bot + 2 * x + 1] as u32;
                    out.push(((sum + 2) / 4) as u16);
                }
            }
            DynamicImage::ImageLuma16(image::ImageBuffer::from_raw(w2, h2, out).unwrap())
        }
        DynamicImage::ImageLuma8(buf) => {
            let (w2, h2) = (buf.width() / 2, buf.height() / 2);
            if w2 == 0 || h2 == 0 {
                return DynamicImage::ImageLuma8(buf);
            }
            let src = buf.as_raw();
            let stride = buf.width() as usize;
            let mut out = Vec::with_capacity((w2 * h2) as usize);
            for y in 0..h2 as usize {
                let (top, bot) = (2 * y * stride, (2 * y + 1) * stride);
                for x in 0..w2 as usize {
                    let sum = src[top + 2 * x] as u16
                        + src[top + 2 * x + 1] as u16
                        + src[bot + 2 * x] as u16
                        + src[bot + 2 * x + 1] as u16;
                    out.push(((sum + 2) / 4) as u8);
                }
            }
            DynamicImage::ImageLuma8(image::ImageBuffer::from_raw(w2, h2, out).unwrap())
        }
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_roundtrip() {
        let fourcc = u32::from_le_bytes(*b"RGGB");
        assert_eq!(CfaPattern::from_fourcc(fourcc), Some(CfaPattern::Rggb));
        assert_eq!(CfaPattern::from_fourcc(u32::from_le_bytes(*b"GBRG")), Some(CfaPattern::Gbrg));
        assert_eq!(CfaPattern::from_fourcc(0), None);
    }

    #[test]
    fn channel_layout() {
        // RGGB: (0,0)=R (1,0)=G (0,1)=G (1,1)=B, repeating with period 2.
        let p = CfaPattern::Rggb;
        assert_eq!(p.channel_at(0, 0), 0);
        assert_eq!(p.channel_at(1, 0), 1);
        assert_eq!(p.channel_at(0, 1), 1);
        assert_eq!(p.channel_at(1, 1), 2);
        assert_eq!(p.channel_at(2, 2), 0);
        // BGGR is RGGB with R and B swapped.
        assert_eq!(CfaPattern::Bggr.channel_at(0, 0), 2);
        assert_eq!(CfaPattern::Bggr.channel_at(1, 1), 0);
    }

    #[test]
    fn superpixel_exact_average() {
        // One 2×2 cell: (100 + 200 + 300 + 401) / 4 = 250.25 → rounds to 250.
        let buf = image::ImageBuffer::from_raw(2, 2, vec![100u16, 200, 300, 401]).unwrap();
        let out = superpixel_bin(image::DynamicImage::ImageLuma16(buf));
        let image::DynamicImage::ImageLuma16(out) = out else { panic!("wrong variant") };
        assert_eq!((out.width(), out.height()), (1, 1));
        assert_eq!(out.as_raw(), &vec![250u16]);
    }

    #[test]
    fn superpixel_drops_odd_edges() {
        let buf = image::ImageBuffer::from_raw(3, 3, vec![10u16; 9]).unwrap();
        let out = superpixel_bin(image::DynamicImage::ImageLuma16(buf));
        assert_eq!((out.width(), out.height()), (1, 1));
    }

    #[test]
    fn cfa_histogram_counts_by_channel() {
        // 4×4 RGGB frame: R pixels hold 10, G hold 100, B hold 1000.
        let p = CfaPattern::Rggb;
        let vals: Vec<u16> = (0..16)
            .map(|i| match p.channel_at(i % 4, i / 4) {
                0 => 10u16,
                1 => 100,
                _ => 1000,
            })
            .collect();
        let buf = image::ImageBuffer::from_raw(4, 4, vals).unwrap();
        let img = image::DynamicImage::ImageLuma16(buf);
        let [r, g, b] = compute_cfa_histograms(&img, p, 16).unwrap();
        // Green covers half the mosaic, R and B a quarter each.
        assert_eq!(r.counts.iter().sum::<u64>(), 4);
        assert_eq!(g.counts.iter().sum::<u64>(), 8);
        assert_eq!(b.counts.iter().sum::<u64>(), 4);
        assert_eq!((r.data_min, r.data_max), (10.0, 10.0));
        assert_eq!((g.data_min, g.data_max), (100.0, 100.0));
        assert_eq!((b.data_min, b.data_max), (1000.0, 1000.0));
        // Binning matches the main histogram: 256 bins over [0, 65535].
        assert_eq!(r.counts.len(), 256);
        assert_eq!(b.counts[(1000.0 / (65535.0 / 256.0)) as usize], 4);
    }
}
