//! Times centroid extraction on a real FITS frame: memory-bandwidth floor,
//! fast path at several bg_grid sizes, and the CCL path for reference.
//!
//! Usage: cargo run --release --features starsolve --example extract_bench <file.fits> [sigma]

use std::time::Instant;

fn time_ms(iters: usize, mut f: impl FnMut() -> usize) -> (f32, usize) {
    let mut best = f32::INFINITY;
    let mut out = 0;
    for _ in 0..iters {
        let t = Instant::now();
        out = f();
        best = best.min(t.elapsed().as_secs_f32() * 1000.0);
    }
    (best, out)
}

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).expect("usage: extract_bench <file.fits> [sigma]");
    let sigma: f32 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(3.6);

    let fits = fitskit::FitsFile::from_file(&path)?;
    let hdu = fits
        .iter()
        .find(|h| matches!(&h.data, fitskit::HduData::Image(im) if im.axes.len() >= 2))
        .expect("no image HDU");
    let im = match &hdu.data {
        fitskit::HduData::Image(im) => im,
        _ => unreachable!(),
    };
    let (w, h) = (im.axes[0] as u32, im.axes[1] as u32);
    let bscale = hdu.header.get_float("BSCALE").unwrap_or(1.0);
    let bzero = hdu.header.get_float("BZERO").unwrap_or(0.0);
    let px: Vec<f32> = im.scaled_values(bscale, bzero)[..(w * h) as usize]
        .iter()
        .map(|&v| v as f32)
        .collect();
    println!("{w}x{h}, {} Mpix, sigma_threshold {sigma}", (w * h) as f32 / 1e6);

    // Memory-bandwidth floor: one sequential read of every pixel.
    let (ms, n) = time_ms(100, || {
        px.iter().fold(0.0f32, |a, &b| a.max(b)) as usize
    });
    println!("read-only pass          {ms:6.2} ms   (max={n})");

    for bg_grid in [16u32, 32, 64, 128] {
        let cfg = tetra3::FastCentroidConfig {
            sigma_threshold: sigma,
            bg_grid,
            min_pixels: 5,
            max_pixels: 10000,
            max_centroids: None,
            max_elongation: Some(3.0),
            ..Default::default()
        };
        let (ms, n) = time_ms(50, || {
            tetra3::extract_centroids_fast(&px, w, h, &cfg).unwrap().centroids.len()
        });
        println!("fast, bg_grid={bg_grid:<3}        {ms:6.2} ms   ({n} stars)");
    }

    let ccl = tetra3::CentroidExtractionConfig {
        sigma_threshold: sigma,
        min_pixels: 5,
        max_pixels: 10000,
        max_centroids: None,
        local_bg_block_size: Some(32),
        max_elongation: Some(3.0),
        matched_filter_sigma: Some(0.6),
        ..Default::default()
    };
    let (ms, n) = time_ms(20, || {
        tetra3::extract_centroids_from_raw(&px, w, h, &ccl).unwrap().centroids.len()
    });
    println!("ccl + blur (reference)  {ms:6.2} ms   ({n} stars)");

    Ok(())
}
