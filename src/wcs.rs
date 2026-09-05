//! FITS WCS keywords from a live plate solve, so recorded frames open in
//! other software already on the sky.
//!
//! The solve gives a gnomonic (TAN) fit: a reference point on the sky
//! (CRVAL), the pixel it lands on (CRPIX), and a CD matrix mapping pixel
//! offsets from CRPIX to East/North tangent-plane offsets. That is exactly
//! the FITS `RA---TAN` / `DEC--TAN` description, so the conversion is a
//! change of origin and units:
//!
//! * tetra3 pixel coordinates are centered on the geometric image center
//!   `((W-1)/2, (H-1)/2)` in 0-based pixels, with +X right and +Y down. FITS
//!   pixel axes are 1-based, and axis 2 counts rows in file order, which the
//!   recorder writes top-first. So the same offset `(x, y)` is
//!   `(p1 - (W+1)/2, p2 - (H+1)/2)`, and CRPIX is that center plus the
//!   solve's optical-center offset.
//! * The CD matrix is in radians per pixel; FITS wants degrees.
//!
//! Lens distortion from a loaded camera model is not carried across (that
//! would need SIP terms); the linear TAN fit is exact at the reference point
//! and drifts by the distortion toward the corners.

/// The keyword values for one frame, computed on the UI thread from the last
/// solve so the recorder never touches solver types.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WcsKeys {
    /// 1-based FITS pixel coordinates of the reference point.
    pub crpix: [f64; 2],
    /// Reference point `[RA, Dec]`, degrees.
    pub crval_deg: [f64; 2],
    /// `[[CD1_1, CD1_2], [CD2_1, CD2_2]]`, degrees per pixel.
    pub cd_deg: [[f64; 2]; 2],
}

impl WcsKeys {
    /// Build from a solve. `None` when the solve was of a differently sized
    /// frame (a ROI or binning change since the last lock), since its CRPIX
    /// and scale would then describe a different image.
    #[cfg(feature = "starsolve")]
    pub fn from_solution(sol: &tetra3::Solution, width: u32, height: u32) -> Option<Self> {
        let cam = &sol.camera_model;
        if cam.image_width != width || cam.image_height != height {
            return None;
        }
        let crpix = [
            (width as f64 + 1.0) / 2.0 + cam.crpix[0],
            (height as f64 + 1.0) / 2.0 + cam.crpix[1],
        ];
        let crval_deg = [sol.crval_rad[0].to_degrees().rem_euclid(360.0), sol.crval_rad[1].to_degrees()];
        let cd_deg = sol.cd_matrix.map(|row| row.map(f64::to_degrees));
        if !crpix.iter().chain(crval_deg.iter()).chain(cd_deg.iter().flatten()).all(|v| v.is_finite()) {
            return None;
        }
        Some(WcsKeys { crpix, crval_deg, cd_deg })
    }

    /// Write the keywords into a FITS header.
    pub fn write(&self, header: &mut fitskit::Header) {
        use fitskit::HeaderValue::{Float, Integer, Logical, String as Str};
        header.set("WCSAXES", Integer(2), Some("number of WCS axes"));
        header.set("CTYPE1", Str("RA---TAN".into()), Some("gnomonic projection, RA"));
        header.set("CTYPE2", Str("DEC--TAN".into()), Some("gnomonic projection, Dec"));
        header.set("CUNIT1", Str("deg".into()), None);
        header.set("CUNIT2", Str("deg".into()), None);
        header.set("CRPIX1", Float(self.crpix[0]), Some("reference pixel, 1-based"));
        header.set("CRPIX2", Float(self.crpix[1]), Some("reference pixel, 1-based"));
        header.set("CRVAL1", Float(self.crval_deg[0]), Some("RA at reference pixel (deg)"));
        header.set("CRVAL2", Float(self.crval_deg[1]), Some("Dec at reference pixel (deg)"));
        header.set("CD1_1", Float(self.cd_deg[0][0]), Some("deg/pixel"));
        header.set("CD1_2", Float(self.cd_deg[0][1]), Some("deg/pixel"));
        header.set("CD2_1", Float(self.cd_deg[1][0]), Some("deg/pixel"));
        header.set("CD2_2", Float(self.cd_deg[1][1]), Some("deg/pixel"));
        header.set("RADESYS", Str("ICRS".into()), Some("celestial reference frame"));
        header.set("EQUINOX", Float(2000.0), Some("equinox of coordinates"));
        header.set("PLTSOLVD", Logical(true), Some("WCS from the live plate solve"));
    }

    /// Sky position of a 1-based FITS pixel through these keywords, degrees.
    /// The reader's side of the contract; used to check the writer.
    #[cfg(all(test, feature = "starsolve"))]
    pub fn pixel_to_world(&self, p1: f64, p2: f64) -> (f64, f64) {
        let dx = p1 - self.crpix[0];
        let dy = p2 - self.crpix[1];
        let xi = (self.cd_deg[0][0] * dx + self.cd_deg[0][1] * dy).to_radians();
        let eta = (self.cd_deg[1][0] * dx + self.cd_deg[1][1] * dy).to_radians();
        let (ra0, dec0) = (self.crval_deg[0].to_radians(), self.crval_deg[1].to_radians());
        // Inverse gnomonic projection about (ra0, dec0).
        let (sin_d0, cos_d0) = dec0.sin_cos();
        let denom = cos_d0 - eta * sin_d0;
        let ra = ra0 + (xi).atan2(denom);
        let dec = ((sin_d0 + eta * cos_d0) / (1.0 + xi * xi + eta * eta).sqrt()).asin();
        (ra.to_degrees().rem_euclid(360.0), dec.to_degrees())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writes_every_keyword() {
        let k = WcsKeys { crpix: [960.5, 540.5], crval_deg: [102.5, 72.4], cd_deg: [[-0.004, 0.001], [0.001, 0.004]] };
        let mut h = fitskit::Header::new();
        k.write(&mut h);
        for key in ["WCSAXES", "CTYPE1", "CTYPE2", "CRPIX1", "CRPIX2", "CRVAL1", "CRVAL2", "CD1_1", "CD1_2", "CD2_1", "CD2_2", "RADESYS", "EQUINOX", "PLTSOLVD"] {
            assert!(h.find(key).is_some(), "missing {key}");
        }
        assert_eq!(h.get_string("CTYPE1"), Some("RA---TAN"));
        assert_eq!(h.get_float("CRPIX1"), Some(960.5));
        assert_eq!(h.get_bool("PLTSOLVD"), Some(true));
    }

    /// The keywords must agree with the solver's own pixel-to-sky mapping at
    /// the reference pixel, the corners, and an off-center point, including
    /// a mirrored field and an off-center optical axis.
    #[cfg(feature = "starsolve")]
    #[test]
    fn round_trips_through_the_solver() {
        let (w, h) = (1920u32, 1080u32);
        for (parity, theta_deg, crpix) in [(false, 30.0_f64, [0.0_f64, 0.0]), (true, -75.0, [12.5, -8.0]), (false, 190.0, [0.0, 3.0])] {
            let sol = synthetic_solution(w, h, parity, theta_deg.to_radians(), crpix);
            let keys = WcsKeys::from_solution(&sol, w, h).expect("dimensions match");
            let center_x = (w as f64 - 1.0) / 2.0;
            let center_y = (h as f64 - 1.0) / 2.0;
            for (col, row) in [(center_x + crpix[0], center_y + crpix[1]), (0.0, 0.0), (1919.0, 0.0), (0.0, 1079.0), (1919.0, 1079.0), (300.25, 800.75)] {
                // Solver: centered 0-based pixels. FITS: 1-based.
                let (ra_s, dec_s) = sol.pixel_to_world(col - center_x, row - center_y);
                let (ra_k, dec_k) = keys.pixel_to_world(col + 1.0, row + 1.0);
                let d_ra = ((ra_s - ra_k + 180.0).rem_euclid(360.0) - 180.0) * dec_s.to_radians().cos();
                let d_dec = dec_s - dec_k;
                let sep_arcsec = (d_ra * d_ra + d_dec * d_dec).sqrt() * 3600.0;
                assert!(sep_arcsec < 0.01, "parity {parity} θ {theta_deg}: ({col},{row}) differs by {sep_arcsec:.4}\"");
            }
        }
        // A solve of a different frame size is refused.
        let sol = synthetic_solution(w, h, false, 0.0, [0.0, 0.0]);
        assert!(WcsKeys::from_solution(&sol, w / 2, h / 2).is_none());
    }

    /// A pinhole solution with the given roll, parity, and optical-center
    /// offset, built the way tetra3 builds its CD matrix from (θ, scale, parity).
    #[cfg(feature = "starsolve")]
    fn synthetic_solution(w: u32, h: u32, parity_flip: bool, theta: f64, crpix: [f64; 2]) -> tetra3::Solution {
        let fov_rad = 8.0_f64.to_radians();
        let mut camera_model = tetra3::CameraModel::from_fov(fov_rad, w, h);
        camera_model.parity_flip = parity_flip;
        camera_model.crpix = crpix;
        let ps = 1.0 / camera_model.focal_length_px;
        let (s, c) = theta.sin_cos();
        let cd_matrix = if parity_flip {
            [[-ps * c, -ps * s], [-ps * s, ps * c]]
        } else {
            [[ps * c, -ps * s], [ps * s, ps * c]]
        };
        tetra3::Solution {
            qicrs2cam: tetra3::Quaternion::identity(),
            fov_rad: fov_rad as f32,
            num_matches: 0,
            rmse_rad: 0.0,
            p90e_rad: 0.0,
            max_err_rad: 0.0,
            prob: 0.0,
            solve_time_ms: 0.0,
            attitude_cov_rad2: [[0.0; 3]; 3],
            parity_flip,
            observer_velocity_km_s: None,
            matched_catalog_ids: Vec::new(),
            matched_centroid_indices: Vec::new(),
            cd_matrix,
            crval_rad: [102.8736_f64.to_radians(), 72.462_f64.to_radians()],
            camera_model,
            theta_rad: theta,
        }
    }
}
