//! Dual-type frame pixel buffer.
//!
//! Integer camera sources (GigE / INDI Mono8/10/12/14/16) keep their pixels as
//! `u16` all the way through decode, statistics and colormapping; only the
//! consumers that genuinely need floating point (plate solving, the FITS
//! recorder's f32 path, background subtraction) pay for the widening. That
//! halves the per-frame memory traffic and buffer size versus storing every
//! integer frame as `f32`.

use std::borrow::Cow;
use std::sync::Arc;

/// A frame's mono pixel buffer, either native `u16` (integer sources) or `f32`
/// (float FITS and anything that has been background-subtracted, which can go
/// negative). Cheap to clone — both variants are `Arc`-shared.
#[derive(Clone)]
pub enum Pixels {
    // Constructed only by integer sources (GigE / INDI / native-mono camera
    // decode), which are behind feature flags; unused in a default build.
    #[allow(dead_code)]
    U16(Arc<Vec<u16>>),
    F32(Arc<Vec<f32>>),
}

impl Pixels {
    pub fn len(&self) -> usize {
        match self {
            Pixels::U16(v) => v.len(),
            Pixels::F32(v) => v.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// View the pixels as `f32`: borrows when already `F32`, allocates a widened
    /// copy when `U16`. Use for consumers that need a contiguous `&[f32]`.
    pub fn as_f32(&self) -> Cow<'_, [f32]> {
        match self {
            Pixels::F32(v) => Cow::Borrowed(v.as_slice()),
            Pixels::U16(v) => Cow::Owned(v.iter().map(|&x| x as f32).collect()),
        }
    }

    /// An owned `Arc<Vec<f32>>` for consumers that must own the pixels as f32
    /// across a thread boundary (the plate solver, the recorder's f32 path).
    /// Clones the `Arc` for free when already `F32`; allocates a widened copy
    /// for `U16`.
    #[allow(dead_code)] // only the starsolve/recorder paths need this
    pub fn to_f32_arc(&self) -> Arc<Vec<f32>> {
        match self {
            Pixels::F32(v) => Arc::clone(v),
            Pixels::U16(v) => Arc::new(v.iter().map(|&x| x as f32).collect()),
        }
    }

    /// Pixel value at `idx` as `f32`, or `None` if out of range.
    pub fn value_at(&self, idx: usize) -> Option<f32> {
        match self {
            Pixels::F32(v) => v.get(idx).copied(),
            Pixels::U16(v) => v.get(idx).map(|&x| x as f32),
        }
    }

    /// Convert to the `F32` variant in place if needed and return a mutable
    /// reference to the underlying `Vec<f32>` for in-place editing (background
    /// subtraction, whose result has negatives). Widens a `U16` buffer once;
    /// on an already-uniquely-owned `F32` buffer this is free.
    pub fn make_f32_mut(&mut self) -> &mut Vec<f32> {
        if let Pixels::U16(v) = self {
            let widened: Vec<f32> = v.iter().map(|&x| x as f32).collect();
            *self = Pixels::F32(Arc::new(widened));
        }
        match self {
            Pixels::F32(v) => Arc::make_mut(v),
            Pixels::U16(_) => unreachable!("just converted to F32"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn u16_and_f32_agree() {
        let raw: Vec<u16> = vec![0, 1, 255, 4095, 65535, 100];
        let u = Pixels::U16(Arc::new(raw.clone()));
        let f = Pixels::F32(Arc::new(raw.iter().map(|&x| x as f32).collect()));
        assert_eq!(u.len(), f.len());
        assert_eq!(u.as_f32(), f.as_f32());
        for i in 0..raw.len() + 2 {
            assert_eq!(u.value_at(i), f.value_at(i));
        }
    }

    #[test]
    fn make_f32_mut_widens_and_allows_negatives() {
        let mut p = Pixels::U16(Arc::new(vec![10, 20, 30]));
        {
            let v = p.make_f32_mut();
            for x in v.iter_mut() {
                *x -= 100.0;
            }
        }
        assert_eq!(p.as_f32().as_ref(), &[-90.0, -80.0, -70.0]);
    }
}
