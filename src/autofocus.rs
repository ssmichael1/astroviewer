//! V-curve autofocus: step the focuser through a range of positions around
//! the start, measure HFR at each, fit the curve, move to its minimum.
//!
//! The routine is a state machine with no clock, no thread and no hardware
//! access. The app feeds it two kinds of events: focuser telemetry (position,
//! moving) and focus samples (the HFR the worker measured on a frame, with
//! the focuser state at the time that frame was dispatched). It answers with
//! [`Action`]s the app carries out: move the focuser, halt it, log a line.
//! Keeping it pure makes the whole sweep testable against a synthetic star.
//!
//! # Method
//!
//! HFR against focuser position is a hyperbola: linear in `|p - p0|` far
//! from focus, rounding off at the minimum where the star reaches its
//! seeing- or optics-limited size. Squaring it makes it a parabola,
//! `hfr² = m²·(p - p0)² + a²`, which fits by plain linear least squares, so
//! the vertex comes out in closed form with no iteration to go wrong.
//!
//! Every measured position is approached from the same side (below): the
//! sweep first overshoots under its lowest point by `overshoot` steps, then
//! walks upward, and the final move to the fitted minimum overshoots under
//! it and comes back up the same way. Mechanical backlash therefore cancels
//! out of the measurement rather than having to be known.
//!
//! After each move the first `settle_frames` frames are discarded: a frame
//! that arrives just after the motor stops was at least partly exposed while
//! it was still turning.
//!
//! # Starting far from focus
//!
//! A sweep only works when it brackets the minimum. When the lowest HFR
//! lands on an end of the sweep the routine does not stop there: it keeps
//! walking in that direction with a step that doubles each time
//! (`max_extensions` batches of `points_per_side` points), and once the HFR
//! turns back up it discards the coarse points and runs a fresh sweep at the
//! original step around the turn. Only when it cannot extend further (end of
//! travel, or the extension limit) does it settle for the best measured
//! point, and then it reports [`Outcome::Unbracketed`] rather than a focus.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutofocusConfig {
    /// Focuser steps between sweep points.
    pub step: i32,
    /// Sweep points each side of the start position; the sweep has
    /// `2 * points_per_side + 1` points.
    pub points_per_side: usize,
    /// HFR measurements averaged (median) at each point.
    pub frames_per_point: usize,
    /// Frames discarded after each move before measuring.
    pub settle_frames: usize,
    /// Backlash margin: each approach starts this many steps below its
    /// target and moves up.
    pub overshoot: i32,
    /// Frames without a measurable star tolerated at one point before that
    /// point is skipped.
    pub max_misses: usize,
    /// Give up on a move that has not been reported complete within this
    /// long.
    pub move_timeout: Duration,
    /// How many times the sweep may be extended when the HFR is still
    /// falling at one end; each extension doubles the step.
    pub max_extensions: usize,
}

impl Default for AutofocusConfig {
    fn default() -> Self {
        AutofocusConfig {
            step: 250,
            points_per_side: 4,
            frames_per_point: 3,
            settle_frames: 2,
            overshoot: 500,
            max_misses: 6,
            move_timeout: Duration::from_secs(60),
            max_extensions: 5,
        }
    }
}

/// One measured sweep point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AfPoint {
    pub position: i32,
    /// Median HFR over the frames measured here, pixels.
    pub hfr: f32,
    pub frames: usize,
}

/// The fitted curve `hfr(p) = sqrt(a² + m²·(p - vertex)²)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HyperbolaFit {
    pub vertex: f64,
    /// HFR at the vertex, pixels.
    pub hfr_min: f64,
    /// Slope of the asymptotes, HFR pixels per focuser step.
    pub slope: f64,
}

impl HyperbolaFit {
    pub fn eval(&self, p: f64) -> f64 {
        let d = p - self.vertex;
        (self.hfr_min * self.hfr_min + self.slope * self.slope * d * d).sqrt()
    }
}

/// Least-squares hyperbola through the points, or `None` when there are too
/// few points, they do not curve upward, or the minimum is not bracketed by
/// the measurements (an extrapolated vertex is not to be trusted).
pub fn fit_hyperbola(points: &[AfPoint]) -> Option<HyperbolaFit> {
    if points.len() < 4 {
        return None;
    }
    // Centre and scale the abscissa so the normal equations are well
    // conditioned at positions in the tens of thousands.
    let n = points.len() as f64;
    let x_mean = points.iter().map(|p| p.position as f64).sum::<f64>() / n;
    let x_scale = points
        .iter()
        .map(|p| (p.position as f64 - x_mean).abs())
        .fold(0.0_f64, f64::max)
        .max(1.0);
    // Normal equations for y² = A u² + B u + C, u = (x - x_mean) / x_scale.
    let (mut s0, mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let (mut t0, mut t1, mut t2) = (0.0, 0.0, 0.0);
    for p in points {
        let u = (p.position as f64 - x_mean) / x_scale;
        let y2 = (p.hfr as f64) * (p.hfr as f64);
        let u2 = u * u;
        s0 += 1.0;
        s1 += u;
        s2 += u2;
        s3 += u2 * u;
        s4 += u2 * u2;
        t0 += y2;
        t1 += y2 * u;
        t2 += y2 * u2;
    }
    let m = [[s4, s3, s2], [s3, s2, s1], [s2, s1, s0]];
    let rhs = [t2, t1, t0];
    let [a, b, c] = solve3(m, rhs)?;
    if !(a > 0.0) || !a.is_finite() || !b.is_finite() || !c.is_finite() {
        return None;
    }
    let u0 = -b / (2.0 * a);
    let vertex = x_mean + u0 * x_scale;
    let lo = points.iter().map(|p| p.position).min()? as f64;
    let hi = points.iter().map(|p| p.position).max()? as f64;
    if vertex < lo || vertex > hi {
        return None;
    }
    let y2_min = c - b * b / (4.0 * a);
    // A vertex below zero HFR² means the points are noisier than the curve
    // is deep; the minimum is still where the parabola says, so clamp.
    let hfr_min = y2_min.max(0.0).sqrt();
    let slope = a.sqrt() / x_scale;
    Some(HyperbolaFit { vertex, hfr_min, slope })
}

/// Gaussian elimination with partial pivoting on a 3×3 system.
fn solve3(mut m: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
    for col in 0..3 {
        let pivot = (col..3).max_by(|&i, &j| m[i][col].abs().partial_cmp(&m[j][col].abs()).unwrap())?;
        if m[pivot][col].abs() < 1e-12 {
            return None;
        }
        m.swap(col, pivot);
        b.swap(col, pivot);
        for row in (col + 1)..3 {
            let f = m[row][col] / m[col][col];
            for k in col..3 {
                m[row][k] -= f * m[col][k];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for row in (0..3).rev() {
        let mut acc = b[row];
        for k in (row + 1)..3 {
            acc -= m[row][k] * x[k];
        }
        x[row] = acc / m[row][row];
    }
    Some(x)
}

fn median(v: &mut [f32]) -> f32 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = v.len();
    if n == 0 {
        f32::NAN
    } else if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// Which way along the focuser's travel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    /// Toward higher step positions.
    Up,
    /// Toward lower step positions.
    Down,
}

impl Direction {
    fn word(self) -> &'static str {
        match self {
            Direction::Up => "above",
            Direction::Down => "below",
        }
    }
}

/// How the final position was chosen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    /// The fitted hyperbola's vertex.
    Fit,
    /// The fit was not usable; the lowest measured point instead.
    BestPoint,
    /// The HFR was still falling at this end of the sweep and the sweep
    /// could not be extended; the lowest measured point, which is not focus.
    BeyondSweep(Direction),
}

#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Focused {
        position: i32,
        method: Method,
        /// HFR predicted at `position` by the fit, or measured there for `BestPoint`.
        hfr_expected: f32,
        /// HFR measured after the final move, when the verification frames
        /// found a star.
        hfr_verified: Option<f32>,
    },
    /// The HFR was still falling at an end of the sweep and the sweep could
    /// not be extended any further (end of travel, or the extension limit).
    /// The focuser was moved to the best measured point, but focus lies
    /// beyond it in `direction`.
    Unbracketed {
        position: i32,
        direction: Direction,
        hfr_measured: f32,
        hfr_verified: Option<f32>,
    },
    /// Nothing usable was measured; the focuser was sent back to where it started.
    Failed(String),
    Aborted,
}

/// Something the app must do on the routine's behalf.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    MoveTo(i32),
    Halt,
    Log(String),
    /// The run is over; `outcome()` has the result.
    Finished,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Measure {
    /// Approach-only move, nothing measured.
    None,
    Sweep,
    Verify,
}

#[derive(Clone, Copy, Debug)]
struct Stage {
    target: i32,
    measure: Measure,
}

#[derive(Debug)]
enum Phase {
    /// Waiting for the focuser to report arrival at `stage.target`.
    Moving { stage: Stage, since: Instant },
    Settling { stage: Stage, dropped: usize },
    Sampling { stage: Stage, hfrs: Vec<f32>, misses: usize },
    Done,
}

pub struct Autofocus {
    cfg: AutofocusConfig,
    start_pos: i32,
    max_step: i32,
    plan: VecDeque<Stage>,
    phase: Phase,
    /// Whether the plan being executed is the sweep or the final approach.
    finalizing: bool,
    /// Sweep extensions made so far (see `extend`).
    extensions: usize,
    /// A fine sweep around the coarse walk's turning point has been run.
    refined: bool,
    points: Vec<AfPoint>,
    total_sweep_points: usize,
    fit: Option<HyperbolaFit>,
    chosen: Option<(i32, Method, f32)>,
    outcome: Option<Outcome>,
    status: String,
}

impl Autofocus {
    /// Plan a run around `start_pos`. Fails when the sweep would leave
    /// `0..=max_step`, so a badly placed start is caught before anything moves.
    pub fn start(cfg: AutofocusConfig, start_pos: i32, max_step: i32) -> Result<Self, String> {
        if cfg.step <= 0 {
            return Err("step must be positive".into());
        }
        if cfg.points_per_side == 0 {
            return Err("need at least one point each side".into());
        }
        if cfg.frames_per_point == 0 {
            return Err("need at least one frame per point".into());
        }
        let n = cfg.points_per_side as i32;
        let lo = start_pos - n * cfg.step;
        let hi = start_pos + n * cfg.step;
        if lo < 0 || hi > max_step {
            return Err(format!(
                "sweep {lo}..{hi} leaves the focuser range 0..{max_step}; reduce the step or points, or move the start"
            ));
        }
        let mut plan = VecDeque::new();
        plan.push_back(Stage { target: (lo - cfg.overshoot).max(0), measure: Measure::None });
        for i in 0..=(2 * n) {
            plan.push_back(Stage { target: lo + i * cfg.step, measure: Measure::Sweep });
        }
        let total_sweep_points = 2 * cfg.points_per_side + 1;
        Ok(Autofocus {
            cfg,
            start_pos,
            max_step,
            plan,
            phase: Phase::Done,
            finalizing: false,
            extensions: 0,
            refined: false,
            points: Vec::with_capacity(total_sweep_points),
            total_sweep_points,
            fit: None,
            chosen: None,
            outcome: None,
            status: format!("Sweeping {lo}..{hi}"),
        })
    }

    /// Kick off the first move. Separate from `start` so the caller can
    /// hold the routine before it acts.
    pub fn begin(&mut self) -> Vec<Action> {
        let mut actions = vec![Action::Log(format!(
            "Autofocus: {} points, step {}, {} frames each, from {}",
            self.total_sweep_points, self.cfg.step, self.cfg.frames_per_point, self.start_pos
        ))];
        self.advance(&mut actions);
        actions
    }

    pub fn is_running(&self) -> bool {
        !matches!(self.phase, Phase::Done)
    }

    pub fn outcome(&self) -> Option<&Outcome> {
        self.outcome.as_ref()
    }

    pub fn status(&self) -> &str {
        &self.status
    }

    pub fn points(&self) -> &[AfPoint] {
        &self.points
    }

    pub fn fit(&self) -> Option<&HyperbolaFit> {
        self.fit.as_ref()
    }

    /// `(measured, planned)` sweep points.
    pub fn progress(&self) -> (usize, usize) {
        (self.points.len(), self.total_sweep_points)
    }

    /// True while measuring sweep points, as opposed to the final approach
    /// and verification.
    pub fn sweeping(&self) -> bool {
        self.is_running() && !self.finalizing
    }

    /// The position currently being measured or moved to, if any.
    pub fn current_target(&self) -> Option<i32> {
        match &self.phase {
            Phase::Moving { stage, .. } | Phase::Settling { stage, .. } | Phase::Sampling { stage, .. } => Some(stage.target),
            Phase::Done => None,
        }
    }

    /// Focuser state changed.
    pub fn on_telemetry(&mut self, position: i32, moving: bool) -> Vec<Action> {
        let mut actions = Vec::new();
        if let Phase::Moving { stage, since } = &self.phase {
            let (stage, since) = (*stage, *since);
            if !moving && position == stage.target {
                self.phase = match stage.measure {
                    Measure::None => {
                        // Approach-only stage: go straight on to the next.
                        Phase::Done
                    }
                    _ => Phase::Settling { stage, dropped: 0 },
                };
                if matches!(self.phase, Phase::Done) {
                    self.advance(&mut actions);
                }
            } else if since.elapsed() > self.cfg.move_timeout {
                self.fail(format!("focuser did not reach {} (at {position}, moving={moving})", stage.target), &mut actions);
            }
        }
        actions
    }

    /// A frame's focus measurement arrived. `position`/`moving` are the
    /// focuser state when the frame was handed to the worker; `hfr` is
    /// `None` when no star could be measured.
    pub fn on_sample(&mut self, position: Option<i32>, moving: bool, hfr: Option<f32>) -> Vec<Action> {
        let mut actions = Vec::new();
        match &mut self.phase {
            Phase::Settling { stage, dropped } => {
                if moving || position.is_some_and(|p| p != stage.target) {
                    return actions;
                }
                *dropped += 1;
                if *dropped >= self.cfg.settle_frames {
                    let stage = *stage;
                    self.phase = Phase::Sampling { stage, hfrs: Vec::new(), misses: 0 };
                    self.status = match stage.measure {
                        Measure::Verify => format!("Verifying at {}", stage.target),
                        _ => format!("Measuring at {} ({} of {})", stage.target, self.points.len() + 1, self.total_sweep_points),
                    };
                }
            }
            Phase::Sampling { stage, hfrs, misses } => {
                if moving || position.is_some_and(|p| p != stage.target) {
                    return actions;
                }
                match hfr {
                    Some(h) if h.is_finite() && h > 0.0 => hfrs.push(h),
                    _ => *misses += 1,
                }
                let stage = *stage;
                if hfrs.len() >= self.cfg.frames_per_point {
                    let h = median(hfrs);
                    let frames = hfrs.len();
                    match stage.measure {
                        Measure::Sweep => {
                            self.points.push(AfPoint { position: stage.target, hfr: h, frames });
                            actions.push(Action::Log(format!("Autofocus: HFR {h:.2} at {}", stage.target)));
                            self.advance(&mut actions);
                        }
                        Measure::Verify => self.finish_verified(Some(h), &mut actions),
                        Measure::None => self.advance(&mut actions),
                    }
                } else if *misses > self.cfg.max_misses {
                    actions.push(Action::Log(format!("Autofocus: no measurable star at {}, skipping", stage.target)));
                    if stage.measure == Measure::Verify {
                        self.finish_verified(None, &mut actions);
                    } else {
                        self.advance(&mut actions);
                    }
                }
            }
            Phase::Moving { .. } | Phase::Done => {}
        }
        actions
    }

    /// Stop everything and send the focuser back to where the run began.
    pub fn abort(&mut self) -> Vec<Action> {
        if !self.is_running() {
            return Vec::new();
        }
        self.phase = Phase::Done;
        self.plan.clear();
        self.outcome = Some(Outcome::Aborted);
        self.status = "Aborted".into();
        vec![
            Action::Halt,
            Action::MoveTo(self.start_pos),
            Action::Log(format!("Autofocus aborted; returning to {}", self.start_pos)),
            Action::Finished,
        ]
    }

    /// Move on to the next stage of the plan, or to the next plan.
    fn advance(&mut self, actions: &mut Vec<Action>) {
        if let Some(stage) = self.plan.pop_front() {
            self.phase = Phase::Moving { stage, since: Instant::now() };
            if stage.measure != Measure::Verify {
                self.status = format!("Moving to {}", stage.target);
            }
            actions.push(Action::MoveTo(stage.target));
            return;
        }
        if self.finalizing {
            // Verify stage never ran (shouldn't happen; plan always ends in one).
            self.finish_verified(None, actions);
            return;
        }
        self.sweep_done(actions);
    }

    /// A batch of sweep points is measured. Decide whether the minimum is
    /// bracketed, and if not, walk further; if a coarse walk has just
    /// bracketed it, sweep finely around the turn; otherwise fit.
    fn sweep_done(&mut self, actions: &mut Vec<Action>) {
        let Some(best) = self.best_point() else {
            self.fail(format!("none of {} points measured", self.total_sweep_points), actions);
            return;
        };
        if let Some(dir) = self.min_at_end(best) {
            if self.extensions < self.cfg.max_extensions && self.extend(dir, actions) {
                return;
            }
            actions.push(Action::Log(format!(
                "Autofocus: HFR still falling at the {} end of the sweep and cannot extend further; \
                 focus is {} {}. Moving to the best measured point.",
                match dir { Direction::Up => "top", Direction::Down => "bottom" },
                dir.word(),
                best.position
            )));
            self.plan_approach(best.position, Method::BeyondSweep(dir), best.hfr, actions);
            return;
        }
        if self.extensions > 0 && !self.refined && self.refine(best.position, actions) {
            return;
        }
        self.plan_final(actions);
    }

    fn best_point(&self) -> Option<AfPoint> {
        self.points
            .iter()
            .min_by(|a, b| a.hfr.partial_cmp(&b.hfr).unwrap_or(std::cmp::Ordering::Equal))
            .copied()
    }

    /// The end of the sweep the lowest point sits on, if it does: the HFR
    /// was still falling there, so focus lies beyond it.
    fn min_at_end(&self, best: AfPoint) -> Option<Direction> {
        if self.points.len() < 2 {
            return None;
        }
        let lo = self.points.iter().map(|p| p.position).min()?;
        let hi = self.points.iter().map(|p| p.position).max()?;
        if best.position == hi {
            Some(Direction::Up)
        } else if best.position == lo {
            Some(Direction::Down)
        } else {
            None
        }
    }

    /// Add `points_per_side` sweep points beyond the `dir` end, at a step
    /// that doubles with each extension. Downward extensions dip under
    /// their lowest point first so every point is still approached from
    /// below. Returns false when no point fits in the focuser's range.
    fn extend(&mut self, dir: Direction, actions: &mut Vec<Action>) -> bool {
        let n = self.cfg.points_per_side as i32;
        let step = self.cfg.step.saturating_mul(1i32 << (self.extensions + 1).min(20));
        let lo = self.points.iter().map(|p| p.position).min().unwrap_or(self.start_pos);
        let hi = self.points.iter().map(|p| p.position).max().unwrap_or(self.start_pos);
        // Ascending order, so the walk is always upward.
        let targets: Vec<i32> = match dir {
            Direction::Up => (1..=n).map(|i| hi.saturating_add(i * step)).filter(|p| *p <= self.max_step).collect(),
            Direction::Down => (1..=n).rev().map(|i| lo.saturating_sub(i * step)).filter(|p| *p >= 0).collect(),
        };
        if targets.is_empty() {
            return false;
        }
        self.extensions += 1;
        if dir == Direction::Down {
            self.plan.push_back(Stage { target: (targets[0] - self.cfg.overshoot).max(0), measure: Measure::None });
        }
        for &t in &targets {
            self.plan.push_back(Stage { target: t, measure: Measure::Sweep });
        }
        self.total_sweep_points += targets.len();
        let word = match dir { Direction::Up => "up", Direction::Down => "down" };
        actions.push(Action::Log(format!(
            "Autofocus: HFR still falling at {}; extending the sweep {word} to {} at step {step}",
            match dir { Direction::Up => hi, Direction::Down => lo },
            match dir { Direction::Up => targets[targets.len() - 1], Direction::Down => targets[0] },
        )));
        self.status = format!("Extending sweep {word} to {}", match dir { Direction::Up => targets[targets.len() - 1], Direction::Down => targets[0] });
        self.advance(actions);
        true
    }

    /// The coarse walk has bracketed the minimum near `center`: drop the
    /// coarse points and sweep at the configured step around it. Returns
    /// false when the range is too short for a full sweep.
    fn refine(&mut self, center: i32, actions: &mut Vec<Action>) -> bool {
        self.refined = true;
        let half = self.cfg.points_per_side as i32 * self.cfg.step;
        if self.max_step < 2 * half {
            return false;
        }
        let c = center.clamp(half, self.max_step - half);
        let lo = c - half;
        let n = 2 * self.cfg.points_per_side as i32;
        self.points.clear();
        self.total_sweep_points = n as usize + 1;
        self.plan.push_back(Stage { target: (lo - self.cfg.overshoot).max(0), measure: Measure::None });
        for i in 0..=n {
            self.plan.push_back(Stage { target: lo + i * self.cfg.step, measure: Measure::Sweep });
        }
        actions.push(Action::Log(format!("Autofocus: minimum bracketed near {center}; sweeping {lo}..{} at step {}", lo + n * self.cfg.step, self.cfg.step)));
        self.status = format!("Refining around {c}");
        self.advance(actions);
        true
    }

    /// Plan the backlash-safe final approach to `target` and its verification.
    fn plan_approach(&mut self, target: i32, method: Method, expected: f32, actions: &mut Vec<Action>) {
        self.finalizing = true;
        let target = target.clamp(0, self.max_step);
        self.chosen = Some((target, method, expected));
        self.plan.push_back(Stage { target: (target - self.cfg.overshoot).max(0), measure: Measure::None });
        self.plan.push_back(Stage { target, measure: Measure::Verify });
        self.status = match method {
            Method::BeyondSweep(_) => format!("Moving to best point at {target}"),
            _ => format!("Moving to focus at {target}"),
        };
        self.advance(actions);
    }

    /// Sweep done and bracketed: fit, choose the target, plan the approach.
    fn plan_final(&mut self, actions: &mut Vec<Action>) {
        self.fit = fit_hyperbola(&self.points);
        let choice = match self.fit {
            Some(f) => {
                actions.push(Action::Log(format!(
                    "Autofocus: fit minimum at {:.0}, HFR {:.2}, slope {:.4} px/step",
                    f.vertex, f.hfr_min, f.slope
                )));
                Some((f.vertex.round() as i32, Method::Fit, f.hfr_min as f32))
            }
            None => {
                let best = self
                    .points
                    .iter()
                    .min_by(|a, b| a.hfr.partial_cmp(&b.hfr).unwrap_or(std::cmp::Ordering::Equal))
                    .copied();
                match best {
                    Some(b) if self.points.len() >= 3 => {
                        actions.push(Action::Log(format!(
                            "Autofocus: no usable fit ({} points); using best measured point {} (HFR {:.2})",
                            self.points.len(),
                            b.position,
                            b.hfr
                        )));
                        Some((b.position, Method::BestPoint, b.hfr))
                    }
                    _ => None,
                }
            }
        };
        let Some((target, method, expected)) = choice else {
            self.fail(format!("only {} of {} points measured", self.points.len(), self.total_sweep_points), actions);
            return;
        };
        self.plan_approach(target, method, expected, actions);
    }

    fn finish_verified(&mut self, hfr_verified: Option<f32>, actions: &mut Vec<Action>) {
        let (position, method, hfr_expected) = self.chosen.unwrap_or((self.start_pos, Method::BestPoint, f32::NAN));
        let verified = match hfr_verified {
            Some(h) => format!("HFR {h:.2}"),
            None => "not verified".to_string(),
        };
        self.outcome = Some(match method {
            Method::BeyondSweep(direction) => Outcome::Unbracketed { position, direction, hfr_measured: hfr_expected, hfr_verified },
            _ => Outcome::Focused { position, method, hfr_expected, hfr_verified },
        });
        self.status = match method {
            Method::BeyondSweep(dir) => format!("Not focused: focus is {} {position} ({verified}); run again from here", dir.word()),
            _ => format!("Focused at {position}: {verified}"),
        };
        actions.push(Action::Log(format!("Autofocus: {}", self.status)));
        self.phase = Phase::Done;
        self.plan.clear();
        actions.push(Action::Finished);
    }

    fn fail(&mut self, why: String, actions: &mut Vec<Action>) {
        self.phase = Phase::Done;
        self.plan.clear();
        self.status = format!("Failed: {why}");
        self.outcome = Some(Outcome::Failed(why.clone()));
        actions.push(Action::Halt);
        actions.push(Action::MoveTo(self.start_pos));
        actions.push(Action::Log(format!("Autofocus failed: {why}; returning to {}", self.start_pos)));
        actions.push(Action::Finished);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(position: i32, hfr: f32) -> AfPoint {
        AfPoint { position, hfr, frames: 3 }
    }

    #[test]
    fn hyperbola_fit_recovers_vertex() {
        let truth = HyperbolaFit { vertex: 45_120.0, hfr_min: 1.8, slope: 0.004 };
        let pts: Vec<AfPoint> = (-4..=4).map(|i| {
            let p = 45_000 + i * 250;
            pt(p, truth.eval(p as f64) as f32)
        }).collect();
        let f = fit_hyperbola(&pts).expect("fit");
        assert!((f.vertex - truth.vertex).abs() < 1.0, "vertex {}", f.vertex);
        assert!((f.hfr_min - truth.hfr_min).abs() < 0.01);
        assert!((f.slope - truth.slope).abs() < 1e-5);
    }

    #[test]
    fn fit_rejects_unbracketed_minimum() {
        // Monotonic run: the minimum is off the end of the sweep.
        let truth = HyperbolaFit { vertex: 50_000.0, hfr_min: 1.5, slope: 0.004 };
        let pts: Vec<AfPoint> = (0..9).map(|i| {
            let p = 44_000 + i * 250;
            pt(p, truth.eval(p as f64) as f32)
        }).collect();
        assert!(fit_hyperbola(&pts).is_none());
    }

    #[test]
    fn fit_needs_four_points() {
        assert!(fit_hyperbola(&[pt(0, 3.0), pt(100, 2.0), pt(200, 3.0)]).is_none());
    }

    /// Drive the state machine against a simulated focuser and star. The
    /// focuser arrives instantly; frames report the HFR of the hyperbola at
    /// the focuser's position with a little noise.
    fn run_sim(cfg: AutofocusConfig, start: i32, truth: &HyperbolaFit, max_step: i32) -> (Autofocus, i32, Vec<i32>) {
        let mut af = Autofocus::start(cfg, start, max_step).expect("start");
        let mut pos = start;
        let mut moves = Vec::new();
        let mut pending = af.begin();
        let mut seed = 12345_u32;
        let mut noise = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.06
        };
        let mut guard = 0;
        while af.is_running() || !pending.is_empty() {
            guard += 1;
            assert!(guard < 10_000, "runaway");
            let mut next = Vec::new();
            for a in pending.drain(..) {
                match a {
                    Action::MoveTo(p) => {
                        pos = p;
                        moves.push(p);
                        next.extend(af.on_telemetry(pos, false));
                    }
                    Action::Halt | Action::Log(_) | Action::Finished => {}
                }
            }
            if next.is_empty() {
                // A frame arrives.
                let hfr = truth.eval(pos as f64) as f32 + noise();
                next.extend(af.on_sample(Some(pos), false, Some(hfr)));
            }
            pending = next;
        }
        (af, pos, moves)
    }

    #[test]
    fn sweep_finds_focus() {
        let truth = HyperbolaFit { vertex: 45_180.0, hfr_min: 1.9, slope: 0.005 };
        let cfg = AutofocusConfig::default();
        let (af, pos, moves) = run_sim(cfg, 45_000, &truth, 60_000);
        match af.outcome() {
            Some(Outcome::Focused { position, method: Method::Fit, hfr_verified: Some(v), .. }) => {
                assert!((*position - 45_180).abs() <= 15, "landed at {position}");
                assert_eq!(pos, *position);
                assert!((*v - 1.9).abs() < 0.1);
            }
            other => panic!("unexpected outcome {other:?}"),
        }
        assert_eq!(af.points().len(), 9);
        // First move overshoots under the lowest sweep point; the final
        // approach overshoots under the chosen position and returns up.
        assert_eq!(moves[0], 45_000 - 4 * 250 - 500);
        assert_eq!(moves[1], 45_000 - 4 * 250);
        let n = moves.len();
        assert_eq!(moves[n - 2], moves[n - 1] - 500);
        // Every sweep position was approached from below (the last two moves
        // are the final approach, which dips before coming back up).
        for w in moves[..n - 2].windows(2) {
            assert!(w[1] > w[0], "moves {:?}", moves);
        }
    }

    /// Start 2000 steps under focus with a sweep that covers only ±400: the
    /// walk must extend upward, bracket the minimum, refine, and land on it.
    #[test]
    fn far_start_extends_and_refines() {
        let truth = HyperbolaFit { vertex: 48_873.0, hfr_min: 2.0, slope: 0.006 };
        let cfg = AutofocusConfig { step: 100, ..Default::default() };
        let (af, pos, moves) = run_sim(cfg, 46_898, &truth, 60_000);
        match af.outcome() {
            Some(Outcome::Focused { position, method: Method::Fit, hfr_verified: Some(v), .. }) => {
                assert!((*position - 48_873).abs() <= 15, "landed at {position}");
                assert_eq!(pos, *position);
                assert!((*v - 2.0).abs() < 0.1);
            }
            other => panic!("unexpected outcome {other:?}"),
        }
        assert!(af.extensions >= 1, "no extension happened");
        assert!(af.refined, "no fine sweep after the coarse walk");
        // The fine sweep is a fresh set of points at the configured step.
        assert_eq!(af.points().len(), 9);
        let ps: Vec<i32> = af.points().iter().map(|p| p.position).collect();
        assert!(ps.windows(2).all(|w| w[1] - w[0] == 100), "fine sweep {ps:?}");
        assert!(ps[0] <= 48_873 && 48_873 <= ps[8], "fine sweep {ps:?} does not bracket");
        // Every measured position was approached from below: the only
        // downward moves are overshoot dips, each followed by an upward move.
        for w in moves.windows(3) {
            if w[1] < w[0] {
                assert!(w[2] > w[1], "dip not followed by an upward move: {moves:?}");
            }
        }
        assert!(moves.last().unwrap() > &moves[moves.len() - 2], "final approach not from below");
    }

    /// Focus below the start: the walk must extend downward, dipping under
    /// each new batch so the points are still approached from below.
    #[test]
    fn far_start_extends_downward() {
        let truth = HyperbolaFit { vertex: 20_000.0, hfr_min: 2.0, slope: 0.006 };
        let cfg = AutofocusConfig { step: 100, ..Default::default() };
        let (af, pos, _) = run_sim(cfg, 22_000, &truth, 60_000);
        match af.outcome() {
            Some(Outcome::Focused { position, method: Method::Fit, .. }) => {
                assert!((*position - 20_000).abs() <= 15, "landed at {position}");
                assert_eq!(pos, *position);
            }
            other => panic!("unexpected outcome {other:?}"),
        }
    }

    /// With extensions disabled, a start far from focus ends at the top of
    /// the sweep and says so, rather than claiming to be focused.
    #[test]
    fn unbracketed_is_reported_not_claimed() {
        let truth = HyperbolaFit { vertex: 48_873.0, hfr_min: 2.0, slope: 0.006 };
        let cfg = AutofocusConfig { step: 100, max_extensions: 0, ..Default::default() };
        let (af, pos, _) = run_sim(cfg, 46_898, &truth, 60_000);
        match af.outcome() {
            Some(Outcome::Unbracketed { position, direction: Direction::Up, hfr_verified: Some(_), .. }) => {
                assert_eq!(*position, 46_898 + 400);
                assert_eq!(pos, *position);
            }
            other => panic!("unexpected outcome {other:?}"),
        }
        assert!(af.status().starts_with("Not focused"), "{}", af.status());
    }

    /// Focus beyond the end of travel: the extension is cut short by the
    /// range, and the result is reported as unbracketed.
    #[test]
    fn extension_stops_at_end_of_travel() {
        let truth = HyperbolaFit { vertex: 70_000.0, hfr_min: 2.0, slope: 0.006 };
        let cfg = AutofocusConfig { step: 100, ..Default::default() };
        let (af, pos, moves) = run_sim(cfg, 46_898, &truth, 50_000);
        assert!(moves.iter().all(|m| (0..=50_000).contains(m)), "{moves:?}");
        match af.outcome() {
            Some(Outcome::Unbracketed { position, direction: Direction::Up, .. }) => {
                assert_eq!(pos, *position);
                assert!(*position > 47_298, "never extended: {position}");
            }
            other => panic!("unexpected outcome {other:?}"),
        }
    }

    #[test]
    fn samples_taken_while_moving_are_ignored() {
        let cfg = AutofocusConfig { settle_frames: 1, frames_per_point: 1, ..Default::default() };
        let mut af = Autofocus::start(cfg, 10_000, 60_000).unwrap();
        let acts = af.begin();
        assert!(matches!(acts.last(), Some(Action::MoveTo(_))));
        // Arrive at the approach stage, then the first sweep point.
        let first = 10_000 - 4 * 250;
        let acts = af.on_telemetry(first - 500, false);
        assert_eq!(acts, vec![Action::MoveTo(first)]);
        assert!(af.on_telemetry(first, false).is_empty());
        // A frame from before arrival (wrong position) does not count as settled.
        assert!(af.on_sample(Some(first - 500), false, Some(3.0)).is_empty());
        assert!(af.on_sample(Some(first), true, Some(3.0)).is_empty());
        // Settle frame, then the measurement.
        assert!(af.on_sample(Some(first), false, Some(3.0)).is_empty());
        let acts = af.on_sample(Some(first), false, Some(3.1));
        assert!(acts.iter().any(|a| matches!(a, Action::MoveTo(p) if *p == first + 250)));
        assert_eq!(af.points().len(), 1);
    }

    #[test]
    fn abort_returns_to_start() {
        let mut af = Autofocus::start(AutofocusConfig::default(), 20_000, 60_000).unwrap();
        af.begin();
        let acts = af.abort();
        assert_eq!(acts[0], Action::Halt);
        assert_eq!(acts[1], Action::MoveTo(20_000));
        assert!(!af.is_running());
        assert_eq!(af.outcome(), Some(&Outcome::Aborted));
    }

    #[test]
    fn sweep_outside_range_is_refused() {
        assert!(Autofocus::start(AutofocusConfig::default(), 500, 60_000).is_err());
        assert!(Autofocus::start(AutofocusConfig::default(), 59_500, 60_000).is_err());
    }

    #[test]
    fn no_stars_anywhere_fails_and_returns() {
        let cfg = AutofocusConfig { settle_frames: 0, max_misses: 1, ..Default::default() };
        let mut af = Autofocus::start(cfg, 30_000, 60_000).unwrap();
        let mut pending = af.begin();
        let mut pos = 30_000;
        let mut last_moves = Vec::new();
        let mut guard = 0;
        while af.is_running() || !pending.is_empty() {
            guard += 1;
            assert!(guard < 1000);
            let mut next = Vec::new();
            for a in pending.drain(..) {
                if let Action::MoveTo(p) = a {
                    pos = p;
                    last_moves.push(p);
                    next.extend(af.on_telemetry(pos, false));
                }
            }
            if next.is_empty() {
                next.extend(af.on_sample(Some(pos), false, None));
            }
            pending = next;
        }
        assert!(matches!(af.outcome(), Some(Outcome::Failed(_))));
        assert_eq!(*last_moves.last().unwrap(), 30_000);
    }
}
