//! The app's one focuser, independent of which camera is open.
//!
//! A focuser is either its own device (ZWO EAF over USB) or a port on the
//! camera (ToupTek astro models), and later perhaps an INDI driver. Each
//! backend is a submodule implementing [`FocuserDriver`]; the rest of the app only sees
//! [`Focuser`]: a position, whether it is moving, an optional temperature,
//! and `move_to` / `halt`. The Focus tab tags every measurement with the
//! position from here and the autofocus routine drives it through here, so
//! neither knows which backend is behind it.

#[cfg(feature = "eaf")]
pub mod eaf;
#[cfg(feature = "toupcam")]
pub mod toupcam;

/// A backend's view of the focuser at one instant.
#[derive(Clone, Copy, Debug, Default)]
pub struct FocuserTelemetry {
    pub position: i32,
    pub moving: bool,
    /// Motion started from a hand controller, which `halt` cannot stop.
    pub hand_control: bool,
    /// `None` when the backend has no sensor, or the reading is not fresh.
    pub temperature_c: Option<f32>,
}

/// One focuser backend. Implementations are owned by the UI thread and must
/// not block: hardware I/O belongs on a backend thread, with commands and
/// telemetry crossing over channels.
pub trait FocuserDriver: Send {
    /// Display name, e.g. `"EAF"` or `"Camera focuser"`.
    fn name(&self) -> &str;

    /// Inclusive upper limit of the step position; the lower limit is zero.
    fn max_step(&self) -> i32;

    /// Start moving to an absolute step position. Must return immediately.
    fn move_to(&mut self, pos: i32);

    /// Stop any motion. Must return immediately.
    fn halt(&mut self);

    /// Drain pending telemetry, returning the newest snapshot if any
    /// arrived. Backends whose state arrives by another route (a camera's
    /// telemetry stream) return `None` here and the app calls
    /// [`Focuser::apply`] instead.
    fn poll(&mut self) -> Option<FocuserTelemetry>;

    /// True when the focuser lives on the camera and goes away with it.
    fn is_camera_bound(&self) -> bool {
        false
    }

    /// Shut the backend down and wait for its thread, so the SDK is closed
    /// cleanly before the driver is dropped.
    fn stop(&mut self) {}
}

pub struct Focuser {
    driver: Box<dyn FocuserDriver>,
    pub position: i32,
    pub moving: bool,
    pub hand_control: bool,
    pub temperature_c: Option<f32>,
    /// Side-panel "Target" field.
    pub target: i32,
    /// Side-panel jog step, steps.
    pub jog: i32,
    /// Position of the last `move_to`, so a completed move can be recognised.
    pub last_target: Option<i32>,
}

impl Focuser {
    pub fn new(driver: Box<dyn FocuserDriver>, initial: FocuserTelemetry) -> Self {
        Focuser {
            driver,
            position: initial.position,
            moving: initial.moving,
            hand_control: initial.hand_control,
            temperature_c: initial.temperature_c,
            target: initial.position,
            jog: 100,
            last_target: None,
        }
    }

    pub fn name(&self) -> &str {
        self.driver.name()
    }

    pub fn max_step(&self) -> i32 {
        self.driver.max_step().max(1)
    }

    pub fn is_camera_bound(&self) -> bool {
        self.driver.is_camera_bound()
    }

    pub fn move_to(&mut self, pos: i32) {
        let pos = pos.clamp(0, self.max_step());
        self.last_target = Some(pos);
        // Optimistic: the next telemetry confirms, but until then neither the
        // UI nor the autofocus routine should mistake the old position for
        // "arrived".
        self.moving = true;
        self.driver.move_to(pos);
    }

    pub fn halt(&mut self) {
        self.last_target = None;
        self.driver.halt();
    }

    /// Fold a telemetry snapshot into the state. A missing temperature
    /// keeps the last reading rather than blanking it.
    pub fn apply(&mut self, t: FocuserTelemetry) {
        self.position = t.position;
        self.moving = t.moving;
        self.hand_control = t.hand_control;
        if t.temperature_c.is_some() {
            self.temperature_c = t.temperature_c;
        }
    }

    /// Pull whatever the backend has reported since the last call.
    pub fn poll(&mut self) {
        if let Some(t) = self.driver.poll() {
            self.apply(t);
        }
    }

    pub fn stop(&mut self) {
        self.driver.stop();
    }
}
