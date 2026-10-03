//! Focuser port on ToupTek astro cameras, reached through the open camera's
//! command channel rather than a device of its own.

use crossbeam_channel::Sender;

use super::{FocuserDriver, FocuserTelemetry};
use crate::camera::toupcam::{FocuserState, ToupCmd};

/// [`FocuserDriver`](FocuserDriver) for the focuser port on
/// a ToupTek astro camera. Commands ride the camera's own command channel;
/// position and motion arrive with the camera telemetry, which the app
/// folds in with [`Focuser::apply`](super::Focuser::apply), so
/// `poll` here has nothing to return. Closes with the camera.
pub struct ToupFocuserDriver {
    cmd_tx: Sender<ToupCmd>,
    max_step: i32,
}

impl ToupFocuserDriver {
    pub fn new(cmd_tx: Sender<ToupCmd>, state: &FocuserState) -> (Self, FocuserTelemetry) {
        let initial = FocuserTelemetry {
            position: state.position,
            moving: state.moving,
            ..Default::default()
        };
        (ToupFocuserDriver { cmd_tx, max_step: state.max_step }, initial)
    }
}

impl FocuserDriver for ToupFocuserDriver {
    fn name(&self) -> &str {
        "Camera focuser"
    }

    fn max_step(&self) -> i32 {
        self.max_step
    }

    fn move_to(&mut self, pos: i32) {
        let _ = self.cmd_tx.send(ToupCmd::SetFocuserPosition(pos));
    }

    fn halt(&mut self) {
        let _ = self.cmd_tx.send(ToupCmd::FocuserHalt);
    }

    fn poll(&mut self) -> Option<FocuserTelemetry> {
        None
    }

    fn is_camera_bound(&self) -> bool {
        true
    }
}
