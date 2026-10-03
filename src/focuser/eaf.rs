//! ZWO EAF focuser backend: one thread per open focuser owning the SDK
//! handle, driven by commands over a channel and reporting position, motion
//! and temperature back as telemetry.
//!
//! The SDK handle is `Send` but not `Sync`, so the thread is the only place
//! it is touched. Polling is quick while a move is in progress (an autofocus
//! run wants to know the moment the motor stops) and relaxed when idle.
//!
//! Nothing here may call the SDK on the main thread. On macOS the SDK pumps
//! the current thread's run loop while it scans for HID devices, and on the
//! main thread that re-enters winit's event handler and aborts the process.
//! Scanning therefore runs on a throwaway thread and opening happens inside
//! the focuser thread itself.

use std::thread;
use std::time::{Duration, Instant};

use crossbeam_channel::{bounded, Receiver, RecvTimeoutError, Sender};

use super::{FocuserDriver, FocuserTelemetry};
use crate::LogEntry;

pub use zwo_eaf::FocuserInfo;

/// Scan for attached EAF focusers on a background thread. The list arrives
/// on the returned channel; empty when none is plugged in or the SDK cannot
/// be reached. If the thread cannot be spawned the sender is dropped and the
/// receiver reports Disconnected, which callers treat as an empty scan.
pub fn scan() -> Receiver<Vec<FocuserInfo>> {
    let (tx, rx) = bounded(1);
    let _ = thread::Builder::new().name("eaf-scan".into()).spawn(move || {
        let _ = tx.send(zwo_eaf::connected_focusers().unwrap_or_default());
    });
    rx
}

pub enum EafCmd {
    /// Move to an absolute step position (clamped to the focuser's range).
    MoveTo(i32),
    Halt,
    /// Close the focuser and end the thread.
    Stop,
}

/// Snapshot pushed by the focuser thread whenever something changes, and at
/// least once a second so a stale reading is never mistaken for a fresh one.
type EafTelemetry = FocuserTelemetry;

/// [`FocuserDriver`] for one open EAF: the UI-thread end of the channels
/// to its thread.
pub struct EafDriver {
    name: String,
    /// User-configured travel limit (the SDK's `max_step`), not the hardware range.
    max_step: i32,
    cmd_tx: Sender<EafCmd>,
    telemetry_rx: Receiver<EafTelemetry>,
    join_handle: Option<thread::JoinHandle<()>>,
}

impl FocuserDriver for EafDriver {
    fn name(&self) -> &str {
        &self.name
    }

    fn max_step(&self) -> i32 {
        self.max_step
    }

    fn move_to(&mut self, pos: i32) {
        let _ = self.cmd_tx.send(EafCmd::MoveTo(pos));
    }

    fn halt(&mut self) {
        let _ = self.cmd_tx.send(EafCmd::Halt);
    }

    fn poll(&mut self) -> Option<FocuserTelemetry> {
        let mut latest = None;
        while let Ok(t) = self.telemetry_rx.try_recv() {
            latest = Some(t);
        }
        latest
    }

    /// Close the focuser and wait for its thread, so the SDK is shut down
    /// cleanly before the driver is dropped.
    fn stop(&mut self) {
        let _ = self.cmd_tx.send(EafCmd::Stop);
        if let Some(jh) = self.join_handle.take() {
            let _ = jh.join();
        }
    }
}

impl Drop for EafDriver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Start the focuser thread, which opens the device, and wait for it to
/// report ready. Returns the driver plus the initial state so the UI can
/// show a value before the first telemetry. Blocks the caller for the open
/// (well under a second) and fails if the device does not answer.
pub fn start(info: &FocuserInfo, log_tx: Sender<LogEntry>) -> anyhow::Result<(EafDriver, FocuserTelemetry)> {
    let (cmd_tx, cmd_rx) = bounded::<EafCmd>(16);
    let (telemetry_tx, telemetry_rx) = bounded::<EafTelemetry>(8);
    let (ready_tx, ready_rx) = bounded::<Result<(i32, FocuserTelemetry), String>>(1);
    let name = if info.name.is_empty() { "EAF".to_string() } else { info.name.clone() };
    let thread_name = name.clone();
    let id = info.id;
    let fallback_max = info.max_step;
    let join_handle = thread::Builder::new().name("eaf-focuser".into()).spawn(move || {
        let eaf = match zwo_eaf::Focuser::open(id) {
            Ok(eaf) => eaf,
            Err(e) => {
                let _ = ready_tx.send(Err(e.to_string()));
                return;
            }
        };
        let max_step = eaf.max_step().unwrap_or(fallback_max).max(1);
        let initial = read_telemetry(&eaf, true);
        if ready_tx.send(Ok((max_step, initial))).is_err() {
            return;
        }
        focuser_loop(eaf, thread_name, max_step, cmd_rx, telemetry_tx, log_tx);
    })?;

    let (max_step, initial) = match ready_rx.recv_timeout(Duration::from_secs(10)) {
        Ok(Ok(v)) => v,
        Ok(Err(e)) => {
            let _ = join_handle.join();
            anyhow::bail!("{e}");
        }
        Err(_) => anyhow::bail!("focuser did not answer"),
    };

    Ok((
        EafDriver {
            name,
            max_step,
            cmd_tx,
            telemetry_rx,
            join_handle: Some(join_handle),
        },
        initial,
    ))
}

fn read_telemetry(eaf: &zwo_eaf::Focuser, with_temperature: bool) -> EafTelemetry {
    let (moving, hand_control) = eaf.is_moving().unwrap_or((false, false));
    EafTelemetry {
        position: eaf.position().unwrap_or(0),
        moving,
        hand_control,
        temperature_c: if with_temperature { eaf.temperature().ok() } else { None },
    }
}

fn focuser_loop(
    eaf: zwo_eaf::Focuser,
    name: String,
    max_step: i32,
    cmd_rx: Receiver<EafCmd>,
    telemetry_tx: Sender<EafTelemetry>,
    log_tx: Sender<LogEntry>,
) {
    const POLL_MOVING: Duration = Duration::from_millis(60);
    const POLL_IDLE: Duration = Duration::from_millis(400);
    const TEMP_EVERY: Duration = Duration::from_secs(2);
    const HEARTBEAT: Duration = Duration::from_secs(1);

    let mut last = read_telemetry(&eaf, true);
    let mut last_sent = Instant::now();
    let mut last_temp = Instant::now();
    let mut temperature = last.temperature_c;
    let _ = telemetry_tx.try_send(last);

    loop {
        let wait = if last.moving { POLL_MOVING } else { POLL_IDLE };
        match cmd_rx.recv_timeout(wait) {
            Ok(EafCmd::MoveTo(p)) => {
                let p = p.clamp(0, max_step);
                // The SDK rejects a move while the motor is turning, so a
                // jog or Move issued mid-move would be lost. Stop the current
                // move first; the newest target wins. A hand-controller move
                // cannot be stopped from here, and the SDK reports that.
                let (moving, hand) = eaf.is_moving().unwrap_or((false, false));
                if moving && !hand {
                    if let Err(e) = eaf.stop_and_wait(Duration::from_secs(2)) {
                        let _ = log_tx.try_send(LogEntry::error(format!("{name}: stop before move failed: {e}")));
                    }
                }
                if let Err(e) = eaf.move_to(p) {
                    let _ = log_tx.try_send(LogEntry::error(format!("{name}: move to {p} failed: {e}")));
                }
                // Report the motion immediately rather than after the next poll.
                last.moving = true;
            }
            Ok(EafCmd::Halt) => {
                // Wait for the motor to actually stop, so a move queued right
                // behind the halt (autofocus abort returning to start) is not
                // rejected by the SDK as "still moving".
                if let Err(e) = eaf.stop_and_wait(Duration::from_secs(2)) {
                    let _ = log_tx.try_send(LogEntry::error(format!("{name}: halt failed: {e}")));
                }
            }
            Ok(EafCmd::Stop) => break,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        // The temperature read is a separate USB transaction and the value
        // moves slowly; don't let it slow down position polling during a move.
        let read_temp = !last.moving && last_temp.elapsed() >= TEMP_EVERY;
        let mut now = read_telemetry(&eaf, read_temp);
        if read_temp {
            last_temp = Instant::now();
            if now.temperature_c.is_some() {
                temperature = now.temperature_c;
            }
        }
        now.temperature_c = temperature;

        let changed = now.position != last.position || now.moving != last.moving || now.hand_control != last.hand_control;
        if changed || last_sent.elapsed() >= HEARTBEAT {
            // A full channel means the UI is behind; the newest snapshot
            // supersedes the queued ones, so dropping is fine.
            let _ = telemetry_tx.try_send(now);
            last_sent = Instant::now();
        }
        last = now;
    }
    // `eaf` drops here and closes the SDK handle on this thread.
}
