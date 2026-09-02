//! Per-OS scheduling help for the GVSP receive thread.

use std::io;

/// Raise the calling thread's scheduling priority as far as an unprivileged
/// process may. The receive thread must win the CPU from the UI and decode
/// threads while a frame's packets burst in at line rate; otherwise the socket
/// buffer is all that stands between a preemption and a hole in the frame.
/// Returns a description of what was applied.
///
/// Windows: `THREAD_PRIORITY_TIME_CRITICAL` (15, the top of the normal
/// priority class — what packet-capture and audio engines use). macOS: the
/// user-interactive QoS class. Linux and the BSDs: nice -10 for this thread,
/// which needs `CAP_SYS_NICE` or a raised `RLIMIT_NICE` and is otherwise
/// refused with `EACCES` — best effort.
pub fn raise_thread_priority() -> io::Result<&'static str> {
    #[cfg(windows)]
    {
        use windows_sys::Win32::System::Threading::{
            GetCurrentThread, SetThreadPriority, THREAD_PRIORITY_TIME_CRITICAL,
        };
        // SAFETY: GetCurrentThread returns a pseudo-handle that needs no
        // closing; SetThreadPriority only reads its two arguments.
        if unsafe { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL) } == 0 {
            return Err(io::Error::last_os_error());
        }
        Ok("THREAD_PRIORITY_TIME_CRITICAL")
    }
    #[cfg(target_vendor = "apple")]
    {
        // SAFETY: FFI with constant arguments; affects only the calling thread.
        let rc = unsafe { libc::pthread_set_qos_class_self_np(libc::qos_class_t::QOS_CLASS_USER_INTERACTIVE, 0) };
        if rc != 0 {
            return Err(io::Error::from_raw_os_error(rc));
        }
        Ok("QOS_CLASS_USER_INTERACTIVE")
    }
    #[cfg(all(unix, not(target_vendor = "apple")))]
    {
        // SAFETY: FFI with constant arguments. On Linux, PRIO_PROCESS with id 0
        // names the calling thread, not the whole process.
        if unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, -10) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok("nice -10")
    }
    #[cfg(not(any(windows, unix)))]
    {
        Err(io::Error::other("no thread-priority support on this platform"))
    }
}
