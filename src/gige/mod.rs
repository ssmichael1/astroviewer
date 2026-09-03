//! App-owned GigE Vision transport: a synchronous, pure-Rust GVCP control plane
//! ([`gvcp`]), GVSP stream parsing and reassembly ([`gvsp`]), network
//! interface / socket helpers ([`nic`]), receive-thread scheduling
//! ([`platform`]) and the OS extensions the receive socket uses to take many
//! packets per call: Winsock coalescing on Windows ([`winsock`]), `recvmmsg`
//! batching on Linux ([`linux`]).
//!
//! This replaces the external `viva-gige` crate (and its `tokio` + `bytes`
//! dependencies) with a std-socket implementation the app owns end to end. Wire
//! layouts, register addresses, status codes and the spec-derived golden-byte
//! test vectors are adapted from the MIT-licensed `viva-gige` / `viva-gencp`
//! crates (<https://github.com/VitalyVorobyev/viva-genicam>); see the module
//! headers for the specific provenance.
//!
//! GenICam feature access (Exposure, Gain, PixelFormat, …) stays with
//! `cameleon-genapi`, bridged onto [`gvcp::Device`] in `crate::gev_camera`.

pub mod gvcp;
pub mod gvsp;
#[cfg(target_os = "linux")]
pub mod linux;
#[cfg(target_os = "macos")]
pub mod macos;
pub mod nic;
pub mod platform;
#[cfg(windows)]
pub mod winsock;
