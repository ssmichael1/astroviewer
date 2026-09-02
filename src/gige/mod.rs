//! App-owned GigE Vision transport: a synchronous, pure-Rust GVCP control plane
//! ([`gvcp`]), GVSP stream parsing and reassembly ([`gvsp`]), network
//! interface / socket helpers ([`nic`]), receive-thread scheduling
//! ([`platform`]) and, on Windows, the Winsock extensions the receive socket
//! uses ([`winsock`]).
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
pub mod nic;
pub mod platform;
#[cfg(windows)]
pub mod winsock;
