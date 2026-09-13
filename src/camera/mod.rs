//! Camera backends. Each submodule owns one kind of device: it enumerates,
//! opens, runs a capture thread that produces [`FrameData`](crate::FrameData)
//! frames over a channel, and exposes whatever controls that SDK or protocol
//! has. The registry in `sources` lists them for the Connect dialog.

#[cfg(feature = "gev")]
pub mod gev;
#[cfg(feature = "indi")]
pub mod indi;
#[cfg(feature = "svbony")]
pub mod svbony;
#[cfg(feature = "toupcam")]
pub mod toupcam;
