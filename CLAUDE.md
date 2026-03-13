# Viewer — Live Camera Image Viewer

## Overview

A Rust + egui application for displaying live camera images with false-color mapping, interactive scaling, overlays, and camera controls. Designed to work with any image source (`image::DynamicImage`), with initial integration for SVBony USB cameras via the published `svbony` crate.

## Tech Stack

- **UI:** `egui` via `eframe` (immediate-mode GUI)
- **Plotting:** `egui_plot` (histograms, draggable scale lines)
- **Image processing:** `image` crate (`DynamicImage` as the universal frame type)
- **Camera:** `svbony` crate (with `image` feature) for SVBony cameras
- **Color mapping:** Custom — Grayscale, Hot, Parula, Viridis, Inferno, Magma, Red
- **Logging:** `tracing` + `tracing-subscriber`
- **Error handling:** `anyhow`

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                      eframe App                         │
│                                                         │
│  ┌──────────────┐   ┌──────────────────────────────┐    │
│  │  Side Panel   │   │       Central Panel          │    │
│  │              │   │                              │    │
│  │ Camera       │   │  ┌────────────────────────┐  │    │
│  │  Selection   │   │  │   Image Display        │  │    │
│  │  Controls    │   │  │   (false-colored,      │  │    │
│  │  (exposure,  │   │  │    zoomable,           │  │    │
│  │   gain, etc) │   │  │    with overlays)      │  │    │
│  │              │   │  └────────────────────────┘  │    │
│  │ Display      │   │  ┌────────────────────────┐  │    │
│  │  Colormap    │   │  │   Histogram + Range    │  │    │
│  │  Scale mode  │   │  │   (draggable min/max)  │  │    │
│  │  Gamma       │   │  └────────────────────────┘  │    │
│  │              │   │                              │    │
│  │ Statistics   │   │  Colorbar (optional)         │    │
│  │  FPS, mean,  │   │  Axes (optional)             │    │
│  │  stddev      │   │                              │    │
│  └──────────────┘   └──────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

### Data Flow

```
Camera Thread                    Main Thread (egui)
─────────────                    ──────────────────
svbony::Camera::get_image()
  → DynamicImage
  → compute stats (mean, stddev)
  → compute histogram
  → send via channel ──────────→ recv latest frame
                                 → apply colormap + gamma
                                 → update egui texture
                                 → paint image + overlays
                                 → paint histogram
```

### Key Design Decisions

1. **`DynamicImage` as the interchange type** — The camera thread produces `DynamicImage` values. The UI consumes them. This decouples camera source from display logic. Any source (svbony, file, simulated) just needs to produce a `DynamicImage`.

2. **Channel-based frame delivery** — A background thread captures frames and sends them via `std::sync::mpsc::channel` (or `crossbeam`). The UI thread drains the channel each frame, keeping only the latest. No shared mutable state.

3. **Colormapping on the UI thread** — Apply colormap, gamma, and scaling when converting to an egui texture. This keeps the pipeline simple and lets scale/gamma changes take effect immediately without re-fetching from the camera.

4. **Camera controls via `svbony` API directly** — Query `control_caps()` to discover available controls, render a slider/checkbox for each writable one. No intermediate abstraction layer.

## Implementation Plan

### Phase 1: Skeleton App
- [ ] Initialize `Cargo.toml` with dependencies: `eframe`, `egui`, `egui_plot`, `image`, `anyhow`, `tracing`, `tracing-subscriber`
- [ ] Create `src/main.rs` with minimal eframe app (empty window, side panel + central panel layout)
- [ ] Verify it builds and runs

### Phase 2: Image Display
- [ ] Create `src/colormaps.rs` — define colormap trait and implementations (Grayscale, Hot, Viridis, Inferno, Magma, Parula, Red). Each colormap is a `[u8; 256 * 3]` lookup table. Input: normalized `f32` in [0,1] → RGB.
- [ ] Create `src/imageview.rs` — custom egui widget that:
  - Takes a mono image (as `&[u16]` or similar + width/height), scale range (min, max), gamma, and colormap
  - Applies scaling: `normalized = clamp((pixel - min) / (max - min), 0, 1)`
  - Applies gamma: `normalized = normalized.powf(1.0 / gamma)`
  - Looks up colormap → RGB
  - Creates/updates an `egui::TextureHandle`
  - Renders with `ui.image()`, using `TextureOptions::NEAREST` for pixelated zoom
  - Reports mouse hover position and pixel value under cursor
- [ ] Test with a synthetic gradient image

### Phase 3: Histogram & Scale Controls
- [ ] Create `src/histogram.rs` — computes histogram from mono image data (256 bins, linear spacing between data min and max)
- [ ] Display histogram in bottom panel using `egui_plot`
- [ ] Add draggable vertical lines on histogram for min/max scale range
- [ ] Add scale mode selector: Full (bit-depth range), Auto (per-frame min/max), Manual (draggable lines / DragValue fields)
- [ ] Add gamma slider

### Phase 4: Overlays
- [ ] Colorbar — vertical gradient strip beside the image showing the active colormap + scale values
- [ ] Coordinate axes — tick marks and pixel labels on image edges
- [ ] ROI selection — click-drag rectangle on image, report bounds
- [ ] Crosshair / pixel inspector — show value at mouse position

### Phase 5: SVBony Camera Integration
- [ ] Add `svbony = { version = "0.1", features = ["image"] }` dependency
- [ ] Create `src/camera.rs` — camera manager:
  - Enumerate cameras with `svbony::connected_cameras()`
  - Open selected camera with `svbony::Camera::open(id)`
  - Spawn capture thread: loop calling `cam.get_image(timeout)`, send `DynamicImage` over channel
  - Expose camera properties (sensor size, bit depth, pixel size)
- [ ] Camera selection dropdown in side panel (refresh button to re-enumerate)
- [ ] Camera controls panel:
  - Query `cam.num_controls()` + `cam.control_caps(i)` for each control
  - For each writable control: render slider (min..max range) + auto checkbox if supported
  - Key controls with dedicated UI: Exposure, Gain
  - Read-only controls (e.g., CurrentTemperature): display as text
- [ ] Start/stop capture button
- [ ] Display frame rate (computed from frame timestamps)
- [ ] Bit-depth-aware scaling (use `cam.property().max_bit_depth`)

### Phase 6: Simulated Camera Source
- [ ] Create `src/sim.rs` — generates synthetic `DynamicImage` frames:
  - Moving Gaussian blob + noise (similar to viewer.bak pattern)
  - Configurable resolution, frame rate, bit depth
- [ ] Selectable alongside real cameras in the UI (useful for development/testing without hardware)

### Phase 7: Polish
- [ ] Keyboard shortcuts (space = start/stop, r = reset scale, etc.)
- [ ] Persist settings (window size, last colormap, gamma) via `eframe::Storage`
- [ ] Frame recording to FITS files (stretch goal)
- [ ] Performance profiling — ensure 30+ fps at full sensor resolution

## Project Structure

```
viewer/
├── Cargo.toml
├── CLAUDE.md          ← this file
└── src/
    ├── main.rs        ← eframe app, top-level layout, panel wiring
    ├── imageview.rs   ← image display widget (colormap, zoom, overlays)
    ├── colormaps.rs   ← colormap definitions (LUTs)
    ├── histogram.rs   ← histogram computation + display widget
    ├── camera.rs      ← svbony camera manager (thread, controls)
    └── sim.rs         ← simulated camera source
```

## Build & Run

```bash
# Build
cargo build --release

# Run (will enumerate connected SVBony cameras)
cargo run --release

# Run without camera hardware (sim only)
cargo run --release
```

## Dependencies

```toml
[dependencies]
eframe = "0.31"
egui = "0.31"
egui_plot = "0.31"
image = "0.25"
svbony = { version = "0.1", features = ["image"] }
anyhow = "1"
tracing = "0.1"
tracing-subscriber = "0.3"
crossbeam-channel = "0.5"

[profile.test]
opt-level = 3
```
