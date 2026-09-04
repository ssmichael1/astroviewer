# AstroViewer

A live camera image viewer for astronomy, built with Rust and egui. It displays
frames from USB and network cameras (or FITS files) with false-color mapping,
interactive scaling, histograms, zoom, star-centroid overlays, live plate
solving, and FITS recording.

## Camera backends

Backends are selected at build time with Cargo features:

| Feature | Source | Connects by |
|---|---|---|
| `svbony` | SVBony USB cameras | discovery |
| `toupcam` | ToupTek USB/GigE cameras (and rebrands: Altair, Omegon, RisingCam, …) | discovery |
| `gev` | Any GigE Vision camera (pure-Rust transport + GenICam) | discovery or IP address |
| `indi` | INDI / INDIGO server | host\[:port\] address |
| `starsolve` | Centroid extraction + lost-in-space plate solving (tetra3) | — |
| `focus` | Focus assist: HFR readout and trend (implies `starsolve`) | — |
| `all` | Everything above | — |

FITS file playback is always built in.

```bash
# Build with every backend
cargo build --release --features all

# Or just what you need
cargo build --release --features svbony,indi
```

## Running

```bash
# Start idle (or reconnect to the last source used)
cargo run --release --features all

# Or name a source on the command line
viewer path/to/capture.fits          # bare FITS path
viewer file:/data/capture.fits       # explicit scheme
viewer svb:0                         # SVBony camera id
viewer toupcam:<id>                  # ToupTek enumeration id
viewer gev:192.168.0.2               # GigE camera by IP (or by discovered id)
viewer indi:astro.local:7624         # INDI server
```

With no argument the viewer reconnects to the last source it used; the same
descriptor strings are what it remembers between runs. `viewer --help` prints
the full list.

## Choosing a source: the Connect dialog

**Source ▸ Connect…** (or the **Connect…** button in the side panel) opens a
connection manager listing every backend compiled into the binary:

- Discovered devices appear grouped under their backend, one click to connect.
- **GigE Vision** has an IP field for cameras that are reachable by unicast
  but don't answer broadcast discovery (the last-used IP is remembered).
- **INDI** has a server field (`host` or `host:port`). After connecting, pick
  a device and control it from the Controls tab.
- **Files ▸ Open FITS…** opens the native file picker.
- **⟳ Refresh** re-enumerates all backends.

The Source menu also lists discovered cameras directly for quick switching,
plus Play/Stop for the current source. With no source selected, Play opens
the Connect dialog instead of doing nothing.

### GigE Vision troubleshooting

Controls appear but no image is a transport problem, and the **Log** tab
says which kind. Three seconds after acquisition starts the viewer reports
either *no GVSP packets received* (a host firewall or endpoint agent is
dropping inbound UDP to this program, the packet size is larger than the link
carries, the camera is waiting for a trigger, or another application holds
control) or *packets received but no frame completed* (packet loss or a
packet-size mismatch). The stream line logged at connect shows the negotiated
and effective packet size and the socket buffer the OS granted.

Environment variables for diagnosis:

- `GEV_PACKET_SIZE=1500` caps the GVSP packet size. Try this first on a NIC
  with jumbo frames enabled: a camera that accepts a size the path cannot
  carry streams nothing.
- `GEV_TRACE=1` logs packets per second, frames completed and decoded, and
  time spent on control traffic and decoding once a second.
- `GEV_PIXEL_FORMAT=Mono8` (or any symbolic entry) selects the pixel format
  to try first.

On Windows, allow the program through Windows Defender Firewall when
prompted; the viewer also sends a small datagram to the camera's stream port
so stateful firewalls treat the stream as reply traffic. The
`gev_stream` example (`cargo run --features gev --example gev_stream -- <ip>`)
is a headless version of the same pipeline with per-packet counters.

## Display controls

- **Colormaps:** Grayscale, Hot, Viridis, Inferno, Plasma, Magma, Cubehelix,
  Turbo.
- **Scale modes:** Full Range (bit-depth), Auto (per-frame min/max), ZScale,
  Manual.
- **Transfer functions:** Linear (with gamma) or Asinh (alpha up to 1000) for
  pulling faint detail out of high-dynamic-range frames.
- **Histogram:** live, with draggable min/max scale lines, optional log-y, and
  per-channel R/G/B curves for color sensors streaming RAW.
- **Background subtraction:** percentile-based temporal background for
  multi-frame FITS.
- **Zoom:** right-click-drag a rectangle on the image to open a zoom window
  (close with Esc or X, or draw a new rectangle; a stray left click leaves
  it alone).
- **Manual scale:** sliders plus exact numeric entry for the min and max.
- **Overlays:** colorbar, pixel axes, hover pixel readout.

Camera controls (exposure, gain, cooling, filter wheel, resolution, trigger
mode, and backend-specific advanced options) appear in the Controls tab and
side panel once a camera is connected.

## Plate solving (`starsolve`)

With the `starsolve` feature, the viewer extracts star centroids from live
frames and solves them against a star database (built once from the bundled
catalog on first run — the app offers to generate it). Overlays show detected
centroids, matched stars, and named bright stars. Solver parameters (pixel
sigma, min/max blob size, FOV estimate) are adjustable in the Plate Solve tab
and persist across runs.

For very busy frames, only the brightest few thousand centroids are drawn;
extraction and solving always use the full set.

## Focus assist (`focus`)

The Focus tab shows the median half flux radius (HFR) of the brightest 30
unsaturated, round stars in each frame, in large digits, with the best value
since the last reset and a trend plot of recent frames. HFR is measured from
the raw pixels around each centroid after local background subtraction, so it
does not shift with the detection threshold; smaller is better. Once a plate
solve has locked, the readout also gives HFR in arcseconds.

A whole-frame **sharpness** figure (gradient energy over variance, larger is
better) covers the far-out-of-focus regime where stars are donuts the
extractor no longer detects; switch the plot to it for coarse focusing, then
back to HFR. **ROI only** restricts measurement to stars inside the zoom
region, and **Label stars** writes each star's HFR on the image, which makes
tilt and field curvature visible. With a ToupTek focuser attached, samples
carry the focuser position and the best value shows where it was reached.

Focus measurement rides on the plate-solve worker, so it needs **Solve**
enabled in the Plate Solve tab; a star database is not required.

## Recording

**File ▸ Start Recording** (or the ● button in the toolbar) streams incoming
frames to `~/Documents/AstroViewer/astroviewer-YYYYMMDD-HHMMSS.fits`. RAW
color frames record the `BAYERPAT` keyword so calibration software can
demosaic them later. With `starsolve` built in and a solve locked, each frame
also carries a standard TAN WCS (`CTYPE`, `CRPIX`, `CRVAL`, `CD` matrix,
`RADESYS`) derived from the latest solve, so the file opens on the sky in
astropy, DS9, PixInsight and the like. Frames recorded before the first lock,
or after a size change the solve has not caught up with, are written without it.

## Odds and ends

- **Themes:** Dark, Light, and Night (deep-red night vision) via the Theme menu.
- **Keyboard:** `S` toggles the side panel, `B` the bottom panel; `Esc`
  closes the zoom window or Connect dialog.
- **Remembered between runs:** theme, colormap, scale mode, transfer, gamma,
  the axes and colorbar toggles, panel visibility and sizes, the active
  bottom tab, and the window size and position (`ui.json` in the config
  directory, next to `config.json`).
- **Warnings and errors** show as a count badge on the Log tab and, with the
  latest message, at the right of the status bar; clicking there opens the
  Log. Both clear once the Log tab has been viewed.
- **Ctrl-C** in the terminal (or SIGTERM) closes the window normally: the
  recording is flushed, the camera stopped and released, and settings saved.
  A second Ctrl-C exits immediately.
- Bit depth of a FITS file comes from its `BITDEPTH` keyword when present
  (this app's recordings write it) or else from `BITPIX`; only float data
  falls back to inferring it from the pixel values.
- The UI repaints when a frame actually arrives rather than on a fixed timer,
  so idle and low-frame-rate CPU usage stays low.
- If the app ever crashes, the panic message and backtrace are written to
  `~/Library/Application Support/astroviewer/last_panic.txt` (config dir on
  other platforms) — please include it when reporting a bug.
