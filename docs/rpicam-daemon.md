# rpicam-daemon

`rpicam-daemon` exposes a small HTTP API for DSLR-style workflows on top of the
`rpicam-apps` pipeline. The daemon now drives the camera for CinemaDNG stills
and video capture, serves preview frames, and keeps the most recent capture
metadata in memory. By default all recordings are written to `/ssd/RAW`.

## Building

The target is built alongside the other applications:

```bash
meson setup build
ninja -C build rpicam-daemon
```

## Running

```bash
./build/apps/rpicam-daemon --port 8400
```

The process blocks and serves until a `SIGINT`/`SIGTERM` is delivered.

## HTTP endpoints

All JSON endpoints accept and return UTF-8 encoded JSON. Unless noted otherwise
the daemon responds with `Content-Type: application/json`.

### `GET /status`

Returns the current session state, settings and summary of the last capture:

```json
{
  "state": {
    "active": false,
    "mode": "none",
    "last_error": ""
  },
  "settings": {
    "fps": 24.0,
    "shutter_us": 0.0,
    "analogue_gain": 1.0,
    "auto_exposure": true,
    "output_dir": "/ssd/RAW",
    "mode": ""
  },
  "last_capture": null
}
```

`last_capture` becomes a structure such as

```json
{
  "type": "still",
  "frames": ["/ssd/RAW/frame-00000000.dng"],
  "count": 1
}
```

whenever a capture finishes successfully.

### `GET /settings`

Returns only the current settings block.

### `POST /settings`

Updates the camera settings. Only the provided keys are modified. Supported
fields:

- `fps` – floating point frames per second (> 0)
- `shutter_us` – shutter time in microseconds (≥ 0)
- `analogue_gain` – analogue gain (> 0)
- `auto_exposure` – boolean toggle for automatic exposure
- `output_dir` – directory where CinemaDNG files are written (defaults to `/ssd/RAW`)
- `mode` – optional sensor mode string (`W:H:bit-depth:P|U`) used to pick a specific RAW resolution; empty string resets to automatic selection. Use `rpicam-cinedng --list-cameras` to confirm exact values (e.g. the HQ/IMX477 lists `1928x1090` for HD and `3856x2180` for 4K).

Example payloads:

```json
{"fps": 25.0, "auto_exposure": false, "shutter_us": 40000, "analogue_gain": 2.0}
```

```json
{"mode": "1928:1090:12:P", "fps": 25.0, "auto_exposure": true}
```

```json
{"mode": "3856:2180:12:P", "fps": 25.0}
```

The HD example matches the HQ/IMX477 sensor's 1:1 cropped mode (≈1080p). The 4K
example selects the same sensor's full-resolution crop. If your camera lists
different values, copy the strings that appear under `--list-cameras`.

### `POST /capture/still`

Triggers a single CinemaDNG frame. The request blocks until the frame has been
written. The response contains the filenames that were created:

```json
{"frames":["/ssd/RAW/frame-00000017.dng"],"count":1}
```

A HTTP 409 is returned if another session is already active.

### `POST /recordings/video`

Starts a CinemaDNG video recording that runs until it is explicitly stopped.
The response contains the updated daemon status.

### `DELETE /recordings/video`

Stops the active video recording. A HTTP 409 is returned if no recording is
running.

### `GET /preview`

Returns the latest preview frame as a single JPEG image. The response has
`Content-Type: image/jpeg` and disables HTTP caching so that clients always see
the current camera view.

### `GET /preview/stream`

Returns a live MJPEG stream that browsers can display and refresh continuously.
The response uses `Content-Type: multipart/x-mixed-replace` and sends a
sequence of JPEG images. Example usages:

- Browser: open `http://localhost:8400/preview/stream`
- `curl`: `curl -N http://localhost:8400/preview/stream > stream.mjpeg`

Notes:
- One stream client at a time (additional clients receive HTTP 409).
- While streaming, single-shot preview `/preview` will return 503 (camera busy).

## Example commands

The following `curl` commands illustrate a typical workflow on the default
port (8400):

```bash
# Query status
curl http://localhost:8400/status | jq

# Configure a 25 fps pipeline with manual exposure (1/25s) and gain 2x
curl -X POST http://localhost:8400/settings      -H 'Content-Type: application/json'      -d '{"fps":25.0,"auto_exposure":false,"shutter_us":40000,"analogue_gain":2.0}'

# Switch to a 4K/UHD raw mode (IMX477 example) at 25 fps
curl -X POST http://localhost:8400/settings      -H 'Content-Type: application/json'      -d '{"mode":"3856:2180:12:P","fps":25.0}'

# Switch back to an HD/1080p raw mode (IMX477 example) at 25 fps
curl -X POST http://localhost:8400/settings      -H 'Content-Type: application/json'      -d '{"mode":"1928:1090:12:P","fps":25.0}'

# Capture a single CinemaDNG still
curl -X POST http://localhost:8400/capture/still

# Quick sequence: jump to UHD, grab a still, then return to HD
curl -X POST http://localhost:8400/settings      -H 'Content-Type: application/json'      -d '{"mode":"3856:2180:12:P","fps":25.0}'
curl -X POST http://localhost:8400/capture/still
curl -X POST http://localhost:8400/settings      -H 'Content-Type: application/json'      -d '{"mode":"1928:1090:12:P","fps":25.0}'

# Fetch a JPEG preview frame
curl http://localhost:8400/preview --output preview.jpg

# Start a CinemaDNG video recording
curl -X POST http://localhost:8400/recordings/video

# Stop the recording
curl -X DELETE http://localhost:8400/recordings/video
```

The CinemaDNG files created by still and video captures are written to
`/ssd/RAW` unless `output_dir` is changed via the settings endpoint.

## Compatibility alias

For convenience with older examples, the daemon also exposes a session endpoint:

- `POST /session` with body `{ "mode": "still" | "video" | "none" }` starts a still capture, starts video recording, or clears any active session.
- `DELETE /session` stops an active session (equivalent to `DELETE /recordings/video`).

Prefer the dedicated endpoints above for clarity (`/capture/still` and `/recordings/video`).
