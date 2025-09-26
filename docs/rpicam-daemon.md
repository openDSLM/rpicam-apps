# rpicam-daemon

`rpicam-daemon` is an experimental HTTP process that exposes an API for DSLR-style
workflows. The daemon is intended to grow into the backend that powers the Open DSLM UI.

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

All endpoints exchange JSON bodies. Responses always set `Content-Type` to
`application/json`.

### `GET /status`

Returns the current session state and settings:

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
    "auto_exposure": true
  }
}
```

### `GET /settings`

Returns only the current settings block.

### `POST /settings`

Updates the camera settings. Only the provided keys are modified. All keys are
optional but must obey the following rules:

- `fps` – floating point frames per second (> 0)
- `shutter_us` – shutter time in microseconds (≥ 0)
- `analogue_gain` – analogue gain (> 0)
- `auto_exposure` – boolean

Example payload:

```json
{"fps": 25.0, "auto_exposure": false}
```

### `POST /session`

Starts a logical capture session.

Payload:

```json
{"mode": "cinemadng"}
```

Supported modes are `preview`, `still`, `video`, `cinemadng` and `none`.
If a session is already running the endpoint returns HTTP 409.

### `DELETE /session`

Stops the active session. Returns HTTP 409 if nothing is running.

### `GET /preview`

Currently returns HTTP 503 – the preview transport will be wired up in a
follow-up change.

## Notes

This daemon currently tracks settings and state in-memory without driving the
camera pipeline. The skeleton allows the UI to integrate while the capture,
CinemaDNG output and HTTP streaming support are implemented incrementally.

