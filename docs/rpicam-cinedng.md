# rpicam-cinedng

Record the camera’s RAW stream as a CinemaDNG image sequence while optionally serving a live MJPEG preview over TCP. The output is a numbered `.dng` per frame that DaVinci Resolve and other NLEs recognize as a single clip.

## Features

- CinemaDNG sequence writer with per-frame metadata.
- Optional live MJPEG preview (HTTP multipart/x-mixed-replace).
- Supports standard rpicam options (exposure, gain, AWB, denoise, focus, etc.).
- Flexible file naming and safe defaults for sequences.

## Build & Install

Build rpicam-apps as usual. This target is built alongside other apps.

```
meson setup build
meson compile -C build
sudo meson install -C build
sudo ldconfig
```

After installation the binary is available as `rpicam-cinedng`.

## Basic Usage

- Time-based capture:
  - `rpicam-cinedng -t 10s -o /path/shot-%08u.dng`
- Frame-count capture:
  - `rpicam-cinedng --frames 300 -o frames/frame-%08u.dng`
- With live preview (port 8080 on all interfaces):
  - `rpicam-cinedng -t 10s -o frames/%08u.dng --preview-stream :8080`
- Specify sensor mode and FPS:
  - `rpicam-cinedng --mode 2028:1520:12:P --framerate 24 -o clip-%08u.dng`

## Output & Naming

CinemaDNG writes one `.dng` per frame; stdout is not supported.

- If `-o` is empty: uses `frame-%08u.dng` in the current directory.
- If `-o` is a directory: uses `<dir>/frame-%08u.dng`.
- If `-o` is a filename with an extension: becomes `name-%08u.ext` (e.g. `shot.dng` → `shot-%08u.dng`).
- If `-o` contains `%`: treated as a `printf`-style pattern and used verbatim.

Index starts at 0 and increments per frame. `--wrap N` resets the index modulo N (may overwrite earlier frames if the sequence is longer than `N`).

## Live Preview (MJPEG)

Serve a lightweight MJPEG stream over TCP to monitor framing/focus:

- Enable with `--preview-stream <host:port | :port | port>`.
  - `:8080` listens on all interfaces; `127.0.0.1:8080` binds localhost only.
- Open `http://<host>:<port>/` in a browser or MJPEG client.
- Preview encodes from the YUV viewfinder stream; adjust resolution using `--width/--height`.
- Quality can be set with `--quality <1-100>` (default 75).
- Security: no authentication/TLS. Bind to localhost or firewall the port as needed.

## Common Options

All standard rpicam video options apply. Commonly used:

- `-o, --output <pattern|file|dir>`: Output target and naming.
- `-t, --timeout <duration>`: Stop after duration (e.g. `10s`, `5000ms`).
- `--frames <N>`: Stop after N frames.
- `--mode W:H:bit-depth:P|U`: Select RAW sensor mode and packing.
- `--framerate <fps>`: Target FPS (subject to sensor/mode limits).
- `--preview-stream <host:port|:port|port>`: Serve MJPEG live preview.
- `--quality <1-100>`: MJPEG preview quality (default 75).
- `--wrap <N>`: Wrap frame index modulo N.

Examples for exposure/AWB/denoise/focus controls are shared with other rpicam apps; see the main documentation for the full list.

## CinemaDNG Details

The writer supports common RAW Bayer formats (10/12/16‑bit, packed/unpacked) and PiSP compressed RAW where available. It stores key metadata per frame so NLEs can decode correctly:

- CFA pattern, white level, black levels, ColorMatrix1, AsShotNeutral, Calibration Illuminant.
- ExposureTime, AnalogueGain (ISO), optional LensPosition (as SubjectDistance), DateTimeOriginal.
- A UniqueCameraModel tag is set along with DNGVersion.

Note: The Software tag currently reads “rpicam-still”; this is cosmetic and can be changed later without affecting compatibility.

## Workflow: DaVinci Resolve

- Import the folder; Resolve should auto‑group `%08u` sequences into single clips.
- If frames appear as stills, enable “Image Sequence” in the Media Pool.
- Adjust CinemaDNG decode settings (WB/highlight recovery) in Project Settings → Camera RAW → CinemaDNG.
- Resolve reads exposure and ISO from metadata; set WB manually if needed.

## Performance & Storage

RAW data rates can be high. Approximate throughput:

```
bytes_per_frame ≈ width × height × (bits_per_pixel / 8)
throughput ≈ bytes_per_frame × fps
```

- Packed 12‑bit reduces size vs 16‑bit; compressed PiSP RAW reduces further.
- Use fast storage (SSD/USB3). SD cards may drop frames at higher resolutions/FPS.
- If you encounter dropped frames, reduce `--framerate`, lower resolution, or switch to faster media.

## Troubleshooting

- “No RAW stream available”: ensure RAW is enabled (default for this app). The app requests both YUV (for preview) and RAW.
- “Output to stdout not supported”: provide `-o` pointing to a file/dir/pattern.
- MJPEG port in use or blocked: try a different port or adjust firewall.
- Resolve doesn’t group frames: ensure contiguous, zero‑padded numbering in one folder.

## Examples

- 24fps clip at 2028×1520 12‑bit packed, with live preview on 8080:
  - `rpicam-cinedng --mode 2028:1520:12:P --framerate 24 --preview-stream :8080 -t 30s -o take1-%08u.dng`
- Fixed frame count, wrap index every 100 frames (overwrites):
  - `rpicam-cinedng --frames 1000 --wrap 100 -o ring-%08u.dng`

## Notes & Limitations

- CinemaDNG writing to stdout is not supported.
- MJPEG preview has no authentication; protect it at the network level.
- Actual sensor modes and limits depend on the camera module and libcamera pipeline.

