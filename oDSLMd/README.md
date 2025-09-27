# oDSLMd Daemon Extract

This directory contains a standalone build of the rpicam settings/preview daemon
and the minimum shared support code copied from the original `rpicam-apps`
project.  It is intended to make it easier to evolve the daemon independently of
other camera applications while preserving attribution and licensing.

The layout mirrors the upstream source tree so that existing include paths
remain valid:

* `core/`, `encoder/`, `image/`, `output/`, `preview/`, `post_processing_stages/`,
  and `utils/` provide the common infrastructure the daemon relies on.
* `daemon/` contains the HTTP controller implementation.
* `apps/` supplies the `rpicam-daemon` entry point and Meson build rules.

## Building

```
meson setup build
ninja -C build rpicam-daemon
```

The resulting binary links against the locally built `rpicam_app` support
library and exports no additional applications by default.

## Licensing

All files are copied verbatim from the upstream project and retain their
original copyright and SPDX headers.  The top-level `license.txt` from
`rpicam-apps` is included for convenience.
