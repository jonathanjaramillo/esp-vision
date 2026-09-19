# esp-vision

Modular on-device computer-vision tools for the UNIHIKER K10 (ESP32-S3),
plus the firmware apps that exercise them.

## Layout

```
lib/                    shared, reusable modules (PlatformIO auto-discovers
                         these via each firmware project's lib_extra_dirs)
  fastcorner/            FAST-9 corner detector (Shi-Tomasi ranked) +
                         blur/downsample helpers
  orb/                   ORB orientation + BRIEF-256 descriptor on top of
                         detected corners
  k10image/              RGB565 -> luma conversion (K10 camera byte-swap fix
                         + correct BT.601 weighting)
  k10stream/             UDP wire format + send helpers for streaming ORB
                         features / LK tracks / accelerometer samples to a host
  lktrack/               pyramidal Lucas-Kanade sparse point tracker
                         (host-testable — see k10-lk-track/test/)

firmware/               PlatformIO projects that build for the K10
  k10-fast-corners/      live FAST-9 + ORB detection, on-screen preview,
                         WiFi UDP streaming to a host — see its PLAN.md
  k10-lk-track/          FAST-9 + pyramidal LK tracking, streams LK tracks
                         + accel for visual odometry — see its PLAN.md
  k10-smoke/             full hardware smoke test (display, camera, SD,
                         buttons, sensors, audio, WiFi)
  camera-format-tests/   probes raw camera pixel-format/timing behavior

experiments/            build harnesses around the firmware projects
  visual-odometry/        two VO data-streaming variants (ORB features vs
                         LK tracks) + recv_server.py stats dashboard +
                         run.sh build wrapper — start here to run the
                         experiments

Each `firmware/<project>/PLAN.md` documents that project's architecture,
measured numbers, and hard-won gotchas — read those before changing anything
in `lib/`, since several of the "obvious" simplifications there were already
tried and measured wrong (see `k10-fast-corners/PLAN.md`'s "Gotchas").

## Secrets

WiFi credentials (`WIFI_SSID`, `WIFI_PASSWORD`) and the streaming host's IP
(`STREAM_HOST_IP`) are never committed to source. Each firmware project's
`platformio.ini` pulls them from the shell environment at build time via
PlatformIO's `${sysenv.*}` substitution — export them before `pio run`:

```
export WIFI_SSID="your-ssid"
export WIFI_PASSWORD="your-password"
export STREAM_HOST_IP="192.168.1.50"   # streaming firmwares: k10-fast-corners, k10-lk-track
```

## Building a firmware project

```
cd firmware/k10-fast-corners
pio run                # build
pio run -t upload      # flash over USB-C
pio device monitor
```

## Host-side tools and tests

The detector/ORB modules in `lib/` have no Arduino or camera dependency and
build/run on a laptop for fast iteration — see `firmware/k10-fast-corners/PLAN.md`
("Host tests", "Capturing a real frame") for the exact commands.

## Related repos (not vendored here)

- https://github.com/DFRobot/framework-arduinounihiker — the K10's Arduino
  framework/SDK (pulled automatically by PlatformIO's `platform` field; also
  kept as a local sibling checkout, `../framework-arduinounihiker`, for
  reference/grep, but intentionally not a submodule of this repo).
- https://github.com/UNIHIKER/unihiker-docs — UNIHIKER hardware/API docs
  (kept as a local sibling checkout, `../unihiker-docs`, same reasoning).
