# Visual odometry experiments — UNIHIKER K10

Two firmware variants stream feature/track data + raw accelerometer samples
from the K10 (ESP32-S3) to this laptop over UDP, for experimenting with
different visual-odometry approaches host-side. Same receiver, same wire
conventions, swappable at flash time.

```
exp1 (ORB features)   ->  variant 1: FAST-9 + ORB on-device -> streams
                         ORBF (features) + ACCL (accel). Sources:
                         ../../firmware/k10-fast-corners/src
exp2 (LK tracks)      ->  variant 2: FAST-9 detect + pyramidal
                         Lucas-Kanade on-device -> streams TRCK (tracks)
                         + ACCL. Sources: ../../firmware/k10-lk-track/src
recv_server.py        ->  the laptop side: receives all three streams,
                         serves live stats + the VO trajectory at
                         http://localhost:8080/
calibrate.py           -> one-time camera calibration (checkerboard, via
                         the existing 'D' serial dump) -> calib_fov0.json
vo.py                  -> pure monocular VO logic (matching, pose
                         estimation, triangulation, sliding-window BA) --
                         no sockets, independently testable
vo_worker.py           -> thread draining ORBF frames into vo.py, feeding
                         recv_server.py's /traj.json
replay.py              -> record/replay harness for offline VO tuning
                         (see "Monocular VO" below)
run.sh                 -> build/flash wrapper: ./run.sh exp1|exp2
                         [upload|monitor]
```

The firmware projects themselves live in `../../firmware/` with the shared
streaming/detector modules in `../../lib/` and per-project PLAN.md
histories; this folder is the experiment harness around them. PlatformIO
resolves `lib_extra_dirs = ../../lib` relative to the project (not the
cwd), so builds must run from `../../firmware/<project>` — the Runbook
below has the exact commands.

## Prerequisites

- `pio` on PATH (PlatformIO Core) and the board flashed as usual.
- WiFi credentials exported (already added to `~/.zshrc`):

  ```
  export WIFI_SSID="Fallyn"
  export WIFI_PASSWORD="*********"
  ```

  `platformio.ini` injects them via `${sysenv.*}` at build time — they are
  never committed to source.
- For the stats dashboard alone: nothing beyond the stdlib (`recv_server.py`
  runs with no third-party dependencies if VO is unused/uninstalled).
- For monocular VO (`calibrate.py`, `vo.py` and friends): `pip install -r
  requirements.txt` (opencv-python, numpy, scipy, pyserial). `recv_server.py`
  detects whether these import successfully and disables VO gracefully
  (dashboard keeps working either way) if they're missing.

## Runbook

1. Export the laptop's IP on the K10's WiFi — the firmware **unicasts** to
   it (`STREAM_HOST_IP` is baked in at build time; broadcast was tried and
   this router drops it, see `../../firmware/k10-fast-corners/PLAN.md`):

   ```
   export STREAM_HOST_IP=$(ipconfig getifaddr en0)   # e.g. 172.16.35.211
   ```

2. Start the receiver **first**, so it's already listening when the board
   boots:

   ```
   python3 recv_server.py            # stats at http://localhost:8080/
   ```

   It listens on 5005/5006/5007 and simply shows `(silent)` for whichever
   firmware isn't streaming, so it can stay up across reflashes.

3. Flash one variant at a time with `./run.sh` (from this folder — it checks
   the three env vars and auto-detects `STREAM_HOST_IP` if unset). Only one
   firmware streams at once:

   ```
   ./run.sh exp1 upload    # ORB approach   (firmware/k10-fast-corners)
   ./run.sh exp2 upload    # Lucas-Kanade approach (firmware/k10-lk-track)
   ./run.sh exp2 monitor   # serial monitor for either
   ```

   (Or the long way: `cd ../../firmware/<project> && pio run -t upload`
   with `WIFI_SSID`/`WIFI_PASSWORD`/`STREAM_HOST_IP` exported —
   `platformio.ini` picks them up via `${sysenv.*}`.)

4. Watch `http://localhost:8080/`: packets/sec ("messages"), frames/sec,
   features/tracks per packet (last + avg), latest accelerometer (ax/ay/az,
   |a|) with a window mean, and a scope plot of the accel samples.

5. On the board, **hold button A or B ≥ ~0.6 s** to toggle the screen off.
   The screen stops redrawing (that's the ~75 ms/frame SPI flush — the actual
   frame-rate ceiling), while detection/tracking + streaming keep running at
   materially higher FPS. Hold again to restore the preview. A quick tap of
   A/B is unaffected (threshold up/down in variant 1).

## Monocular VO (v1, variant 1 / ORB features only)

Pure monocular visual odometry (trajectory only — no persistent map, no loop
closure, no IMU fusion yet; see `vo.py`'s module docstring for the full
scope/limitations). The K10 has no gyroscope (the SC7A20H is accelerometer-
only), so the accelerometer stream is **not** used anywhere in this pipeline
— real inertial preintegration needs angular rate, which isn't available.
Monocular scale is therefore permanently ambiguous: the dashboard's
trajectory plot is relative-scale-only, fixed arbitrarily at bootstrap.

**Calibration is per FOV mode** — each `GC2145_FOV_MODE` selects a different
sensor window/crop, so K differs by mode even though the *streamed* ORB
coordinate space (120x160) doesn't. `GC2145_FOV_MODE=0` (stock, the default
`./run.sh exp1 upload` flashes) works fine but has a narrow FOV; a wider FOV
gives more scene context per frame (more/better-spread features, less risk
of essential-matrix degeneracy under rotation) at some fps cost. For VO,
**`wide_fov` (`GC2145_FOV_MODE=1`, 720x960/3 portrait, ~6.96 fps) is the
recommended starting point** — it stays in the portrait pipeline (no runtime
rotation path, unlike `sensor_1_4`/`sensor_full_1_5`) so it's the least risk
for the most FOV gain. See `../../firmware/k10-fast-corners/platformio.ini`
for the full env list if you want to try the others.

1. Flash the FOV mode you're calibrating for:
   ```
   ./run.sh exp1 upload wide_fov      # or omit the 3rd arg for stock FOV
   ```
2. With the board connected over USB and that firmware running, print a
   checkerboard (default 9×6 internal corners, 25 mm squares) and run:
   ```
   python3 calibrate.py --fov-mode 1   # captures 15 frames via the 'D'
                                        # serial dump, writes calib_fov1.json
   ```
   (`--fov-mode` must match whatever's flashed — `calibrate.py` doesn't
   verify this for you, a mismatch just silently produces bad poses later,
   not an error.) Hold the checkerboard at varied angles/distances for each
   capture (prompted interactively). A reprojection error under ~0.5 px is a
   good calibration; the script warns if it's above 0.8 px.
3. Point `recv_server.py` at that calibration file (it defaults to
   `calib_fov0.json`, so non-default FOV modes need `--calib` spelled out):
   ```
   python3 recv_server.py --calib calib_fov1.json
   ```
   It enables VO (prints "VO enabled" to stderr; if the file's missing or
   opencv/numpy/scipy aren't installed, it says why and falls back to
   stats-only, same as before this feature existed). The
   dashboard gains a "Monocular VO" section: a live top-down (x, z) trajectory
plot, current tracking state, map size, and inlier count.

**Tracking states**, shown in the dashboard and in each `/traj.json` point:
- `bootstrapping` — no map yet (startup, or just lost tracking); trying to
  initialize from the next well-matched, sufficiently-parallaxed frame pair.
- `tracking` — pose estimated via PnP against the existing map each frame.
- `lost` — tracking failed for several consecutive frames (occlusion, motion
  blur, panning too fast, camera covered); VO automatically restarts
  bootstrapping once matches recover. This produces a new disconnected
  trajectory `segment` (visible as a plot break), **not** a relocalization
  against the old map — v1 has no loop closure/relocalization.

**Offline tuning via record/replay** — the RANSAC/inlier/parallax thresholds
in `vo.py` are very likely to need tuning against real footage; iterate
without needing the hardware live each time:

```
python3 recv_server.py --record session1.bin     # capture a real session
python3 replay.py session1.bin                    # re-send it, real-time
python3 replay.py session1.bin --speed 0           # ...or as fast as possible
```

## Wire format (unicast UDP to `STREAM_HOST_IP`)

All packets are little-endian, packed 1-byte alignment, matching the
`#pragma pack(push,1)` structs in `../../lib/k10stream/k10stream.h`
(`recv_server.py`'s `struct` formats must stay in sync — check both when
changing either).

| Port | Magic  | Payload | From |
|------|--------|---------|------|
| 5005 | `ORBF` | 18 B hdr (seq, t_us, frame_w/h, n) + n × 38 B: x:i16 y:i16 angle_mrad:i16 desc[32] — ORB features, coords in the half-res detection frame (×2 → camera px) | variant 1 |
| 5006 | `ACCL` | 18 B: seq, t_us, ax:i16 ay:i16 az:i16 (raw sensor units) | both |
| 5007 | `TRCK` | 18 B hdr + n × 10 B: x_q4:i16 y_q4:i16 id:u16 age:u16 score:u16 — LK tracks; x_q4/16 = level-0 px (×2 → camera px), Q4 keeps LK's sub-pixel precision | variant 2 |

`seq` increments per packet; `t_us` is the board's `micros()` at
compute time — use the *pair* (seq↔t_us) for timing, since UDP may drop or
reorder. One feature/track packet per processed video frame → packet rate
**is** frames/sec.

## Notes / gotchas

- The board unicasts to `STREAM_HOST_IP`, so a changed laptop DHCP address
  means re-export + rebuild + re-flash — not a firmware bug. If the receiver
  shows `(silent)`, check `pio device monitor` first: the board's 1 Hz stats
  line prints `wifi=<ip|down> udp_ok=<n> udp_fail=<n>`, which separates
  "WiFi didn't come up" / "nothing sent" from "packets never arrived".
  Firewall/ sleeping-laptop sleep are the usual remaining culprits; macOS
  needs to be awake and on the same subnet.
- Accelerometer cadence differs slightly by variant as of the ZYXDA-gated
  accel fix (variant 1 / k10-fast-corners only, see its PLAN.md): variant 1
  now sends only on a genuinely fresh SC7A20H sample (tracks the sensor's
  true ~10 Hz ODR, no stale duplicates); variant 2 (k10-lk-track) still uses
  the older blind-timer read against the vendor lib's cached value. Not
  relevant to v1 monocular VO either way, since it doesn't consult the accel
  stream at all (see "Monocular VO" above).
- Variant 2 (LK) has just gained UDP streaming — see
  `../../firmware/k10-lk-track/PLAN.md` for its verification status;
  the tracker's gates (`LKT_MIN_EIG`, `LKT_MAX_MEAN_SSD`) are unverified
  on real footage and may need `lkt_set_gates()` tuning via serial before
  track counts look sane in the receiver.
- Ideas for later variants: stream raw downsampled frames (bandwidth test),
  5-point essential-matrix pose packets, or optical-flow residual instead
  of feature positions.
