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
recv_server.py        ->  the laptop side: receives all three streams and
                         serves live stats at http://localhost:8080/
run.sh                ->  build/flash wrapper: ./run.sh exp1|exp2
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
- Accelerometer is streamed at the same cadence in both variants, so
  accel↔frame alignment works identically for either odometry approach.
- Variant 2 (LK) has just gained UDP streaming — see
  `../../firmware/k10-lk-track/PLAN.md` for its verification status;
  the tracker's gates (`LKT_MIN_EIG`, `LKT_MAX_MEAN_SSD`) are unverified
  on real footage and may need `lkt_set_gates()` tuning via serial before
  track counts look sane in the receiver.
- Ideas for later variants: stream raw downsampled frames (bandwidth test),
  5-point essential-matrix pose packets, or optical-flow residual instead
  of feature positions.
