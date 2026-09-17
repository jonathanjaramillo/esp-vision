# k10-fast-corners — PLAN

FAST-9 corner detection on the UNIHIKER K10 (ESP32-S3 N16R8) with live camera
feed and corner markers drawn on the built-in 2.8" ILI9341 screen.

## Goal

Detect image corners in the GC2145 camera feed in real time and display them
as markers on the live view, with a small HUD (threshold, stride, corner
count, fps). Adjustable at runtime via the front buttons.

## Architecture

```
GC2145 (240x320 PORTRAIT RGB565, DVP, XCLK 16 MHz) — QVGA on the K10
   │  register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, fb=2, xQueueCam)
   ▼
pipeline_task (FreeRTOS, core 0, prio 5; vTaskDelay(1) per frame)
   │  1. xQueueReceive  -> camera_fb_t *fb   (raw RGB565, no JPEG decode)
   │  2. grayscale      -> 76.8 KB uint8  (internal SRAM)
   │  2b. 2x2 downsample -> 120x160 uint8 (19.2 KB) + one 3x3 blur
   │  3. fast_corner_detect -> up to 96 corners (full-ring FAST-9,
   │                           score-ranked, spacing 4 px @ half-res)
   │  4. copy 1:1         -> 240x320 RGB565  (150 KB, PSRAM) + stamp markers
   │  5. invalidate feed_img + small canvas HUD strip + one lv_task_handler()
   ▼
feed_img (lv_img TRUE_COLOR -> dispBuf) + LVGL canvas HUD strip
   -> ILI9341 (SPI; full-screen flush ~70 ms/frame -> ~9 fps ceiling)
```

- Camera orientation: the K10's `register_camera(..., FRAMESIZE_QVGA, ...)`
  already delivers a 240×320 PORTRAIT frame that matches the ILI9341
  screen 1:1 — camera (cx, cy) == display (x, y). No rotation. (An earlier
  build assumed 320×240 landscape and rotated it, which scrambled the image
  into 4 squished sub-frames — verify `fb->width`/`fb->height` before
  transforming coordinates.)
- We **own** the camera queue (our own `register_camera` call) and do NOT use
  `k10.initBgCamerImage()` — that would spawn the framework's display task
  with its own queue and stretched-image coordinate mapping. Drawing markers
  directly into the frame buffer avoids any camera→screen coordinate
  mismatch.
- `feed_img` is created BEFORE `creatCanvas()` so the canvas renders above it.
- The full frame goes through a TRUE_COLOR `lv_img` (same path the framework
  display task uses), **not** through the 4-byte alpha canvas (that measured
  ~86 ms/frame). The canvas only repaints a 240×20 HUD strip + one text line.
- All LVGL calls happen in the pipeline task only; button task and loop()
  never touch LVGL; CDC writes are guarded by `availableForWrite()`.
- Measured @ 240 MHz: detect ~13 ms (stride 1), display ~70 ms (SPI flush
  of 240×320 RGB565 dominates) → ~9 fps, stable for many minutes. The only
  upside levers are a faster SPI flush or a smaller flush area.

## Algorithm — FAST-9 (Rosten & Drummond)

- A pixel is a corner when **9 contiguous** pixels of the full radius-3
  circle-16 are all brighter than center + t, or all darker than center − t.
  (Sampling only a 9-point half-ring and accepting 7-of-9 — the original
  implementation — flags every pixel along a vertical edge as a corner.)
- The 16 ring pixels are folded into two 16-bit polarity masks. The circular
  9-run test is `has_run9(m)`: duplicate the mask (`d = m | m<<16`) and AND
  `d>>0..d>>8`; a set bit in the result means a 9-run starts there.
- A lossless 4-point pre-check (indices 0/4/8/12; any 9-run contains ≥2 of
  them) rejects most pixels after 4 loads instead of 16.
- Ring loads use seven row pointers with constant dx offsets — no per-pixel
  multiply and no pointer-array indirection.
- **FAST proposes, Shi-Tomasi disposes** (the ORB arrangement). Every raw ring
  hit is pooled, scored by the **Shi-Tomasi min-eigenvalue** of the structure
  tensor over a 5×5 window (10×10 of the original frame at half-res), ranked
  strongest-first, then emitted under Chebyshev suppression (radius 4 px,
  capped at 96).
  - The FAST arc score is **not** used for ranking. It cannot tell a corner
    from an edge with a one-pixel jog, and an edge has far more pixels than a
    corner: at a 4 px spacing one 100 px contour can host ~25 detections and
    eat the whole 96-corner budget. min(λ1, λ2) is ≈ 0 wherever the gradient
    has a single dominant direction, so edges rank below corners regardless of
    how contrasty they are.
  - Selection must be by strength, not scan order, for a second reason: a
    plain first-wins cap fills the first ~7 rows and hides the rest of the
    frame — that was the original "plus signs bunched at the top" symptom.
  - The candidate pool is 4096. If it saturates, the dropped tail is dropped
    in *scan order*, which reintroduces exactly that bias — so the stats line
    prints `POOL-FULL` when it happens. Raise `t` if you see it.
- **Detect at half resolution.** The luma frame is 2x2 box-averaged to
  120x160 before FAST, with one 3x3 blur pass on top. This does three
  things at once: FAST's radius-3 ring then spans ~6 px of the original
  frame, so markers land on large-object corners instead of 1-2 px texture;
  the box average halves the sensor noise (the GC2145 feed has a mean
  |neighbour difference| of 15-19 gray levels, which otherwise floods FAST);
  and the scan is 4x cheaper. Full-res detection locks onto fine detail and
  misses soft/rounded large corners — which is what the camera actually sees.
- `stride` (1 or 2): stride 2 scans every 4th pixel for a ~4× speedup with
  negligible marker drift (markers are 5×5 px plus signs).

Measured @ 240 MHz (half-res, stride 1): detect ~12 ms, display ~68 ms (SPI
flush of 240x320 RGB565 dominates) → ~9 fps. The flush, not detection, is
the ceiling.

## Controls

| Input  | Action                                   |
|--------|------------------------------------------|
| A tap  | threshold +4 (8..96, default 24)         |
| B tap  | threshold −4                             |
| A + B  | toggle stride 1 ⇄ 2                      |
| A or B, held ≥600ms | toggle the display on/off (same action as `s`) |
| `d`    | (serial) dump the 120×160 detector luma  |
| `D`    | (serial) dump the raw 240×320 RGB565     |
| `s`    | (serial) toggle the display on/off — see "Screen on/off toggle" below |

LED: green ≥ 8 fps, yellow ≥ 4, red below (display ceiling ≈ 9 fps).
Serial (1 Hz): `fps=.. detect=..us draw=..us t=.. stride=.. corners=..`

## Screen on/off toggle (2026-09-17, verified live)

The SPI display flush was always the frame-rate ceiling (see "Algorithm"
above), not detection or ORB — confirmed once WiFi streaming made it obvious
detect+orb (~15-20ms combined) had headroom `draw` (~75-80ms) didn't. Typing
`s` into the serial monitor now toggles the on-device preview at runtime, no
reflash needed, useful for a WiFi-streaming/VO session where nobody's
watching the K10's own screen.

- Screen ON:  `draw≈75-80ms`, steady **8 fps** (unchanged from before).
- Screen OFF: `draw=0us`, steady **24 fps** — **3x** throughput, measured live
  switching back and forth on the running device. ORB/accel UDP streaming
  (`udp_ok` counter) keeps climbing through the toggle in both directions —
  streaming is fully independent of the display.
- On the OFF transition the canvas draws one "SCREEN OFF / streaming..."
  message so the K10 doesn't look frozen/dead, then makes no further LVGL
  calls until toggled back on (that's what buys the frame-rate gain — no
  further SPI flush happens at all, not even a cheaper one).
- `g_screen_on` (default ON) is checked once per pipeline iteration; toggling
  is a plain serial-set flag exactly like `g_dump_req`, no locking needed
  (single writer in `loop()`/button callbacks, single reader in the pipeline
  task).
- **Also bound to A or B, held ≥ `LONG_PRESS_MS` (600ms)** — both buttons were
  already spoken for (tap = threshold +/-4, A+B = stride toggle) and the
  Button class only exposes pressed/unpressed callbacks, no built-in
  long-press, so `on_button_{a,b}_pressed` just stamps `millis()` and
  `on_button_{a,b}_released` decides tap-vs-hold from the elapsed time,
  falling through to the normal threshold action on a quick tap. Verified live
  on the device: holding A blanked the screen ("SCREEN OFF"), and a
  subsequent quick tap on A still incremented the threshold normally
  (`t=24` -> `t=28` in the serial log) — tap/hold are cleanly distinguished.
  Known limitation, not tested: holding A+B together (the stride-toggle
  combo) for >=600ms will *also* fire both single-button long-presses, since
  `buttonA`/`buttonB`/`buttonAB` are independent Button objects each polling
  their own GPIO(s) — toggling the screen twice (net no-op) alongside the
  stride toggle. Harmless but worth knowing if button behavior ever looks odd
  during a combo hold.

**Bug found + fixed (2026-09-17): toggling back ON didn't bring the camera
feed back.** The OFF transition paints an *opaque* full-screen black rect for
the "SCREEN OFF" message onto the LVGL canvas, which sits ABOVE `feed_img`
(deliberately, so the HUD strip normally overlays the feed — see setup()).
But the per-frame ON path only ever repaints the small 20px HUD strip, never
the rest of the canvas, so that opaque black rect stayed stuck over the
entire screen forever after — `feed_img` genuinely was updating underneath
the whole time, just invisibly. Fixed with one call on the OFF->ON edge:
`k10.canvas->clearLocalCanvas(0, 0, SCR_W, SCR_H)`, which zeroes the canvas's
ARGB buffer back to fully transparent so the feed shows through again.
Confirmed live: holding a button off then on again now genuinely restores
the camera preview. **Lesson for any future full-screen canvas overlay**:
anything drawn opaque on the canvas needs an equally-explicit clear before
the screen can look "normal" again — the per-frame HUD path only ever
touches its own small strip and will never undo it for you.

## Capturing a real frame

The 2.8" screen and 5×5 markers cannot tell you *why* a detection fired, and
synthetic frames do not reproduce the interesting failures (adding noise to a
fixture dithers away the very quantisation contours you are chasing). Pull a
real frame instead — then thresholds and scoring can be tuned on the host with
no reflash.

```
pio device monitor          # ...then quit it; only one process can hold the port
tools/capture.py            # sends 'd' -> gray.png  (what FAST actually sees)
tools/capture.py --raw      # sends 'D' -> frame.png (raw RGB565, full colour)
```

Then tune offline against that frame, no reflashing:

```
c++ -O2 -std=c++17 -I ../../lib/fastcorner -I ../../lib/orb \
   tools/offline_detect.cpp ../../lib/fastcorner/fastcorner.cpp \
   ../../lib/orb/orb.cpp -o /tmp/od
/tmp/od frame.bin marked.ppm 24      # last arg = threshold
```

Verified end-to-end this way on a real doorway scene (2026-09-17): 15 candidates,
6 corners at t=24, all on genuine geometry (door edge apex, frame junctions) and
**zero** on the flat wall or flat door face.

`tools/capture.py` needs `pyserial`; Pillow is optional (without it you get a
`.pgm`/`.ppm`, which opens fine anywhere). The dump runs in the pipeline task
under the usual `availableForWrite()` guard, chunked with `vTaskDelay()` so the
TWDT stays quiet — expect a second or two of dropped frames.

## Host tests

```
c++ -O2 -std=c++17 -Wall -I ../../lib/fastcorner \
   test/host_test.cpp ../../lib/fastcorner/fastcorner.cpp -o /tmp/fct && /tmp/fct
```

Flat field → 0 corners; straight edge → 0; white square → exactly 4, one per
corner; a high-contrast staircase edge running past a *lower*-contrast square →
all 4 square corners survive and **none** of the 24 raw staircase hits are
emitted; and the RGB565 red ramp check that pins bug (1) above.

Note when reading these: the staircase fixture passes under the *old* arc-score
ranking too, so it does not by itself prove the Shi-Tomasi change earns its
keep — that change rests on the argument in the Algorithm section, not on a
measurement. The luma and byte-order fixes, by contrast, are measured on real
frames off the device.

**Synthetic fixtures did not, and could not, find the byte-swap bug** — a
fixture builds its own pixels and so bakes in whatever byte order the author
assumed. It took one `tools/capture.py --raw` dump, viewed as an image. Reach
for the dump first next time.

## ORB feature extraction (2026-09-17; wired into main.cpp and verified live)

`src/orb.h` / `orb.cpp` turns each FAST corner into a rotation-aware 256-bit
descriptor, following the same "verify on host before touching the device"
workflow as the detector:

- **Orientation**: intensity centroid (Rosin) over a circular patch of radius
  `ORB_PATCH_RADIUS` (9 px) — `angle = atan2(m01, m10)`. Same method the ORB
  paper uses.
- **Descriptor**: 256 fixed intensity-pair tests, rejection-sampled from an
  isotropic Gaussian so every point stays within the patch radius *even after
  rotation* (rotation preserves vector length, so sampling inside the circle
  up front is what keeps the rotated pattern in-bounds — no separate clamp
  needed). This is the ORB paper's untrained "steered BRIEF" baseline, not
  OpenCV's greedily-learned pattern — no OpenCV dependency anywhere in this
  file. The pattern is built once from a fixed-seed PRNG so it's identical
  every run/device (both sides of a future match need the same pattern).
- Matching primitive: `orb_hamming()` — popcount of the XOR, 0..256.

Host test (`test/orb_test.cpp`) checks determinism, border rejection, and
rotation invariance using an alias-free synthetic patch (a continuous
template function sampled at each pixel's *inverse*-rotated coordinate, so
there's no interpolation artifact from rotating an already-rasterized image).
Measured: worst-case Hamming distance 18/256 across 8 rotation angles
(0-360°) of the same patch, vs. 127/256 for an unrelated patch — right where
theory puts the "same feature" vs. "random" bands.

Verified on a synthetic white square (4 corners, symmetric under 90°
rotation): all 4 descriptors are pairwise **identical** (dist=0/256) — the
expected result once each is steered into its own canonical orientation,
since a right-angle corner looks the same as any of its 90°-rotated copies.

```
c++ -O2 -std=c++17 -I ../../lib/orb \
   test/orb_test.cpp ../../lib/orb/orb.cpp -o /tmp/ot && /tmp/ot
c++ -O2 -std=c++17 -I ../../lib/fastcorner -I ../../lib/orb \
   tools/offline_detect.cpp ../../lib/fastcorner/fastcorner.cpp \
   ../../lib/orb/orb.cpp -o /tmp/od
/tmp/od frame.bin marked.ppm 24   # now also prints angle + nearest-neighbour Hamming per corner
```

**Wired into `main.cpp` and verified live on the device (2026-09-17).** The
pipeline task runs `orb_compute_batch` on `detBuf` right after
`fast_corner_detect`, HUD shows `N cnr M orb`, and the serial stats line
reports `orb=<us>(<got>/<total>)`. 15 s of continuous frames, real scene, no
crash, no TWDT reset: steady **8 fps** (unchanged — the ~73 ms SPI flush still
dominates), **detect ~7.8 ms, ORB ~6 ms**, **20-27 of ~25 corners/frame** get
a descriptor (the rest are within `ORB_PATCH_RADIUS` (9) of the half-res
frame edge — `fast_corner_detect`'s own border inset is only 4, so those are
silently skipped by `orb_compute_batch`; widen the inset to >=10 if the
drop rate matters later).

Not done yet: nothing consumes the descriptors on-device. `s_feats[]` is
filled every frame and immediately overwritten — no matching, no streaming.
That's the next step (see "Next: streaming to a host for VO" below).

## Streaming to a host for visual odometry (DONE — verified live 2026-09-17)

Went with WiFi UDP instead of USB-CDC: the K10 already has a proven WiFi STA
path (see k10-smoke), the framework's httpd/CDC-write constraints don't apply
to raw sockets, and it frees the USB-CDC link for `tools/capture.py`/serial
stats at the same time as streaming.

**Two independent streams, both from `main.cpp`, sent as fixed-layout
`#pragma pack(push,1)` UDP datagrams:**

- **ORB features** — one packet per camera frame, from the pipeline task,
  right after `orb_compute_batch`. Header (18 B: magic "ORBF", seq, t_us,
  frame_w/h, n) + up to `ORB_STREAM_MAX_CORNERS` (34) corners (38 B each: x,
  y, angle in milliradians, 32-byte descriptor) — capped so the packet
  (18 + 34*38 = 1310 B) never needs IP fragmentation over WiFi. Corners are
  already strongest-first out of `fast_corner_detect`, so truncating to the
  first N keeps the strongest, not an arbitrary scan-order subset. Port 5005.
- **Accelerometer** — independent of the camera, sampled from `loop()` at
  `ACCEL_PERIOD_MS` (50 Hz): a future visual-*inertial* pipeline wants IMU
  samples faster than frames, and `k10.getAccelerometerX/Y/Z()` just reads a
  value the vendor lib's own background task already keeps current, so this
  never blocks. 18-byte packet (magic "ACCL", seq, t_us, ax/ay/az). Port 5006.

`tools/stream_recv.py` decodes both (its `decode_orb_packet`/
`decode_accel_packet` are the reusable piece for an actual VO consumer, not
just the CLI). Verified live: **8.4-8.6 pkt/s ORB (~15.7 corners/pkt avg),
50 pkt/s accel**, stable over 25s continuous, both streams simultaneously,
`udp_ok` counter climbing steadily with 0 failures on the device side.

**Unicast to a configured host IP (`STREAM_HOST_IP`), not subnet broadcast.**
Broadcast was the first design (neither side needs to know the other's IP)
but measured DOA: the K10 connects, gets a correct IP/mask, computes a
correct broadcast address, and ICMP unicast to it works — but zero broadcast
UDP packets were ever observed arriving at a listener on the same WiFi. This
is consistent with the broadcast/multicast-over-WiFi filtering many mesh/home
routers do to save airtime (unicast between clients still works fine). Switch
back to broadcast only if your network is confirmed not to do this.

**Debugging trap that cost most of the diagnosis time, worth remembering:**
the *symptom* looked identical whether packets were truly not arriving or
were arriving but not yet visible — because the receiver was run as a
backgrounded/piped process, and Python fully-buffers `stdout` when it isn't a
tty. `tools/stream_recv.py` printed nothing for 15-20s of a real, working
stream, was killed by the test harness before the buffer ever flushed, and
looked exactly like total packet loss. Diagnosed by adding an on-device
`udp_ok`/`udp_fail` counter to the serial stats line (confirming the K10 side
believed every send succeeded) before suspecting the receiver itself; fixed
by `sys.stdout.reconfigure(line_buffering=True)` in `stream_recv.py`, at
which point the *original broadcast build's* packets would very likely have
shown up too — the switch to unicast was validated correct independently, but
this particular round of "no packets" evidence was not to be trusted. If a
future stream ever looks silent, reach for `-u`/unbuffered output and an
on-device sent-counter before concluding it's a network problem.

- CDC (`dump_b64`) is unaffected — still used for on-demand frame dumps
  (`d`/`D` serial commands), completely independent of this WiFi path.
- Open item: `loop()`'s WiFi connect is one-shot (no reconnect-on-drop). Fine
  for a bench session; add retry logic if the K10 needs to survive a WiFi
  blip unattended.
- Open question, still unresolved: keep drawing to the screen while
  streaming, or drop the display to spend the SPI-flush time budget on frame
  rate instead? VO wants fps more than it wants a live preview.

## Files

This project's own files (see `../../lib/*` for the shared modules it builds
on — moved out during the 2026-09-17 reorg into `esp-vision`):

- `../../lib/fastcorner/` — detector (full-ring FAST-9 + Shi-Tomasi ranking +
  blur/downsample)
- `../../lib/orb/` — ORB orientation + BRIEF-256 descriptor on top of
  detected corners
- `../../lib/k10image/` — RGB565 -> luma (byte-swap fix + BT.601 weighting)
- `../../lib/k10stream/` — UDP wire format + send helpers for ORB
  features/accelerometer (shared with `tools/stream_recv.py`'s decoder)
- `src/main.cpp` — setup/buttons, pipeline task, display, frame dump, WiFi UDP
  streaming (ORB features + accelerometer)
- `test/host_test.cpp` — off-device detector tests (see above)
- `test/orb_test.cpp` — off-device ORB tests (determinism, border, rotation)
- `tools/capture.py` — pull a frame off the board as a PNG (USB-CDC)
- `tools/offline_detect.cpp` — run the exact device pipeline over a captured
  `.bin`, render the markers, and print ORB descriptors/angles per corner;
  tune thresholds with no reflash
- `tools/stream_recv.py` — receive + decode the live ORB/accelerometer UDP
  streams (see "Streaming to a host for visual odometry")
- `platformio.ini` — same proven env as k10-smoke, plus WiFi (`lib_extra_dirs`
  pulls in the shared `lib/` modules; `WIFI_SSID`/`WIFI_PASSWORD`/
  `STREAM_HOST_IP` come from the shell environment — see "Build / flash")

## Build / flash

`WIFI_SSID`, `WIFI_PASSWORD` and `STREAM_HOST_IP` (the VO host's dotted-quad
IP) are read from the shell environment at build time (`platformio.ini`'s
`build_flags` uses `${sysenv.*}`) — they are never committed to source.
Export them once per shell session before building:

```
export WIFI_SSID="your-ssid"
export WIFI_PASSWORD="your-password"
export STREAM_HOST_IP="192.168.1.50"   # this laptop's IP on the K10's WiFi

pio run
pio run -t upload     # USB-C (/dev/cu.usbmodem1101); press RST if needed
pio device monitor
```

## Gotchas (learned the hard way — do not regress)
- **THE BIG ONE: the camera frame is byte-swapped RGB565, and the luma
  conversion was reading it natively.** Confirmed on a real frame pulled off
  the device (2026-09-17). The frame is stored big-endian on the wire, which is
  what the ILI9341/LVGL path wants — the display `memcpy`s it raw and looks
  perfectly correct. **That is exactly why this hid for so long: the screen is
  not evidence that the numbers are right.** The ESP32 is little-endian, so a
  `uint16_t` read scrambles the channels. With `V = RRRRRGGG GGGBBBBB` stored
  as `[b0, b1]`, a native read gives `N = b1<<8 | b0`, and decoding `N` yields:

  | decoded as | actually contains |
  |------------|-------------------|
  | "red"      | `G2G1G0 B4B3` — green's low bits + blue's high bits |
  | "green"    | `B2B1B0 R4R3R2` |
  | "blue"     | `R1R0 G5G4G3` |

  Low-order colour bits land in high-order luma positions, so any small colour
  change produces a huge luma jump — hard banded "contours" all over flat,
  smoothly-shaded surfaces. **This was the root cause of "markers trace lines
  along the colour contours instead of finding the real corner."**

  Measured on one real frame: mean |neighbour difference| **17.71 scrambled vs
  5.32 correct**. On-device after the fix: raw candidates **~600 → ~200**,
  reported corners **pinned at the 96 cap → 0-29 (scene-dependent)**, detect
  time **15 ms → 6 ms**. Fix is one line: `p = __builtin_bswap16(p);`.

  **The "GC2145 feed is noisy, mean |nd| 15-19" figure recorded here for months
  WAS this bug, not sensor noise** — 17.71 is squarely in that range. Real
  post-blur detector input now measures **1.57**.
- **The RGB565→luma weighting was independently wrong too.** Two more bugs,
  both fixed in `rgb565_luma()` (2026-09-17) and both real, though smaller than
  the byte swap above:
  1. The 5→8 bit expansion `(v<<3)|(v<<1)|(v>>2)` is **not monotonic** —
     measured over v = 0..31 its step ranges from **−17 to +55** (correct
     `(v<<3)|(v>>2)` steps 8 or 9 every time). Smooth red/blue ramps came out
     of it as sawtooths with cliffs that sometimes ran *backwards*.
  2. The 5:2:1 weights put ⅝ of the luma on **red** — the coarsest-quantised
     channel — and only ²⁄₈ on **green**, the finest. One red LSB was worth up
     to **23** luma levels, straight through the default t=24, and pure chroma
     gradients became apparent luma edges.

  **Blur cannot remove any of this** — neither the quantisation contours nor
  the byte-swap banding. Both are coherent step edges spanning a whole surface,
  not per-pixel noise, which is why the half-res + blur preprocessing never
  helped and why the dumped detector input was still visibly striped.
  Measured on a synthetic flat wall with a colour ramp: worst adjacent-pixel
  luma step **27 → 6** (t=24), and max luma step per red LSB **34 → 3**.
  Now: BT.601 weights applied to the raw 5/6/5 fields, no expansion step at
  all — `(r5*79 + g6*76 + b5*30) >> 5`.
- **The GC2145 feed is noisy** (mean |neighbour difference| 15–19 gray
  levels, raw). Threshold alone cannot compensate — even at t=60 there were
  ~1500 raw hits. Detect at half res (2x2 average + 3x3 blur) to suppress it.
  **SUPERSEDED — see the byte-swap entry above. That 15–19 figure was the
  byte-swap bug, not the sensor.** Real figures: 5.32 raw, 1.57 after
  downsample+blur. The preprocessing itself is verified correct (on iid noise
  it takes mean |nd| 12.3 → 6.0 → 1.2, and leaves a constant field exactly
  unchanged) — but **the entire justification for detecting at half resolution
  rested on the bogus noise figure and should now be re-derived.** At half-res
  FAST's ring spans ~6 px of the original frame, which costs real corner
  localisation. Full-res detection is now affordable (detect is 6 ms against a
  70 ms display flush) and is the obvious next experiment.
- **Never cap corners in scan order.** With a 4 px spacing only ~60 corners
  fit per row, so a first-wins cap reports only the top ~7 rows. Rank by
  score, then suppress.
- **Full-ring FAST-9, not a half-arc.** Sampling 9 of the 16 ring points and
  accepting 7-of-9 detects every pixel of a vertical edge as a corner.
- Verify the detector on the host (`g++` + synthetic frames) before flashing:
  a white square must give exactly 4 corners; a plain edge and a flat field
  must give 0.
- **TWDT:** when the sensor produces faster than the pipeline consumes, the
  frame queue never empties, the task never blocks, and IDLE on its core
  starves → task watchdog reset every ~6 s. Fix: `vTaskDelay(1)` at the end
  of every pipeline iteration (the framework's own display task does this).
- **USBCDC::write() can spin forever** (unbounded retry when the TX ring is
  full / EP not draining) and wedges the caller. Guard every CDC write with
  `Serial.availableForWrite()`, and avoid `Serial.printf` in hot paths (it
  mallocs through the global heap).
- **One-task LVGL:** a heavy LVGL task on core 1 plus CDC writes from core 0
  hung the box; core 0 + guarded CDC writes is the verified-stable setup.
- **The K10 QVGA frame is 240×320 portrait, not 320×240 landscape** —
  `fb->width==240, fb->height==320, fb->len==153600`. Never rotate or
  transpose it; camera coords are display coords.
- `k10.begin()` returns `void`; the PSRAM header is `<esp_heap_caps.h>`
  (not `heap_caps.h`) in this SDK; `corners[i]` is `corners.corners[i]`.
- Do NOT add `-DModel=None` (build script chokes on literal `None`); omit
  `Model` entirely — default flashes no speech model.
- Do NOT call `lv_task_handler()` from tasks other than the one driving
  LVGL, and do NOT touch the canvas from button callbacks; callbacks only
  touch the volatile tunables.
- Camera SSCB pins 47/48 share `Wire` I2C — leave `Wire` unused. Do NOT
  instantiate `AHT20` (crashes); avoid `initSDFile()` (loops w/o card).
- GC2145 high-res modes are broken (esp32-camera #664) — stay at QVGA.
- `canvasText`: use `count ≥ 50` (single-line path) and `autoClean=false`.
- `canvasRectangle` border width = `canvasSetLineWidth()` — set to 1 for the
  HUD strip or the 10px default border swallows everything.

## Open items / tuning knobs (if needed)

- Detection is only ~13 ms/frame, so the pipeline has budget to spare; the
  ~70 ms display flush (SPI) caps us at ~9 fps. If detection ever becomes the
  bottleneck again: raise `FC_MIN_SPACING`, reduce `FC_MAX_CORNERS`, or
  default `g_stride` to 2.
- If you outgrow the full-screen flush: raise the TFT SPI clock in the
  framework (TFT_eSPI / initScreen) or flush sub-regions.
