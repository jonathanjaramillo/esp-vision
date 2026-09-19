# k10-lk-track — PLAN

FAST-9 corner detection + pyramidal Lucas-Kanade sparse tracking on the
UNIHIKER K10, with live trails drawn over the camera feed and the live track
set + accelerometer streamed to a host over UDP for visual odometry.

**Status: device-verified 2026-09-17 — both halves of the LK-vs-ORB comparison
stream live.** k10-fast-corners streams ORB+accel at fps=7; this project
streams TRCK (LK tracks) + accel at fps=7 with the on-device preview running,
0 UDP failures. Getting there required finding and fixing a real internal-SRAM
exhaustion bug (below), on top of the video-corrupting rewrite from an earlier
attempt (also below) — both are now resolved; see "Files" for the source layout
if you're not familiar with this repo's convention of keeping fixed bugs
documented here rather than only in commit messages.

An earlier attempt at this streaming rewrote the camera init (`esp_camera_init`
direct call, `fb_count 1`, `CAMERA_GRAB_LATEST`) and added a second task
relaying frames out of the driver, guessing at a fix for a starvation symptom
without device-testing it — that rewrite corrupted the on-device video feed
and was reverted (kept in a local `git stash` at the time of writing, not
deleted — but a stash is local-only and easy to drop, so don't rely on it
still being there; this file is the durable record). Streaming here instead
reuses the working `register_camera()` setup unchanged, mirroring
k10-fast-corners.

## WiFi + camera bug (found & fixed 2026-09-17, device-verified)

**Symptom:** with `k10stream_wifi_begin()` called in `setup()`, the camera
never produced a single frame. `cam_hal: Failed to get the frame on time!`
repeated forever at a steady ~800ms period, and the pipeline task's serial
stats line read `fps=0 track=0 detect=0` from boot onward — it never
recovered, for as long as the board stayed powered. Screen preview, WiFi
itself (`wifi up: <ip> -> ...`), and the accelerometer stream were all fine;
only the camera was affected. k10-fast-corners' identical `register_camera()`
config streamed fine with WiFi (fps=7) on the same board/network, ruling out
"WiFi + camera can't coexist on this board" as an explanation.

**Root cause: internal-SRAM exhaustion, not a timing/DMA problem at all.**
This project's ping-pong pyramid buffers (`detBuf`/`l1Buf`/`l2Buf`, each `[2]`
so `lkt_track()` has both the previous and current frame's pyramid live at
once) commit roughly 30 KB more internal SRAM at boot than k10-fast-corners'
single-buffered detector, because k10-fast-corners doesn't need a "previous
frame" pyramid. Measuring `heap_caps_get_info(MALLOC_CAP_INTERNAL)` at two
points in `setup()` (device-verified, both projects, same session) found:

| Project | Internal free after camera init | Internal free after `WiFi.begin()` | Largest free block after WiFi |
|---------|----------------------------------|--------------------------------------|-------------------------------|
| k10-fast-corners | 108,360 B | 40,696 B | 32,756 B |
| k10-lk-track (before fix) | 78,952 B | 11,292 B | **7,668 B** |

`WiFi.begin()` claims almost exactly the same ~67-68 KB of internal SRAM in
both projects — it's this project's ~30 KB larger *pre-WiFi* commitment that
pushes the post-WiFi largest-free-block down into single-digit KB. Below some
threshold in that range, the camera driver's own internal-SRAM needs during
active capture (DMA descriptors, per-frame housekeeping — not the frame
buffers themselves, which are already correctly in PSRAM) stop being
satisfiable, and it wedges permanently rather than degrading gracefully.

**Controls that got here (all on the same physical board/network, same
session) — each one killed a candidate theory before the real one:**
- No WiFi at all (last-good commit `d6049e6`): fps=8, 0 starvation. Baseline.
- WiFi on, everything else unchanged: fps=0 forever. WiFi is *a* trigger.
- WiFi call alone commented out, all buffers/includes/send-sites left in
  place: fps=8, 0 starvation. Confirms the WiFi call itself, not anything
  else added for streaming, triggers it.
- Screen forced off from boot (skips the ~75ms/frame draw+SPI-flush cost
  entirely): still fps=0, `track=0` — ruled out the abandoned camera-relay
  rewrite's entire theory (consumer holding the frame buffer too long). The
  pipeline never received a first frame to hold, screen-off or on.
- `register_camera()` moved to immediately after `k10.begin()`, before any
  large allocation, WiFi still on: still fps=0 — ruled out allocation
  *order* specifically (as opposed to total footprint, which turned out to
  be the actual variable).
- The heap measurement above, run on both projects: found the actual
  ~30 KB / single-digit-KB-largest-block difference.

**Fix:** move `grayBuf` (the RGB565->luma scratch buffer, `CAM_W*CAM_H` =
76,800 B) from internal SRAM to PSRAM outright, in `setup()`. This is safe
specifically because `grayBuf` is touched only by one sequential pass
(`k10_to_grayscale`, then `fast_downsample2x2` reads it once more) —
PSRAM's sequential bandwidth is fine there. It is *not* one of the
latency-sensitive pyramid buffers `lkt_track()` does ~2300 random bilinear
patch reads against per tracked point per frame (see the algorithm section
below) — those stay on internal SRAM exactly as before. There's no
before/after-fix comparison *with WiFi on*, since the pre-fix build never
produced a frame at all (`track_us`/`detect_us` read 0 the whole time it was
wedged); the comparison that's actually measured is against the no-WiFi
baseline: `track_us`/`detect_us` post-fix (~20-24ms / ~11-12ms) match that
baseline (~20-23ms / ~11-13ms) within noise, so the fix doesn't cost LK
anything measurable. fps does drop from 8 (no WiFi) to 7 (WiFi on) — small,
plausibly just WiFi's own periodic overhead, not investigated further since
it matches k10-fast-corners' fps=7-9 with WiFi on. Freeing grayBuf's 75 KB
is comfortably more than the ~30 KB of headroom needed. Post-fix,
device-verified over three separate flashes: 0-2 benign startup-transient
warnings (same pattern k10-fast-corners shows before its own pipeline task
starts, and sometimes none at all), then fps=7 steady, `udp_ok` climbing,
0 failures, sustained over 15+ seconds of monitoring with zero further
starvation warnings. Confirmed end-to-end (not just the firmware's own
send-side counters): `experiments/visual-odometry/recv_server.py` shows
`tracks.seen=true`, `frames_per_sec≈7.9`, `last_count=40`, `bad_packets=0`
— the TRCK bytes this firmware emits do parse against the host's
`<4sIIHHH`/`<hhHHH` format strings on real device traffic, not just in
`test/trck_wire_test.cpp`'s C-side offset assertions.

**If this resurfaces** (e.g., after `LKT_MAX_TRACKS` or pyramid levels grow):
measure internal-SRAM headroom the same way before guessing — `largest_free_block`
after `WiFi.begin()` is the number that matters, not total free (fragmentation
from earlier allocations can strand memory in blocks too small for whatever
the camera driver needs next).

Fields
below marked (measured) come from `test/lk_test.cpp` on synthetic frames;
everything else is a design decision, called out as such — see the sibling
`k10-fast-corners/PLAN.md` for why that distinction matters in this repo (it
found two real bugs that only a real-frame test caught, not a synthetic one).

## Goal

Detect corners in the GC2145 feed, track them frame-to-frame with pyramidal
LK, and draw each point's recent trail + current position on the live view.
Re-detect periodically to replenish points lost off-frame or through
occlusion — a continuous KLT loop, not one-shot detect-then-track.

## Architecture

```
GC2145 240x320 portrait RGB565 (register_camera, own queue — see k10-fast-corners)
   |
pipeline_task (core 0, prio 5)
   1. grayscale (k10_to_grayscale, byte-swap + BT.601 fix)
   2. downsample2x2 + blur3x3 -> detBuf[cur]   (120x160) = pyramid level 0
   3. lkt_pyramid_build -> level 1 (60x80), level 2 (30x40)
   4. lkt_track(prev pyramid, cur pyramid)     -- refines existing points only
   5. replenish (conditional): fast_corner_detect + lkt_add
   6. draw: raw frame + per-track trail + marker -> dispBuf (skippable at
      runtime — 's' / long-press, see "Screen on/off while streaming")
   7. stream: TRCK (all live tracks) -> udp:5007 once per frame, from the
      pipeline task; ACCL -> udp:5006 from loop() at 50 Hz
   8. swap cur/prev pyramid index (ping-pong, no copy)
   -> ILI9341 (same TRUE_COLOR lv_img path as k10-fast-corners)
```

- **Pyramid level 0 is the detector's own blurred half-res buffer** —
  free reuse, no separate luma pyramid. `lib/lktrack` doesn't know or care
  that level 0 came from the detector; it just wants a reasonably smooth
  8-bit image (LK gradients get noisy on raw sensor pixels, same reason
  `fast_blur3x3` exists in the first place).
- **detBuf/l1Buf/l2Buf are each a ping-pong pair** (`[2]`), one slot per
  parity of frame number. Building this frame's pyramid into the *other*
  slot leaves last frame's pyramid untouched as `prev` — no memcpy needed to
  keep two frames' worth of pyramid alive.
- Streams live tracks + accelerometer over WiFi UDP (added 2026-09-17 for the
  `experiments/visual-odometry` harness), reusing `lib/k10stream`'s wire-format
  pattern and the exact `register_camera()`/button/screen-toggle code
  k10-fast-corners already runs WiFi streaming with — see "Streaming to a
  host for visual odometry" below. Before that it was local-display-only.

## Algorithm — pyramidal Lucas-Kanade (`lib/lktrack`)

- **3 levels** (level 0 + 2 built by 2x2 box downsample), 9x9 patch
  (`LKT_PATCH_RADIUS` = 4), up to 8 iterations per level.
- Template and gradients come from `prev` only, computed once per point per
  level; each iteration resamples `cur` at the current guess and solves
  `d = G^-1 * sum(grad * (T - I))`. This is what keeps the per-point cost
  small — the iterative solve never touches more than a 9x9 (+1px gradient
  border) patch, at any level, for any number of tracked points.
- **Why a shared pyramid, not one built locally per point around its own
  window:** it looks like it violates "only process pixels around tracked
  points," but a local per-point pyramid needs area-averaging at every
  level to avoid aliasing (plain strided sampling turns the coarse levels —
  the ones supposedly buying capture range for large motion — into noise),
  and re-averaging overlapping per-point windows on every iteration is more
  total work above a handful of points. The shared pyramid is a ~1-2ms
  (TBD, measure on device) fixed cost independent of track count; the
  genuinely expensive part (the per-point iterative solve) stays local.
  This was the deciding factor in choosing a shared pyramid over the
  simpler-sounding per-point alternative during design.
- **Lost gates**, evaluated only at level 0 (finest):
  - min eigenvalue of the structure tensor `G`, mean-per-pixel, below
    `LKT_MIN_EIG` — using min-eig rather than `det(G)` matters: an edge's
    (large, ~0) eigenvalue pair can still pass a determinant gate at a
    large enough "large" one, and those are exactly the points that slide
    along the edge instead of getting dropped. (Same reasoning fastcorner.h
    uses Shi-Tomasi min-eig over the FAST arc score for ranking — see that
    PLAN.md.)
  - mean squared residual after convergence above `LKT_MAX_MEAN_SSD`.
  - out of bounds (no room for a full patch + 1px gradient margin) at any
    level, at the initial position or mid-iteration.
  - A degenerate (near-singular) `G` at a *coarse* level is not itself a
    lost condition — the guess just passes through unrefined to the next,
    finer level, where the gate above is the one that actually decides.
  - **Both default thresholds are estimates, not measurements** — derived
    from k10-fast-corners' measured background noise on this same blurred
    half-res buffer (mean |neighbour difference| ~1.57 gray levels) and a
    back-of-envelope real-corner gradient, not from an actual real frame
    (no captured `.bin` exists in this repo to test against offline). A
    synthetic fixture can prove the gate *rejects* garbage; it cannot prove
    the default *accepts* a real corner — that needs a real frame. Both are
    runtime-tunable with `lkt_set_gates()` (serial `[`/`]` for the
    eigenvalue gate, `{`/`}` for the residual gate, `r` to reset the
    lost-reason counters) specifically so bring-up doesn't need a reflash
    per attempt. `lkt_track`'s `stats` output (`lost_eig`/`lost_ssd`/
    `lost_bounds`, on the serial stats line) is what turns "tracks keep
    dying" into "which gate is firing" without a debugger on the device —
    see "Open items" below for the predicted failure mode if the eigenvalue
    default turns out to be too strict.
- **Scale convention**: level `L` position = level `L-1` position / 2, both
  directions, no half-pixel offset correction for the downsample's true
  sample-center shift. A constant offset would cancel inside the residual
  as long as it's used consistently in seeding, propagation, and
  resampling — and since pyramid level 0 already *is* the detector's
  coordinate system, corner seeds need no rescaling at all, only the
  propagation step (`lkt_track`'s `guess = 2 * refined`) needs the
  convention, applied once, consistently. Documented as a v1 simplification,
  not verified to be sub-pixel-neutral in practice — if trails look subtly
  offset from real corners on the device, this is the first thing to
  revisit.
- **Not implemented (v1)**: forward-backward error check (track A->B, then
  B->A, reject if the round trip doesn't land close to the start — a
  standard KLT robustness check, cheap to add if lost/false-track rates on
  real footage turn out to matter).

### Host test (`test/lk_test.cpp`) — measured

```
c++ -O2 -std=c++17 -Wall -I ../../../lib/lktrack \
   lk_test.cpp ../../../lib/lktrack/lktrack.cpp -o /tmp/lkt && /tmp/lkt
```

Synthetic frames use the same trick as `k10-fast-corners/test/orb_test.cpp`:
a continuous template function sampled at each pixel's coordinates (shifted
by a known translation), so the fixture has no rasterization artifact of its
own to confound what's being measured.

All pass: integer translation (3,-2) recovers within 0.3px; **subpixel**
translation (1.5, 0.5) recovers within 0.3px (this is the test that would
have caught a bilinear-sign or scale-convention bug — no such bug turned up
during development, but the test earns its place regardless); 12px
translation (needs the 3-level
pyramid's capture range) recovers within 1px; a 40px translation is
correctly marked **lost**, not silently wrong; a flat field's seed is
rejected by the min-eigenvalue gate; and `lkt_add`'s spacing filter rejects
a second candidate too close to the first.

**One bug the host test caught before this ever reached a device:** the
first version of the "large translation" test put the object near the edge
of a small (64x64) synthetic frame, and the tracker correctly reported the
point lost — but for the wrong reason. The true shifted position was outside
the bounds needed for a full patch at the coarsest pyramid level (16x16,
5px margin), so the test was measuring "hit the image edge," not "exceeded
capture range," despite the tracker's actual math having converged to
within 0.1px of the right answer before the bounds check rejected it. Fixed
by using a larger (160x160) synthetic frame so the true answer has room —
worth remembering if a future test looks like it's failing the thing it
claims to, but is actually hitting a fixture limitation instead.

## Replenish policy

- Trigger when live count < `REPLENISH_FLOOR` (20) **or** every
  `REPLENISH_PERIOD` frames (30, ~4s @ 8fps), whichever comes first — not
  every frame, which would needlessly churn track IDs.
- `fast_corner_detect` already ranks strongest-first (see
  `k10-fast-corners/PLAN.md`), so `lkt_add` filling the budget in that order
  and rejecting anything within `MIN_SPACING` of a live track is a single
  linear scan with no separate sort step.

## Controls (TBD — verify on device, adjust if awkward)

| Input | Action |
|-------|--------|
| A tap | detector threshold +4 (fewer new points on replenish) |
| B tap | detector threshold -4 |
| A or B held >= 600ms | toggle the display on/off (same as `s`) — see "Screen on/off while streaming" |
| A+B   | toggle trail drawing on/off (markers stay either way) |
| `d` (serial) | dump the 120x160 pyramid-level-0 luma |
| `D` (serial) | dump the raw 240x320 RGB565 |
| `s` (serial) | toggle the display on/off |
| `t` (serial) | toggle trails (same as A+B) |
| `[` / `]` (serial) | lower / raise the min-eigenvalue lost gate ×0.7 / ×1.4 |
| `{` / `}` (serial) | lower / raise the residual (SSD) lost gate ×0.7 / ×1.4 |
| `r` (serial) | reset the cumulative lost-reason counters |

LED: green >= 8 fps, yellow >= 4, red below — same convention as
k10-fast-corners; the SPI flush is expected to be the same ~9fps ceiling
there, since drawing here is a similar full-frame blit plus lightweight
per-point line/marker stamps (TBD, measure).

## Open items / next steps

- **Nothing here has run on real footage yet.** Priority 1 before trusting
  any of the tuning constants (`LKT_MIN_EIG`, `LKT_MAX_MEAN_SSD`,
  `MIN_SPACING`, `REPLENISH_FLOOR/PERIOD`): flash, point at a real scene,
  and watch whether trails look stable on real corners and get dropped on
  genuinely lost ones. `tools/capture.py` + the `d`/`D` dump commands are
  wired up for pulling a real frame if any of this needs offline tuning,
  same workflow as k10-fast-corners.
  - **Predicted failure mode if `LKT_MIN_EIG` is still too strict for real
    footage despite the mean-per-pixel fix**: active count jumps to ~40
    right after a replenish, collapses toward 0 within a frame or two (the
    gate fires the first time each new track is evaluated), replenish
    refires because count < `REPLENISH_FLOOR`, repeat — markers flicker in
    and out and trails never grow past one or two points, while fps stays
    steady around 8 because none of this shows up as a performance problem.
    The serial stats line's `lost(eig=.. ssd=.. bounds=..)` counters are
    the fast way to confirm this is what's happening (`eig` dominating)
    versus a different bug; `[`/`]` lower the gate live to check without a
    reflash.
- Forward-backward error check, if lost/false-track behavior on real
  footage warrants it (see "Not implemented (v1)" above).
- Track full-res LK instead of half-res, if k10-fast-corners's own open
  item ("full-res detection is now affordable") lands first — this module
  is resolution-agnostic (`lkt_pyramid_build` takes whatever level-0 image
  it's handed), so that would be a `main.cpp` change only, not a `lktrack`
  change.
- WiFi/UDP streaming of tracks + accel is now in (build-verified — see below);
  **device verification is the next step**, including whether the earlier
  starvation symptom (see Status above) shows up again now that the camera
  init is back to the plain, proven `register_camera()` config. **DONE
  2026-09-17** — see "WiFi + camera bug" above for the internal-SRAM
  exhaustion bug this surfaced and its fix.

## Streaming to a host for visual odometry (added 2026-09-17, device-verified:
fps=7, TRCK+ACCL streaming live with the on-device preview running)

Same decision as k10-fast-corners for the same reasons (WiFi UDP; unicast to
`STREAM_HOST_IP`, because subnet broadcast measured DOA on this network — read
that project's PLAN.md section before concluding a silent receiver means broken
firmware). Only the track payload is new; `k10stream_wifi_begin()` and the
accelerometer path are reused verbatim, and the camera/`register_camera()`
setup is untouched.

**Two streams, both `#pragma pack(push,1)` fixed-layout little-endian:**

| Packet | Port | Bytes | Fields |
|--------|------|-------|--------|
| `k10stream_trck_hdr_t` | 5007 | 18 | magic "TRCK", seq u32, t_us u32, frame_w u16, frame_h u16, n u16 |
| `k10stream_trck_track_t` x n | 5007 | 10 each | x_q4 i16, y_q4 i16, id u16, age u16, score u16 |
| `k10stream_accel_t` | 5006 | 18 | magic "ACCL", seq u32, t_us u32, ax/ay/az i16 |

- **frame_w/frame_h are pyramid level 0 (120x160)** — half the camera's, the
  same convention as the ORBF stream, so one host consumer handles both feature
  sources in the same coordinate system (x2 for full-res/display coords).
- **Positions are Q4 fixed-point (px x 16)**, not whole pixels: sub-pixel LK
  output is the entire product here, and int16 px would discard it. 3.25 px ->
  52. int16 range is +-2047 px, far more than the 120x160 frame needs.
- `score` is **always 0** — `lktrack` exposes no per-track quality value (it
  computes a structure tensor per point per level but retains no score). The
  field is reserved in the layout so wiring one in later doesn't change the
  wire format.
- Sent from the pipeline task **after tracking and after replenishing**, so each
  packet is the frame's complete live set (survivors plus freshly seeded
  points), not the survivors alone.
- Truncation: `K10STREAM_TRACK_MAX_TRACKS` (100) holds the packet to
  18 + 100*10 = 1018 B, under the ~1472 B non-fragmenting UDP ceiling, with
  headroom to raise `LKT_MAX_TRACKS` (40 today) without touching the format.
  Unlike ORBF there is no strongest-first ranking to preserve here, so tracks
  go out in track-array order and the cap is purely a safety net.
- `k10stream_send_tracks()` builds the datagram in one `static` scratch buffer
  before writing it, because WiFiUDP's TX buffer is append-only and the
  header's `n` isn't knowable until the tracks have been walked. ORB's sender
  doesn't need that — its `n` is a plain array length known up front.
- `ACCL` stays on **5006**, deliberately the same port k10-fast-corners uses so
  one host receiver listens identically whichever firmware is flashed; TRCK
  takes 5007 because 5005 is ORBF's.
- Non-fatal: a WiFi failure leaves `g_wifi_up` false and tracker/display run
  exactly as before. The 1 Hz serial line gained `wifi=<ip|down> udp_ok=<n>
  udp_fail=<n>` — the same sent-counter trick that untangled k10-fast-corners'
  stream diagnosis, so "nothing at the host" separates from "nothing sent"
  without a debugger.
- Host side: `../../experiments/visual-odometry/recv_server.py` decodes
  TRCK/ORBF/ACCL and serves live stats (tracks, corners, accel, msg/s,
  frame/s) at http://localhost:8080.

## Screen on/off while streaming

The full-frame blit + `lv_task_handler()` pass is the same ~75 ms/frame
SPI-flush ceiling k10-fast-corners measures (TBD, measure here too), so the
preview and the frame rate compete for one budget — and VO wants fps. `'s'`
over serial, or holding A **or** B >= `LONG_PRESS_MS` (600 ms), clears
`g_screen_on`: the pipeline then skips the `memcpy`, the marker/trail stamps
and the canvas entirely, having painted one last opaque "SCREEN OFF /
streaming..." frame, and makes no LVGL calls at all until toggled back on.
Detection, LK and both UDP streams are untouched by the toggle, so this is
purely a throughput knob for a host-driven session. Toggling back on calls
`clearLocalCanvas()` first — that OFF frame is an opaque full-screen rect on
the canvas layer, which sits above `feed_img`, and without restoring
transparency the rest of it stays stuck over the live feed (same trap, same
fix as k10-fast-corners).

### Host test (`test/trck_wire_test.cpp`) — the wire format itself

```
c++ -O2 -std=c++17 -Wall -I ../../lib/lktrack -I ../../lib/orb \
   -I ../../lib/k10stream test/trck_wire_test.cpp -o /tmp/trckt && /tmp/trckt
```

`#define K10STREAM_NO_WIFI` before including `k10stream.h` (it then
forward-declares `WiFiUDP`/`IPAddress` instead of pulling the Arduino headers)
so the packed structs can be checked on a laptop. Asserts hdr=18 B, track=10 B,
that a full `LKT_MAX_TRACKS` set fits in 1472 B, and that the byte offsets match
the `<4sIIHHH` / `<hhHHH` format strings the Python receiver unpacks — silent
C/Python layout drift is the failure mode this exists to catch.

## Files

- `../../lib/lktrack/` — pyramidal LK tracker (new; this project's own module)
- `../../lib/fastcorner/` — FAST-9 + Shi-Tomasi corner detector (shared with
  k10-fast-corners; reused unmodified here as the seed source)
- `../../lib/k10image/` — RGB565 -> luma (byte-swap fix + BT.601 weighting)
- `../../lib/k10stream/` — wire format + senders (ORBF/TRCK/ACCL), shared with
  k10-fast-corners
- `src/main.cpp` — setup/buttons, pipeline task, display, UDP streaming
- `test/lk_test.cpp` — off-device tracker tests (see above)
- `test/trck_wire_test.cpp` — off-device TRCK byte-layout test (see above)
- `tools/capture.py` — pull a frame off the board as a PNG (USB-CDC; copied
  from k10-fast-corners, same tool). Note: it decodes raw RGB565 naively
  (native `uint16_t` read) — the sensor's bytes are byte-swapped on the wire
  (see `k10-fast-corners/PLAN.md`'s "byte-swap" gotcha), so `--raw` dumps come
  out with a wrong-looking color cast even on a healthy capture. That's a
  known quirk of this debug tool, not evidence of frame corruption; the real
  display path (`memcpy` into `dispBuf`) already expects that native byte
  order and renders correctly.
- `../../experiments/visual-odometry/serial_probe.sh` — one-shot serial boot
  probe: hard-resets the board via RTS, captures boot + first N seconds of
  serial, and reports a starvation-warning count + fps-line verdict. This is
  the tool that found and confirmed the "WiFi + camera bug" fix above —
  reach for it before guessing at any future camera/WiFi timing issue on
  this board.
- `platformio.ini` — same proven env as k10-fast-corners, WiFi included
- `../../experiments/visual-odometry/recv_server.py` — host receiver + live stats
  web page (no dependencies beyond the stdlib)

## Build / flash

`WIFI_SSID`, `WIFI_PASSWORD` and `STREAM_HOST_IP` (the VO host's dotted-quad
IP) are read from the shell environment at build time — `platformio.ini`'s
`build_flags` uses `${sysenv.*}`, so they are never committed to source. Export
them before building; `STREAM_HOST_IP` must be the laptop's *current* address
on the K10's WiFi (`ipconfig getifaddr en0`):

```
export WIFI_SSID="Fallyn"
export WIFI_PASSWORD="..."                  # see shell profile
export STREAM_HOST_IP="172.16.35.211"       # this laptop, on that network

pio run
pio run -t upload     # USB-C; press RST if needed
pio device monitor
```
