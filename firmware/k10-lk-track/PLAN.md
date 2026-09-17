# k10-lk-track — PLAN

FAST-9 corner detection + pyramidal Lucas-Kanade sparse tracking on the
UNIHIKER K10, with live trails drawn over the camera feed.

**Status: host tests pass; not yet flashed/verified on the device.** Fields
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
   6. draw: raw frame + per-track trail + marker -> dispBuf
   7. swap cur/prev pyramid index (ping-pong, no copy)
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
- Local display only, no WiFi/UDP (unlike k10-fast-corners) — this is a pure
  on-device test of the tracker; add streaming later if something needs to
  consume tracks off-device.

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
| A+B   | toggle trail drawing on/off (markers stay either way) |
| `d` (serial) | dump the 120x160 pyramid-level-0 luma |
| `D` (serial) | dump the raw 240x320 RGB565 |
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
- No WiFi streaming yet — add a UDP stream of track id/position (reusing
  `lib/k10stream`'s wire-format pattern) if something downstream wants to
  consume live tracks, same as the ORB feature stream in k10-fast-corners.

## Files

- `../../lib/lktrack/` — pyramidal LK tracker (new; this project's own module)
- `../../lib/fastcorner/` — FAST-9 + Shi-Tomasi corner detector (shared with
  k10-fast-corners; reused unmodified here as the seed source)
- `../../lib/k10image/` — RGB565 -> luma (byte-swap fix + BT.601 weighting)
- `src/main.cpp` — setup/buttons, pipeline task, display
- `test/lk_test.cpp` — off-device tracker tests (see above)
- `tools/capture.py` — pull a frame off the board as a PNG (USB-CDC; copied
  from k10-fast-corners, same tool)
- `platformio.ini` — same proven env as k10-fast-corners, minus WiFi

## Build / flash

```
pio run
pio run -t upload     # USB-C; press RST if needed
pio device monitor
```
