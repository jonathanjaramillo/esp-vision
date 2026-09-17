// lktrack.h — pyramidal Lucas-Kanade sparse point tracker (Bouguet-style).
//
// Given a small set of tracked points and a 2-frame image pyramid (previous,
// current), refines each point's position with iterative LK from the
// coarsest level down to the finest. The expensive per-point work (the
// iterative solve) only ever touches a small patch around each point — nine
// pixels wide by default — never the whole frame; only pyramid *construction*
// touches every pixel once, and that's a fixed ~1-2ms shared cost independent
// of how many points are tracked (see PLAN.md for the trade-off against a
// pyramid built locally per point, which is both aliased and, above a
// handful of points, more total work).
//
// Resolution-agnostic: this module knows nothing about the camera or the
// detector. Level 0 of the pyramid can be a full-res or downsampled luma
// image — whatever the caller already has lying around (e.g. the detector's
// own half-res blurred buffer, reused as level 0 for free).
#pragma once

#include <stdint.h>
#include <stddef.h>

#define LKT_PYR_LEVELS   3      // level 0 (caller-supplied) + 2 built levels
#define LKT_MAX_TRACKS   40
#define LKT_TRAIL_LEN    8      // ring buffer of recent positions, for drawing
#define LKT_PATCH_RADIUS 4      // LK window is (2r+1)x(2r+1) = 9x9
#define LKT_MAX_ITERS    8
#define LKT_EPS2         0.03f  // stop iterating once |d|^2 (px^2, this level) is below this

// Gate on the min eigenvalue of the 2x2 structure tensor G, *mean-per-pixel*
// (i.e. G / patch-pixel-count) so the constant means "mean squared gradient"
// and doesn't have to be re-derived if LKT_PATCH_RADIUS changes. Accumulated
// over the template patch at the finest level (level 0) — a corner-like
// patch has two large eigenvalues; an edge has one large and one near-zero,
// and a flat patch has two near-zero. Using min-eig rather than det(G)
// matters: an edge's (large, ~0) pair can still pass a determinant gate at
// a big enough "large" eigenvalue, and those are exactly the points that
// silently slide along the edge instead of getting rejected.
//
// UNVERIFIED ON REAL FOOTAGE — this default is a rough estimate, not a
// measurement: k10-fast-corners/PLAN.md measures the same blurred half-res
// buffer's background at mean |neighbour difference| ~1.57 gray levels, two
// orders of magnitude gentler than any synthetic fixture in test/lk_test.cpp
// can cheaply cover. Use lkt_set_gates() to tune this at runtime (no
// reflash) against real footage before trusting it — see PLAN.md "Open
// items". If real tracks die immediately after every replenish, this is the
// first thing to lower.
#define LKT_MIN_EIG      15.0f

// Gate on mean squared residual (T - I)^2 over the template patch after
// convergence, at level 0 — catches points that converged to the wrong
// place (occlusion, deformation, or a genuinely lost feature). Same
// unverified-on-real-footage caveat as LKT_MIN_EIG above.
#define LKT_MAX_MEAN_SSD 400.0f  // (20 gray-level RMS residual)^2

typedef struct {
    const uint8_t *data[LKT_PYR_LEVELS];  // data[0] is caller-owned; 1.. built by lkt_pyramid_build
    int16_t        w[LKT_PYR_LEVELS];
    int16_t        h[LKT_PYR_LEVELS];
} lkt_pyramid_t;

// Build levels 1..LKT_PYR_LEVELS-1 by 2x2 box-downsampling level 0 into the
// caller-supplied scratch buffers (each must hold at least
// ceil(w0/2)*ceil(h0/2), ceil(w0/4)*ceil(h0/4) ... bytes — for LKT_PYR_LEVELS
// == 3 that's exactly two buffers, `l1` and `l2`).
//
// level0 should already be reasonably smooth (a blurred detector buffer is
// ideal) — LK's gradients get noisy on raw sensor pixels, same reason
// fast_blur3x3 exists.
void lkt_pyramid_build(lkt_pyramid_t *pyr, const uint8_t *level0, int w0, int h0,
                       uint8_t *l1, uint8_t *l2);

typedef struct {
    float   x, y;                          // current position, level-0 pixel coords
    int16_t trail_x[LKT_TRAIL_LEN];        // ring buffer of recent level-0 positions
    int16_t trail_y[LKT_TRAIL_LEN];
    uint8_t trail_head;                    // next slot to write
    uint8_t trail_count;                   // valid entries, saturates at LKT_TRAIL_LEN
    uint16_t id;
    uint16_t age;                          // frames tracked
    uint8_t  active;
} lkt_track_t;

typedef struct {
    lkt_track_t tracks[LKT_MAX_TRACKS];
    uint16_t    next_id;
} lkt_state_t;

void lkt_init(lkt_state_t *st);

// Override the LKT_MIN_EIG / LKT_MAX_MEAN_SSD compile-time defaults at
// runtime — both are unverified-on-real-footage estimates (see lktrack.h
// above), so bring-up on a real device needs to tune them without a
// reflash cycle. Pass a negative value to leave that gate unchanged.
void lkt_set_gates(float min_eig, float max_mean_ssd);

typedef struct {
    int lost_eig;     // dropped by the min-eigenvalue gate
    int lost_ssd;      // dropped by the residual gate
    int lost_bounds;   // dropped: level 0 ran out of frame margin
} lkt_lost_stats_t;

// Refine every active track from `prev` to `cur` (both full pyramids, same
// dimensions at every level). Only level 0 (the finest) decides lost/alive:
// a track that runs out of frame margin there, or fails the min-eigenvalue
// or residual gate there, is deactivated (active = 0) and not written to
// the trail. A coarser level running out of margin (or hitting a singular
// structure tensor) just skips that level's refinement instead — gating on
// every level was tried and measured to fail almost every freshly-seeded
// point near the frame edge, because a fixed patch margin is a much bigger
// fraction of a coarse level's tiny image than of level 0 (see PLAN.md).
// Surviving tracks get their position updated and a new trail entry
// appended.
//
// Call once per frame after lkt_pyramid_build has produced `cur`; skip on
// the very first frame (there is no `prev` yet). `stats`, if non-NULL, is
// incremented (not reset) per lost track this call — the only way to tell
// "the eigenvalue gate is miscalibrated" from "genuinely lost tracks" from
// a device's serial line without a debugger attached.
void lkt_track(lkt_state_t *st, const lkt_pyramid_t *prev, const lkt_pyramid_t *cur,
              lkt_lost_stats_t *stats = NULL);

// Fill inactive track slots from a strongest-first candidate list (level-0
// pixel coords, e.g. straight out of fast_corner_detect — no rescaling
// needed if level 0 of the pyramid IS the detector's own buffer). A
// candidate is skipped if it is within `min_spacing` (Chebyshev) of any
// currently-active track, so replenishment doesn't pile new points on top of
// ones already being tracked. Returns the number of tracks added.
int lkt_add(lkt_state_t *st, const int16_t *xs, const int16_t *ys, int n,
           int min_spacing);

int lkt_active_count(const lkt_state_t *st);
