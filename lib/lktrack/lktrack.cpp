#include "lktrack.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Pyramid construction — the only step that touches every pixel of the
// frame; everything else in this file only ever visits a small patch per
// tracked point. LKT_PYR_LEVELS is hardcoded to 3 (level 0 + these two built
// levels) — see lktrack.h.
// ---------------------------------------------------------------------------

static void downsample2x2(const uint8_t *src, uint8_t *dst, int w, int h) {
    const int dw = w / 2, dh = h / 2;
    for (int y = 0; y < dh; y++) {
        const uint8_t *r0 = src + (size_t) (2 * y) * w;
        const uint8_t *r1 = src + (size_t) (2 * y + 1) * w;
        uint8_t *d = dst + (size_t) y * dw;
        for (int x = 0; x < dw; x++) {
            const int sx = 2 * x;
            d[x] = (uint8_t) ((r0[sx] + r0[sx + 1] + r1[sx] + r1[sx + 1] + 2) >> 2);
        }
    }
}

void lkt_pyramid_build(lkt_pyramid_t *pyr, const uint8_t *level0, int w0, int h0,
                       uint8_t *l1, uint8_t *l2) {
    pyr->data[0] = level0;
    pyr->w[0] = (int16_t) w0;
    pyr->h[0] = (int16_t) h0;

    const int w1 = w0 / 2, h1 = h0 / 2;
    downsample2x2(level0, l1, w0, h0);
    pyr->data[1] = l1;
    pyr->w[1] = (int16_t) w1;
    pyr->h[1] = (int16_t) h1;

    const int w2 = w1 / 2, h2 = h1 / 2;
    downsample2x2(l1, l2, w1, h1);
    pyr->data[2] = l2;
    pyr->w[2] = (int16_t) w2;
    pyr->h[2] = (int16_t) h2;
}

// ---------------------------------------------------------------------------
// Per-point, per-level iterative LK. Everything below only touches a
// (2r+3)x(2r+3) patch (r = LKT_PATCH_RADIUS) around one point.
// ---------------------------------------------------------------------------

#define LKT_N (2 * LKT_PATCH_RADIUS + 1)

static float s_min_eig      = LKT_MIN_EIG;
static float s_max_mean_ssd = LKT_MAX_MEAN_SSD;

void lkt_set_gates(float min_eig, float max_mean_ssd) {
    if (min_eig >= 0.0f) s_min_eig = min_eig;
    if (max_mean_ssd >= 0.0f) s_max_mean_ssd = max_mean_ssd;
}

static inline float bilerp(const uint8_t *img, int w, int x0, int y0, float fx, float fy) {
    const uint8_t *p = img + (size_t) y0 * w + x0;
    const float v00 = p[0], v01 = p[1], v10 = p[w], v11 = p[w + 1];
    const float v0 = v00 + (v01 - v00) * fx;
    const float v1 = v10 + (v11 - v10) * fx;
    return v0 + (v1 - v0) * fy;
}

// x, y are assumed within [0, w-1] x [0, h-1] already (callers keep every
// sample inside the bounds-checked margin) — clamped defensively anyway.
static inline float sample(const uint8_t *img, int w, int h, float x, float y) {
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    const float xmax = (float) (w - 1) - 1e-3f;
    const float ymax = (float) (h - 1) - 1e-3f;
    if (x > xmax) x = xmax;
    if (y > ymax) y = ymax;
    const int x0 = (int) x, y0 = (int) y;
    return bilerp(img, w, x0, y0, x - (float) x0, y - (float) y0);
}

static inline bool in_bounds(float x, float y, int w, int h, float margin) {
    return x >= margin && x <= (float) (w - 1) - margin &&
           y >= margin && y <= (float) (h - 1) - margin;
}

// Refines (*gx, *gy) — the caller's initial guess at this pyramid level — by
// iterative LK against a template extracted from `prevImg` around (px, py).
// Template and gradients (and so the structure tensor G) come from `prevImg`
// only, computed once; each iteration resamples `curImg` at the current
// guess. This is what keeps iterations cheap and is also what makes the sign
// of the residual matter: b = sum(grad * (T - I)), not (I - T) — get that
// backwards and the solve diverges instead of converging.
//
// out_min_eig / out_mean_ssd are optional (pass NULL to skip the extra
// residual pass) — only the finest level needs them for the lost gate.
// Returns false if the point (or, mid-iteration, its updated guess) leaves
// the bounds needed for a full patch + gradient margin — the caller treats
// that as lost.
static bool track_one_level(const uint8_t *prevImg, const uint8_t *curImg, int w, int h,
                            float px, float py, float *gx, float *gy,
                            float *out_min_eig, float *out_mean_ssd) {
    const int r = LKT_PATCH_RADIUS;
    const float margin = (float) (r + 1);
    if (!in_bounds(px, py, w, h, margin)) return false;

    float T[LKT_N][LKT_N], Ix[LKT_N][LKT_N], Iy[LKT_N][LKT_N];
    float Gxx = 0, Gxy = 0, Gyy = 0;
    for (int j = -r; j <= r; j++) {
        for (int i = -r; i <= r; i++) {
            const float cx = px + (float) i, cy = py + (float) j;
            const float l  = sample(prevImg, w, h, cx, cy);
            const float rt = sample(prevImg, w, h, cx + 1.0f, cy);
            const float lt = sample(prevImg, w, h, cx - 1.0f, cy);
            const float dn = sample(prevImg, w, h, cx, cy + 1.0f);
            const float up = sample(prevImg, w, h, cx, cy - 1.0f);
            const float ix = (rt - lt) * 0.5f;
            const float iy = (dn - up) * 0.5f;
            T[j + r][i + r]  = l;
            Ix[j + r][i + r] = ix;
            Iy[j + r][i + r] = iy;
            Gxx += ix * ix;
            Gxy += ix * iy;
            Gyy += iy * iy;
        }
    }

    const float trace = Gxx + Gyy;
    const float det    = Gxx * Gyy - Gxy * Gxy;
    if (out_min_eig) {
        const float disc = sqrtf(trace * trace - 4.0f * det > 0.0f ? trace * trace - 4.0f * det : 0.0f);
        // Mean-per-pixel, not summed over the patch — see lktrack.h.
        *out_min_eig = 0.5f * (trace - disc) / (float) (LKT_N * LKT_N);
    }

    // A near-singular G (flat or texture-less patch at this level) has no
    // reliable solve — leave the guess as the propagated prediction rather
    // than divide by ~0. This only matters at coarse levels; a degenerate
    // level 0 is caught by the min-eig gate the caller applies there.
    if (det > 1e-3f) {
        const float igxx =  Gyy / det, igxy = -Gxy / det, igyy = Gxx / det;
        for (int it = 0; it < LKT_MAX_ITERS; it++) {
            if (!in_bounds(*gx, *gy, w, h, margin)) return false;
            float bx = 0, by = 0;
            for (int j = -r; j <= r; j++) {
                for (int i = -r; i <= r; i++) {
                    const float ival = sample(curImg, w, h, *gx + (float) i, *gy + (float) j);
                    const float e = T[j + r][i + r] - ival;
                    bx += Ix[j + r][i + r] * e;
                    by += Iy[j + r][i + r] * e;
                }
            }
            const float ddx = igxx * bx + igxy * by;
            const float ddy = igxy * bx + igyy * by;
            *gx += ddx;
            *gy += ddy;
            if (ddx * ddx + ddy * ddy < LKT_EPS2) break;
        }
        if (!in_bounds(*gx, *gy, w, h, margin)) return false;
    }

    if (out_mean_ssd) {
        float ssd = 0;
        for (int j = -r; j <= r; j++) {
            for (int i = -r; i <= r; i++) {
                const float ival = sample(curImg, w, h, *gx + (float) i, *gy + (float) j);
                const float e = T[j + r][i + r] - ival;
                ssd += e * e;
            }
        }
        *out_mean_ssd = ssd / (float) (LKT_N * LKT_N);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Public state API
// ---------------------------------------------------------------------------

void lkt_init(lkt_state_t *st) {
    memset(st, 0, sizeof(*st));
    st->next_id = 1;
}

void lkt_track(lkt_state_t *st, const lkt_pyramid_t *prev, const lkt_pyramid_t *cur,
              lkt_lost_stats_t *stats) {
    for (int ti = 0; ti < LKT_MAX_TRACKS; ti++) {
        lkt_track_t *tr = &st->tracks[ti];
        if (!tr->active) continue;

        const float px_l0 = tr->x, py_l0 = tr->y;
        float guess_x = 0, guess_y = 0;   // propagated from the previous (coarser) level
        float final_x = 0, final_y = 0;
        bool lost = false;
        int lost_reason = 0;   // 0 = bounds/singular, 1 = eig, 2 = ssd

        for (int L = LKT_PYR_LEVELS - 1; L >= 0; L--) {
            const float scale = 1.0f / (float) (1 << L);
            const float px = px_l0 * scale, py = py_l0 * scale;
            float lx = (L == LKT_PYR_LEVELS - 1) ? px : guess_x;
            float ly = (L == LKT_PYR_LEVELS - 1) ? py : guess_y;

            float min_eig = -1.0f, mean_ssd = -1.0f;
            const bool ok = track_one_level(prev->data[L], cur->data[L],
                                            prev->w[L], prev->h[L], px, py, &lx, &ly,
                                            (L == 0) ? &min_eig : NULL,
                                            (L == 0) ? &mean_ssd : NULL);

            if (L == 0) {
                // Only the finest level's outcome decides lost/alive. A
                // fixed patch margin is a much bigger fraction of a coarse
                // level's tiny image than of level 0 (a 5px margin is ~17%
                // of a 30px-wide coarsest level but ~4% of 120px-wide level
                // 0), so gating on every level's bounds check made points
                // seeded anywhere near the frame edge die on the very next
                // frame, regardless of the eigenvalue/residual gates below
                // — measured live: bounds losses climbing almost as fast as
                // points were added. See PLAN.md.
                if (!ok) { lost = true; lost_reason = 0; break; }
                if (min_eig < s_min_eig) { lost = true; lost_reason = 1; break; }
                if (mean_ssd > s_max_mean_ssd) { lost = true; lost_reason = 2; break; }
                final_x = lx;
                final_y = ly;
            } else {
                // Out of bounds or singular G at a coarse level: skip this
                // level's refinement (propagate the unrefined prediction)
                // rather than lose the track — the finer levels below still
                // get a chance to refine it once it's back in a workable
                // region, same tolerance already given to a singular G.
                guess_x = (ok ? lx : px) * 2.0f;
                guess_y = (ok ? ly : py) * 2.0f;
            }
        }

        if (lost) {
            tr->active = 0;
            if (stats) {
                if (lost_reason == 0) stats->lost_bounds++;
                else if (lost_reason == 1) stats->lost_eig++;
                else stats->lost_ssd++;
            }
            continue;
        }

        tr->x = final_x;
        tr->y = final_y;
        tr->trail_x[tr->trail_head] = (int16_t) lroundf(final_x);
        tr->trail_y[tr->trail_head] = (int16_t) lroundf(final_y);
        tr->trail_head = (uint8_t) ((tr->trail_head + 1) % LKT_TRAIL_LEN);
        if (tr->trail_count < LKT_TRAIL_LEN) tr->trail_count++;
        tr->age++;
    }
}

int lkt_add(lkt_state_t *st, const int16_t *xs, const int16_t *ys, int n, int min_spacing) {
    int added = 0;
    for (int c = 0; c < n; c++) {
        int slot = -1;
        for (int i = 0; i < LKT_MAX_TRACKS; i++) {
            if (!st->tracks[i].active) { slot = i; break; }
        }
        if (slot < 0) break;  // no room left

        bool ok = true;
        for (int i = 0; i < LKT_MAX_TRACKS; i++) {
            if (!st->tracks[i].active) continue;
            const int dx = abs((int) xs[c] - (int) lroundf(st->tracks[i].x));
            const int dy = abs((int) ys[c] - (int) lroundf(st->tracks[i].y));
            if (dx < min_spacing && dy < min_spacing) { ok = false; break; }
        }
        if (!ok) continue;

        lkt_track_t *t = &st->tracks[slot];
        memset(t, 0, sizeof(*t));
        t->x = (float) xs[c];
        t->y = (float) ys[c];
        t->trail_x[0] = xs[c];
        t->trail_y[0] = ys[c];
        t->trail_head = 1 % LKT_TRAIL_LEN;
        t->trail_count = 1;
        t->id = st->next_id++;
        t->age = 0;
        t->active = 1;
        added++;
    }
    return added;
}

int lkt_active_count(const lkt_state_t *st) {
    int n = 0;
    for (int i = 0; i < LKT_MAX_TRACKS; i++) if (st->tracks[i].active) n++;
    return n;
}
