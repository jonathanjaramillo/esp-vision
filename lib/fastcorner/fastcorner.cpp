// fastcorner.cpp — FAST-9 corner detector (Rosten & Drummond), full ring.
//
// A pixel is a corner when 9 *contiguous* pixels of the radius-3 circle-16 are
// all brighter (or all darker) than the center +/- threshold. The 16 ring
// pixels are folded into two 16-bit polarity masks; a duplicated-mask AND chain
// answers the circular 9-run test in O(1) word ops.
//
// Selection: FAST proposes, Shi-Tomasi disposes (the ORB arrangement). Every
// raw ring hit is collected, scored by the Shi-Tomasi min-eigenvalue of the
// local structure tensor, ranked strongest-first, and emitted under Chebyshev
// suppression.
//
// The FAST arc score (smallest center-to-ring difference over the winning
// 9-run) is NOT used for ranking: it cannot tell a corner from an edge with a
// one-pixel jog in it, and an edge has far more pixels than a corner does. At
// FC_MIN_SPACING 4 a single 100 px contour can host ~25 detections, so a few
// strong lines ate the whole 96-corner budget and genuine corners never got
// drawn — the "markers trace lines along flat walls" symptom. Shi-Tomasi
// min(l1, l2) is near zero wherever the gradient has one dominant direction,
// so edges rank below corners no matter how contrasty they are.
//
// Ranking before suppression matters for a second reason — a plain scan-order
// cap fills the first few rows (only ~60 slots fit per row at a 4 px spacing)
// and hides the rest of the frame.
//
// Speed: the whole frame is scanned (no early bail), so the inner loop must be
// cheap. Ring addressing uses 16 per-row base pointers (no per-pixel multiply),
// a safe 4-point pre-check rejects most pixels after 4 loads, and the strength
// score is only computed for pixels that actually pass.

#include "fastcorner.h"

#include <stddef.h>
#include <stdlib.h>

// The radius-3 Bresenham circle-16, in circular order, with dy positive =
// down (image row-major). ring_corner hard-codes these offsets as (row, dx)
// pairs: bit i of the polarity mask must correspond to ring position i so
// that "9 contiguous set bits" == "9 contiguous ring pixels".
//
//   i :  0    1    2    3    4    5    6    7
//  dy : -3   -3   -2   -1    0    1    2    3
//  dx :  0    1    2    3    3    3    2    1
//   i :  8    9   10   11   12   13   14   15
//  dy :  3    3    2    1    0   -1   -2   -3
//  dx :  0   -1   -2   -3   -3   -3   -2   -1

// Start index (0..15) of a circular run of >= 9 set bits in the 16-bit mask
// m, or -1 if there is none.
static inline __attribute__((always_inline)) int run9_start(uint32_t m) {
    const uint32_t d = m | (m << 16);
    uint32_t r = d;
    r &= d >> 1;
    r &= d >> 2;
    r &= d >> 3;
    r &= d >> 4;
    r &= d >> 5;
    r &= d >> 6;
    r &= d >> 7;
    r &= d >> 8;
    return r ? __builtin_ctz(r) : -1;
}

// r[0..6] are the row base pointers for y-3 .. y+3. All 16 ring pixels are
// loaded with compile-time constant offsets off those seven pointers, so the
// inner loop has no per-pixel multiply and no pointer-array indirection.
// Returns 1 for a corner. Strength is not computed here — see shi_tomasi().
static inline __attribute__((always_inline)) int ring_corner(const uint8_t *const r[7],
                              int x, int t) {
    const int c = r[3][x];
    const int above = c + t > 255 ? 255 : c + t;
    const int below = c - t < 0 ? 0 : c - t;

    // Ring positions in circular order (index -> row, dx).
    const int v0 = r[0][x + 0],  v1 = r[0][x + 1],  v2 = r[1][x + 2],  v3 = r[2][x + 3];
    const int v4 = r[3][x + 3],  v5 = r[4][x + 3],  v6 = r[5][x + 2],  v7 = r[6][x + 1];
    const int v8 = r[6][x + 0],  v9 = r[6][x - 1], v10 = r[5][x - 2], v11 = r[4][x - 3];
    const int v12 = r[3][x - 3], v13 = r[2][x - 3], v14 = r[1][x - 2], v15 = r[0][x - 1];

    // Cheap, *lossless* pre-check: any 9-run on 16 positions contains at least
    // two of the four compass points (indices 0/4/8/12, spaced 4 apart). So if
    // neither polarity has 2 of them, the pixel cannot be a corner — reject
    // after 4 loads instead of 16. This can never drop a real corner.
    int nb = (v0 > above) + (v4 > above) + (v8 > above) + (v12 > above);
    int nd = (v0 < below) + (v4 < below) + (v8 < below) + (v12 < below);
    if (nb < 2 && nd < 2) return 0;

    const int vv[16] = { v0, v1, v2, v3, v4, v5, v6, v7,
                         v8, v9, v10, v11, v12, v13, v14, v15 };
    uint32_t mb = 0, md = 0;
    for (int i = 0; i < 16; i++) {
        mb |= (uint32_t)(vv[i] > above) << i;
        md |= (uint32_t)(vv[i] < below) << i;
    }

    return run9_start(mb) >= 0 || run9_start(md) >= 0;
}

// Shi-Tomasi cornerness: the smaller eigenvalue of the structure tensor
//
//     M = sum over the window of [ Ix*Ix  Ix*Iy ]
//                                [ Ix*Iy  Iy*Iy ]
//
// summed over a 5x5 window (10x10 of the original frame at half-res detection),
// with Ix, Iy from central differences. min(l1, l2) is large only when the
// gradient turns inside the window: an edge, however contrasty, has one near-zero
// eigenvalue and scores near zero, while a corner scores high.
//
// Costs ~5.6k ops per candidate and only runs on pixels FAST already accepted,
// so at a few thousand candidates it is a couple of ms — affordable against the
// ~70 ms SPI flush that actually caps the frame rate.
//
// Requires x, y at least 3 px from the border (2 for the window, 1 for the
// central difference); fast_corner_detect's scan window is inset by 4.
static float shi_tomasi(const uint8_t *gray, int w, int x, int y) {
    int32_t sxx = 0, syy = 0, sxy = 0;

    for (int dy = -2; dy <= 2; dy++) {
        const uint8_t *row = gray + (size_t)(y + dy) * w + x;
        for (int dx = -2; dx <= 2; dx++) {
            const uint8_t *p = row + dx;
            const int32_t ix = (int32_t) p[1] - (int32_t) p[-1];
            const int32_t iy = (int32_t) p[w] - (int32_t) p[-w];
            sxx += ix * ix;
            syy += iy * iy;
            sxy += ix * iy;
        }
    }

    // min eigenvalue = (a + c)/2 - sqrt(((a - c)/2)^2 + b^2)
    const float a = (float) sxx, c = (float) syy, b = (float) sxy;
    const float half_sum  = 0.5f * (a + c);
    const float half_diff = 0.5f * (a - c);
    return half_sum - __builtin_sqrtf(half_diff * half_diff + b * b);
}

// Candidate pool: every raw FAST hit, ranked before suppression. 4096 * 8 B =
// 32 KB of .bss. Overflow drops the scan-order *tail*, which reintroduces the
// top-of-frame bias ranking exists to remove, so fc_result_t reports both the
// raw hit count and whether the pool saturated — if it does, raise the
// threshold rather than living with a half-scanned frame.
#define FC_MAX_CAND 4096

typedef struct {
    int16_t x;
    int16_t y;
    float   score;
} fc_cand_t;

static fc_cand_t s_cand[FC_MAX_CAND];

static int cand_cmp_desc(const void *a, const void *b) {
    const fc_cand_t *pa = (const fc_cand_t *) a;
    const fc_cand_t *pb = (const fc_cand_t *) b;
    return (pb->score > pa->score) - (pb->score < pa->score);
}
// Separable 3x3 box blur, in place: horizontal pass gray -> tmp, then
// vertical pass tmp -> gray. Runs over the whole frame in two streaming
// passes (edges clamp to the border pixel).
void fast_blur3x3(uint8_t *gray, uint8_t *tmp, int w, int h) {
    for (int y = 0; y < h; y++) {
        const uint8_t *s = gray + (size_t) y * w;
        uint8_t *t = tmp + (size_t) y * w;
        t[0] = (uint8_t)((2 * s[0] + s[1] + 1) / 3);
        for (int x = 1; x < w - 1; x++)
            t[x] = (uint8_t)((s[x - 1] + s[x] + s[x + 1] + 1) / 3);
        t[w - 1] = (uint8_t)((s[w - 2] + 2 * s[w - 1] + 1) / 3);
    }
    for (int x = 0; x < w; x++)
        gray[x] = (uint8_t)((2 * tmp[x] + tmp[w + x] + 1) / 3);
    for (int y = 1; y < h - 1; y++) {
        const uint8_t *t0 = tmp + (size_t)(y - 1) * w;
        const uint8_t *t1 = t0 + w;
        const uint8_t *t2 = t1 + w;
        uint8_t *d = gray + (size_t) y * w;
        for (int x = 0; x < w; x++)
            d[x] = (uint8_t)((t0[x] + t1[x] + t2[x] + 1) / 3);
    }
    {
        const uint8_t *t0 = tmp + (size_t)(h - 2) * w;
        const uint8_t *t1 = t0 + w;
        uint8_t *d = gray + (size_t)(h - 1) * w;
        for (int x = 0; x < w; x++)
            d[x] = (uint8_t)((t0[x] + 2 * t1[x] + 1) / 3);
    }
}

// 2x2 box downsample (see header). Purely a spatial average, so a large-object
// corner stays a corner while 1-2 pixel detail is averaged away.
void fast_downsample2x2(const uint8_t *src, uint8_t *dst, int w, int h) {
    const int w2 = w / 2;
    for (int y = 0; y < h / 2; y++) {
        const uint8_t *s0 = src + (size_t)(2 * y) * w;
        const uint8_t *s1 = s0 + w;
        uint8_t *d = dst + (size_t) y * w2;
        for (int x = 0; x < w2; x++)
            d[x] = (uint8_t)((s0[2 * x] + s0[2 * x + 1] + s1[2 * x] + s1[2 * x + 1] + 2) >> 2);
    }
}


void fast_corner_detect(const uint8_t *gray, int w, int h,
                        int threshold, int stride, fc_result_t *out) {
    out->count = 0;
    out->candidates = 0;
    out->pool_full = 0;

    if (threshold < FC_THRESHOLD_MIN) threshold = FC_THRESHOLD_MIN;
    if (threshold > FC_THRESHOLD_MAX) threshold = FC_THRESHOLD_MAX;
    if (stride < 1) stride = 1;

    // The ring extends +/-3 and shi_tomasi's window +/-3, so inset by 4 —
    // one extra pixel of margin costs nothing and keeps both in bounds.
    const int y0 = 4, y1 = h - 4;
    const int x0 = 4, x1 = w - 4;

    const int t = threshold;
    int ncand = 0;

    for (int y = y0; y < y1; y += stride) {
        // Row base pointers for y-3 .. y+3; ring_corner indexes them with
        // constant offsets, so there is no per-pixel multiply.
        const uint8_t *r[7];
        r[0] = gray + (size_t)(y - 3) * w;
        for (int k = 1; k < 7; k++) r[k] = r[k - 1] + w;

        for (int x = x0; x < x1; x += stride) {
            if (!ring_corner(r, x, t)) continue;

            out->candidates++;
            if (ncand < FC_MAX_CAND) {
                s_cand[ncand].x = (int16_t) x;
                s_cand[ncand].y = (int16_t) y;
                s_cand[ncand].score = shi_tomasi(gray, w, x, y);
                ncand++;
            } else {
                out->pool_full = 1;
            }
        }
    }

    // Strongest first, then spatial suppression. Picking by Shi-Tomasi score
    // (rather than scan order, or the FAST arc score) is what keeps markers on
    // real corners and spread over the frame instead of strung along edges.
    qsort(s_cand, (size_t) ncand, sizeof(s_cand[0]), cand_cmp_desc);

    for (int i = 0; i < ncand && out->count < FC_MAX_CORNERS; i++) {
        const int x = s_cand[i].x;
        const int y = s_cand[i].y;

        int dup = 0;
        for (int j = 0; j < out->count; j++) {
            int dxp = x - out->corners[j].x; if (dxp < 0) dxp = -dxp;
            int dyp = y - out->corners[j].y; if (dyp < 0) dyp = -dyp;
            if (dxp < FC_MIN_SPACING && dyp < FC_MIN_SPACING) { dup = 1; break; }
        }
        if (dup) continue;

        out->corners[out->count].x = (int16_t) x;
        out->corners[out->count].y = (int16_t) y;
        out->count++;
    }
}
