// orb.cpp — ORB feature extraction: orientation + BRIEF-256 descriptor.
//
// Two static tables are built once, on first call:
//
//   s_umax[v]      — half-width of a radius-ORB_PATCH_RADIUS circle at row
//                    offset v, used to keep the orientation moment sum inside
//                    a circular (not square) patch. Pure geometry
//                    (round(sqrt(r*r - v*v))), not a learned/copyrighted table.
//   s_pattern[256] — the 256 BRIEF test-point pairs, rejection-sampled from
//                    an isotropic Gaussian so every point lands within radius
//                    ORB_PATCH_RADIUS (this is what keeps the *rotated*
//                    pattern in-bounds too, since rotation preserves vector
//                    length). This is the ORB paper's untrained G-I baseline
//                    pattern, not OpenCV's greedily-selected one — there is
//                    no dependency on OpenCV source here.
//
// Per keypoint: compute the intensity-centroid angle over the circular patch,
// then rotate all 256 pattern pairs by that angle and threshold the two
// sampled pixels into one bit each — "steered BRIEF".

#include "orb.h"

#include <math.h>
#include <string.h>

typedef struct { int8_t x1, y1, x2, y2; } orb_pair_t;

#define ORB_NPAIRS 256

static int8_t     s_umax[ORB_PATCH_RADIUS + 1];
static orb_pair_t s_pattern[ORB_NPAIRS];
static int         s_init_done = 0;

// Small, deterministic PRNG (xorshift32) so the pattern is identical on every
// run/device — matching descriptors requires both sides to have built the
// same pattern, and a fixed seed is the simplest way to guarantee that.
static uint32_t s_rng_state = 88172645463325252u & 0xFFFFFFFFu;

static inline uint32_t xorshift32(void) {
    uint32_t x = s_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng_state = x;
    return x;
}

// Uniform double in [0, 1).
static inline double rnd_unit(void) {
    return (double) xorshift32() / 4294967296.0;
}

// One standard-normal sample (Box-Muller).
static inline double rnd_gauss(void) {
    double u1 = rnd_unit(), u2 = rnd_unit();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

// One point within radius r, isotropic Gaussian with sigma = r/3, rejection
// sampled to guarantee |point| <= r exactly (so a rotation of it by any
// angle also stays within radius r of the keypoint).
static void sample_point(int r, int8_t *ox, int8_t *oy) {
    const double sigma = r / 3.0;
    for (int tries = 0; tries < 64; tries++) {
        const double x = rnd_gauss() * sigma;
        const double y = rnd_gauss() * sigma;
        if (x * x + y * y <= (double) (r * r)) {
            *ox = (int8_t) lround(x);
            *oy = (int8_t) lround(y);
            return;
        }
    }
    *ox = 0; *oy = 0;  // pathological: fall back to the center (never fires in practice)
}

static void orb_init(void) {
    if (s_init_done) return;
    s_init_done = 1;

    const int r = ORB_PATCH_RADIUS;
    for (int v = 0; v <= r; v++) {
        const double u = sqrt((double) (r * r - v * v));
        s_umax[v] = (int8_t) lround(u);
    }

    for (int i = 0; i < ORB_NPAIRS; i++) {
        sample_point(r, &s_pattern[i].x1, &s_pattern[i].y1);
        sample_point(r, &s_pattern[i].x2, &s_pattern[i].y2);
    }
}

// Intensity-centroid orientation (Rosin) over the circular patch of radius
// ORB_PATCH_RADIUS centered at (cx, cy). angle = atan2(m01, m10).
static float ic_angle(const uint8_t *gray, int w, int cx, int cy) {
    int32_t m01 = 0, m10 = 0;

    // Center row (v = 0) contributes to m10 only.
    {
        const int u = s_umax[0];
        const uint8_t *row = gray + (size_t) cy * w + cx;
        for (int u2 = -u; u2 <= u; u2++) m10 += u2 * row[u2];
    }

    for (int v = 1; v <= ORB_PATCH_RADIUS; v++) {
        const int u = s_umax[v];
        const uint8_t *rowp = gray + (size_t) (cy + v) * w + cx;
        const uint8_t *rowm = gray + (size_t) (cy - v) * w + cx;
        int32_t sum_p = 0, sum_m = 0;
        for (int u2 = -u; u2 <= u; u2++) {
            const int ip = rowp[u2], im = rowm[u2];
            m10 += u2 * (ip + im);
            sum_p += ip;
            sum_m += im;
        }
        m01 += v * (sum_p - sum_m);
    }

    return atan2f((float) m01, (float) m10);
}

int orb_compute(const uint8_t *gray, int w, int h, int cx, int cy, orb_feature_t *out) {
    orb_init();

    const int r = ORB_PATCH_RADIUS;
    if (cx < r + 1 || cx >= w - r - 1 || cy < r + 1 || cy >= h - r - 1) return -1;

    const float angle = ic_angle(gray, w, cx, cy);
    const float cosA = cosf(angle), sinA = sinf(angle);

    memset(out->desc, 0, ORB_DESC_BYTES);
    for (int i = 0; i < ORB_NPAIRS; i++) {
        const orb_pair_t &p = s_pattern[i];

        const int rx1 = (int) lroundf(p.x1 * cosA - p.y1 * sinA);
        const int ry1 = (int) lroundf(p.x1 * sinA + p.y1 * cosA);
        const int rx2 = (int) lroundf(p.x2 * cosA - p.y2 * sinA);
        const int ry2 = (int) lroundf(p.x2 * sinA + p.y2 * cosA);

        const int v1 = gray[(size_t) (cy + ry1) * w + (cx + rx1)];
        const int v2 = gray[(size_t) (cy + ry2) * w + (cx + rx2)];

        if (v1 < v2) out->desc[i >> 3] |= (uint8_t) (1u << (i & 7));
    }

    out->angle = angle;
    out->x = (int16_t) cx;
    out->y = (int16_t) cy;
    return 0;
}

int orb_compute_batch(const uint8_t *gray, int w, int h,
                      const int16_t *xs, const int16_t *ys, int n,
                      orb_feature_t *out, int max_out) {
    int written = 0;
    for (int i = 0; i < n && written < max_out; i++) {
        if (orb_compute(gray, w, h, xs[i], ys[i], &out[written]) == 0) written++;
    }
    return written;
}

int orb_hamming(const uint8_t a[ORB_DESC_BYTES], const uint8_t b[ORB_DESC_BYTES]) {
    int dist = 0;
    for (int i = 0; i < ORB_DESC_BYTES; i++)
        dist += __builtin_popcount((unsigned) (a[i] ^ b[i]));
    return dist;
}
