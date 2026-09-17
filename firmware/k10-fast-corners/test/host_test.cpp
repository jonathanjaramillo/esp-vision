// host_test.cpp — build & run the detector on synthetic frames, off-device.
//
//   c++ -O2 -std=c++17 -I src test/host_test.cpp src/fastcorner.cpp -o /tmp/fct && /tmp/fct
//
// PLAN.md's rule: a white square must give exactly 4 corners, a plain edge and
// a flat field must give 0. Added here: the regression that motivated the
// Shi-Tomasi ranking — an edge must never out-rank a corner — and a check that
// the RGB565 luma path no longer manufactures cliffs out of smooth colour ramps.

#include "fastcorner.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>
#include <string>

static int g_fail = 0;

static void check(bool ok, const char *what, const std::string &detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what,
                detail.empty() ? "" : " — ", detail.c_str());
    if (!ok) g_fail++;
}

// --- the luma path under test, copied verbatim from main.cpp ---------------
// Note the bswap: the camera frame is stored byte-swapped RGB565 (see main.cpp).
// The fixtures below therefore feed byte-swapped values too, so that what is
// being varied really is the red field.
static inline uint8_t rgb565_luma(uint16_t p) {
    p = __builtin_bswap16(p);
    const uint16_t r5 = (p >> 11) & 0x1F;
    const uint16_t g6 = (p >> 5) & 0x3F;
    const uint16_t b5 = p & 0x1F;
    const uint16_t v = (uint16_t)(r5 * 79 + g6 * 76 + b5 * 30) >> 5;
    return (v > 255) ? (uint8_t) 255 : (uint8_t) v;
}
// the version this replaced, for the comparison the fix rests on
static inline uint8_t rgb565_luma_old(uint16_t p) {
    const uint8_t r5 = (p >> 11) & 0x1F, g6 = (p >> 5) & 0x3F, b5 = p & 0x1F;
    const uint8_t r8 = (r5 << 3) | (r5 << 1) | (r5 >> 2);
    const uint8_t g8 = (g6 << 2) | (g6 >> 3);
    const uint8_t b8 = (b5 << 3) | (b5 << 1) | (b5 >> 2);
    const uint16_t v = (uint16_t)(r8 * 5 + g8 * 2 + b8) >> 3;
    return (v > 255) ? (uint8_t) 255 : (uint8_t) v;
}

static const int W = 64, H = 64;

static int near_count(const fc_result_t &r, int x, int y, int tol) {
    int n = 0;
    for (int i = 0; i < r.count; i++)
        if (std::abs(r.corners[i].x - x) <= tol && std::abs(r.corners[i].y - y) <= tol) n++;
    return n;
}

int main() {
    std::vector<uint8_t> img(W * H);
    fc_result_t r;

    // 1. flat field -> nothing
    std::printf("flat field\n");
    std::memset(img.data(), 128, img.size());
    fast_corner_detect(img.data(), W, H, 24, 1, &r);
    check(r.count == 0, "no corners on a flat field");

    // 2. vertical step edge -> nothing (this is the FAST-9-vs-half-arc rule)
    std::printf("straight vertical edge\n");
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) img[y * W + x] = (x < 32) ? 40 : 210;
    fast_corner_detect(img.data(), W, H, 24, 1, &r);
    check(r.count == 0, "no corners on a straight edge",
          r.count ? "edge is being detected as corners" : "");

    // 3. white square -> exactly its 4 corners
    std::printf("white square on black\n");
    std::memset(img.data(), 0, img.size());
    for (int y = 20; y < 44; y++)
        for (int x = 20; x < 44; x++) img[y * W + x] = 255;
    fast_corner_detect(img.data(), W, H, 24, 1, &r);
    check(r.count == 4, "exactly 4 corners",
          r.count != 4 ? "got " + std::to_string(r.count) : "");
    bool all4 = near_count(r,20,20,2) && near_count(r,43,20,2)
             && near_count(r,20,43,2) && near_count(r,43,43,2);
    check(all4, "one marker at each square corner");

    // 4. THE REGRESSION that motivated Shi-Tomasi ranking: a long, high-contrast
    //    staircase edge (what RGB565 banding on a flat wall looks like once it
    //    has been aliased) running past a lower-contrast real square. FAST fires
    //    on every jog of the staircase — the detector is not wrong to. The point
    //    is that none of those hits may out-rank, or crowd out, a genuine corner.
    //
    //    The band is kept a constant width: an earlier version of this fixture
    //    narrowed to a point and the wedge tip was legitimately the sharpest
    //    corner in the frame, which is a property of the fixture, not a bug.
    std::printf("staircase edge vs. a lower-contrast square\n");
    std::memset(img.data(), 60, img.size());
    for (int y = 0; y < H; y++) {                   // near-vertical 1 px staircase
        const int left = 8 + (y / 3);
        for (int x = left; x < left + 10; x++) img[y * W + x] = 200;  // 140 contrast
    }
    for (int y = 40; y < 58; y++)                   // real corners, only 90 contrast
        for (int x = 40; x < 58; x++) img[y * W + x] = 150;

    fast_corner_detect(img.data(), W, H, 24, 1, &r);
    check(r.candidates > 20, "the staircase does generate many raw FAST hits",
          "candidates=" + std::to_string(r.candidates));

    int sq = near_count(r, 40, 40, 3) + near_count(r, 57, 40, 3)
           + near_count(r, 40, 57, 3) + near_count(r, 57, 57, 3);
    check(sq == 4, "all 4 corners of the softer square survive",
          "got " + std::to_string(sq) + " of 4");

    int on_stair = 0;
    for (int i = 0; i < r.count; i++) {
        const int left = 8 + (r.corners[i].y / 3);
        if (r.corners[i].x >= left - 2 && r.corners[i].x <= left + 11) on_stair++;
    }
    check(on_stair == 0, "no staircase hit is emitted at all",
          "emitted " + std::to_string(on_stair) + " of " +
          std::to_string(r.candidates) + " raw hits, total count=" +
          std::to_string(r.count));

    // 5. luma: a smooth RGB565 red ramp must stay smooth
    std::printf("RGB565 red ramp -> luma\n");
    int mx_new = 0, mx_old = 0;
    for (int v = 0; v < 31; v++) {
        const uint16_t lo = __builtin_bswap16((uint16_t)(v << 11));
        const uint16_t hi = __builtin_bswap16((uint16_t)((v + 1) << 11));
        const int d_new = std::abs((int) rgb565_luma(hi)     - (int) rgb565_luma(lo));
        const int d_old = std::abs((int) rgb565_luma_old((uint16_t)((v + 1) << 11))
                                 - (int) rgb565_luma_old((uint16_t)(v << 11)));
        if (d_new > mx_new) mx_new = d_new;
        if (d_old > mx_old) mx_old = d_old;
    }
    std::printf("  max luma step per red LSB: old=%d new=%d (t default=%d)\n",
                mx_old, mx_new, FC_THRESHOLD_DEFAULT);
    check(mx_new < FC_THRESHOLD_MIN, "a 1-LSB red step stays under the lowest usable threshold");
    check(mx_old >= FC_THRESHOLD_DEFAULT, "the old path did exceed the default threshold (the bug)");

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILED" : "ALL PASS",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail != 0;
}
