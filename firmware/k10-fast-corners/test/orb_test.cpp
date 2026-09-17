// orb_test.cpp — build & run the ORB orientation/descriptor code on synthetic
// patches, off-device.
//
//   c++ -O2 -std=c++17 -I src test/orb_test.cpp src/orb.cpp -o /tmp/ot && /tmp/ot
//
// What's checked:
//   1. determinism — same patch, same call, same descriptor
//   2. border rejection — a keypoint without a full patch margin is refused
//   3. rotation (near-)invariance — a patch generated as a rotated view of the
//      same continuous template gets a similar descriptor at every angle
//   4. distinctiveness — two unrelated patches land near the expected ~128/256
//      random-descriptor Hamming distance, not near 0

#include "orb.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

static int g_fail = 0;

static void check(bool ok, const char *what, const std::string &detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what,
                detail.empty() ? "" : " — ", detail.c_str());
    if (!ok) g_fail++;
}

static const int W = 64, H = 64;

// A continuous, rotationally-*asymmetric* template evaluated directly in
// float patch coordinates (u, v) — a wedge, bright on one side of a line
// through the origin at a shallow angle. Sampling this at the *inverse*
// rotation of each pixel's coordinates produces an alias-free rotated view,
// unlike rotating an already-rasterized image (which reintroduces the exact
// interpolation artifacts we don't want to test against).
static uint8_t wedge_template(double u, double v) {
    return (u > 0 && v > -0.3 * u) ? 210 : 60;
}

// Render a WxH image where the wedge template is rotated by `theta` radians
// and centered at (cx, cy).
static std::vector<uint8_t> render_rotated(int cx, int cy, double theta) {
    std::vector<uint8_t> img(W * H, 90);
    const double c = std::cos(-theta), s = std::sin(-theta);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const double dx = x - cx, dy = y - cy;
            const double u = dx * c - dy * s;
            const double v = dx * s + dy * c;
            img[y * W + x] = wedge_template(u, v);
        }
    }
    return img;
}

int main() {
    // 1. determinism
    std::printf("determinism\n");
    auto img0 = render_rotated(32, 32, 0.0);
    orb_feature_t f1, f2;
    check(orb_compute(img0.data(), W, H, 32, 32, &f1) == 0, "orb_compute succeeds in-bounds");
    check(orb_compute(img0.data(), W, H, 32, 32, &f2) == 0, "orb_compute succeeds again");
    check(std::memcmp(f1.desc, f2.desc, ORB_DESC_BYTES) == 0, "identical call -> identical descriptor");

    // 2. border rejection
    std::printf("border rejection\n");
    orb_feature_t fb;
    check(orb_compute(img0.data(), W, H, 2, 2, &fb) == -1, "corner near (0,0) is refused (no patch margin)");
    check(orb_compute(img0.data(), W, H, W - 2, H - 2, &fb) == -1, "corner near (W,H) is refused");
    check(orb_compute(img0.data(), W, H, ORB_PATCH_RADIUS + 1, 32, &fb) == 0, "exactly at the margin succeeds");

    // 3. rotation (near-)invariance
    std::printf("rotation invariance\n");
    orb_feature_t base;
    orb_compute(img0.data(), W, H, 32, 32, &base);
    const double angles_deg[] = {15, 30, 45, 60, 90, 135, 180, 250};
    int worst = 0;
    for (double deg : angles_deg) {
        const double theta = deg * M_PI / 180.0;
        auto img = render_rotated(32, 32, theta);
        orb_feature_t f;
        if (orb_compute(img.data(), W, H, 32, 32, &f) != 0) { check(false, "compute failed"); continue; }
        const int dist = orb_hamming(base.desc, f.desc);
        if (dist > worst) worst = dist;
        std::printf("  theta=%5.1f deg  angle_est_delta=%6.1f deg  hamming=%3d/256\n",
                    deg, (f.angle - base.angle) * 180.0 / M_PI, dist);
    }
    // Untrained steered-BRIEF at a 9px patch is not going to be perfect —
    // this just needs to be much better than the ~128 random baseline.
    check(worst < 90, "descriptor stays well below the random baseline (~128) across rotation",
          "worst=" + std::to_string(worst));

    // 4. distinctiveness: an unrelated flat-ish patch vs the wedge
    std::printf("distinctiveness\n");
    std::vector<uint8_t> flat(W * H);
    uint32_t seed = 12345;
    for (auto &p : flat) {
        seed = seed * 1664525u + 1013904223u;
        p = (uint8_t) (100 + (int) ((seed >> 24) % 40));  // mild noise, no structure
    }
    orb_feature_t fflat;
    orb_compute(flat.data(), W, H, 32, 32, &fflat);
    const int dist_unrelated = orb_hamming(base.desc, fflat.desc);
    std::printf("  wedge vs. noise: hamming=%d/256\n", dist_unrelated);
    check(dist_unrelated > 40, "unrelated patches are far apart in Hamming distance",
          "dist=" + std::to_string(dist_unrelated));

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILED" : "ALL PASS",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail != 0;
}
