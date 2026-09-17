// lk_test.cpp — build & run the pyramidal LK tracker on synthetic frames,
// off-device.
//
//   c++ -O2 -std=c++17 -I ../../../lib/lktrack \
//      lk_test.cpp ../../../lib/lktrack/lktrack.cpp -o /tmp/lkt && /tmp/lkt
//
// Same trick as orb_test.cpp: a continuous template function sampled at each
// pixel's coordinates (shifted by a known translation), so a rendered frame
// has no rasterization/interpolation artifact of its own to confound the
// thing being tested — any error measured is the tracker's, not the fixture's.

#include "lktrack.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

static int g_fail = 0;

static void check(bool ok, const std::string &what, const std::string &detail = "") {
    std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(),
                detail.empty() ? "" : " — ", detail.c_str());
    if (!ok) g_fail++;
}

// A blob of a few soft Gaussian bumps — has real structure (non-degenerate
// gradient in more than one direction) unlike a single edge or flat field,
// so it passes the min-eig corner gate.
static double blob_template(double u, double v) {
    double val = 40.0;
    static const double cx[] = {0, 12, -9, 5};
    static const double cy[] = {0, -8, 10, 14};
    static const double amp[] = {160, 90, 110, 70};
    for (int k = 0; k < 4; k++) {
        const double dx = u - cx[k], dy = v - cy[k];
        val += amp[k] * std::exp(-(dx * dx + dy * dy) / (2.0 * 5.0 * 5.0));
    }
    return val > 255.0 ? 255.0 : val;
}

static std::vector<uint8_t> render(int w, int h, double ox, double oy, double shift_x, double shift_y) {
    std::vector<uint8_t> img(w * h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const double u = (x - ox) - shift_x;
            const double v = (y - oy) - shift_y;
            img[y * w + x] = (uint8_t) std::lround(blob_template(u, v));
        }
    }
    return img;
}

struct Pyr {
    std::vector<uint8_t> l0, l1, l2;
    lkt_pyramid_t p;
};

static void build(Pyr &py, const std::vector<uint8_t> &lvl0, int w, int h) {
    py.l0 = lvl0;
    py.l1.assign((w / 2) * (h / 2), 0);
    py.l2.assign((w / 4) * (h / 4), 0);
    lkt_pyramid_build(&py.p, py.l0.data(), w, h, py.l1.data(), py.l2.data());
}

// Track a single point from frame `a` at (px,py) into frame `b`, return the
// tracker's estimate of where it landed (or false if lost).
static bool track_point(int w, int h, double ox, double oy,
                        double shift_x, double shift_y, float px, float py,
                        float *out_x, float *out_y) {
    auto fa = render(w, h, ox, oy, 0, 0);
    auto fb = render(w, h, ox, oy, shift_x, shift_y);
    Pyr pa, pb;
    build(pa, fa, w, h);
    build(pb, fb, w, h);

    lkt_state_t st;
    lkt_init(&st);
    const int16_t xs[1] = {(int16_t) std::lround(px)};
    const int16_t ys[1] = {(int16_t) std::lround(py)};
    check(lkt_add(&st, xs, ys, 1, 4) == 1, "seed point added");
    lkt_track(&st, &pa.p, &pb.p);

    if (!st.tracks[0].active) return false;
    *out_x = st.tracks[0].x;
    *out_y = st.tracks[0].y;
    return true;
}

int main() {
    // Large enough that a coarsest-level (1/4 scale) patch around the shifted
    // point stays clear of the image edge even for the 40px "huge" shift
    // below — otherwise a lost track there would conflate "hit the edge" with
    // "exceeded capture range", which is the thing that test wants to isolate.
    const int W = 160, H = 160;
    const double OX = 80, OY = 80;

    std::printf("integer translation\n");
    {
        float x, y;
        const bool ok = track_point(W, H, OX, OY, 3.0, -2.0, 80, 80, &x, &y);
        check(ok, "track survives a (3,-2) shift");
        if (ok) {
            const double ex = std::fabs(x - 83.0), ey = std::fabs(y - 78.0);
            check(ex < 0.3 && ey < 0.3, "recovers within 0.3px",
                  "got (" + std::to_string(x) + "," + std::to_string(y) + ")");
        }
    }

    std::printf("subpixel translation\n");
    {
        float x, y;
        const bool ok = track_point(W, H, OX, OY, 1.5, 0.5, 80, 80, &x, &y);
        check(ok, "track survives a (1.5,0.5) shift");
        if (ok) {
            const double ex = std::fabs(x - 81.5), ey = std::fabs(y - 80.5);
            check(ex < 0.3 && ey < 0.3, "recovers within 0.3px (pins bilinear sign/convention)",
                  "got (" + std::to_string(x) + "," + std::to_string(y) + ")");
        }
    }

    std::printf("large translation (needs pyramid capture range)\n");
    {
        float x, y;
        const bool ok = track_point(W, H, OX, OY, 12.0, 0.0, 80, 80, &x, &y);
        check(ok, "track survives a 12px shift with 3 pyramid levels");
        if (ok) {
            const double ex = std::fabs(x - 92.0), ey = std::fabs(y - 80.0);
            check(ex < 1.0 && ey < 1.0, "recovers within 1px",
                  "got (" + std::to_string(x) + "," + std::to_string(y) + ")");
        }
    }

    std::printf("huge translation -> must be marked lost, not silently wrong\n");
    {
        float x, y;
        const bool ok = track_point(W, H, OX, OY, 40.0, 0.0, 80, 80, &x, &y);
        check(!ok, "40px shift exceeds capture range and is dropped");
    }

    std::printf("flat field -> seed rejected by min-eig gate\n");
    {
        std::vector<uint8_t> flat(W * H, 100);
        Pyr pa, pb;
        build(pa, flat, W, H);
        build(pb, flat, W, H);
        lkt_state_t st;
        lkt_init(&st);
        const int16_t xs[1] = {(int16_t) OX};
        const int16_t ys[1] = {(int16_t) OY};
        lkt_add(&st, xs, ys, 1, 4);
        lkt_track(&st, &pa.p, &pb.p);
        check(!st.tracks[0].active, "flat patch fails the min-eigenvalue gate and is dropped");
    }

    std::printf("multi-point + replenish spacing\n");
    {
        lkt_state_t st;
        lkt_init(&st);
        const int16_t xs[3] = {20, 22, 40};   // first two within min_spacing of each other
        const int16_t ys[3] = {20, 21, 40};
        const int added = lkt_add(&st, xs, ys, 3, 4);
        check(added == 2, "second candidate rejected as too close to the first",
              "added=" + std::to_string(added));
        check(lkt_active_count(&st) == 2, "active count matches");
    }

    std::printf("\n%s (%d failure%s)\n", g_fail ? "FAILED" : "ALL PASS",
                g_fail, g_fail == 1 ? "" : "s");
    return g_fail != 0;
}
