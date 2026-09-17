// offline_detect.cpp — run the exact on-device pipeline over a captured frame.
//
//   c++ -O2 -std=c++17 -I src tools/offline_detect.cpp src/fastcorner.cpp src/orb.cpp -o /tmp/od
//   /tmp/od frame.bin out.ppm [threshold]
//
// Takes a raw RGB565 dump from tools/capture.py, runs luma -> 2x2 -> blur ->
// detect with the same code the firmware runs, and writes the colour frame with
// the corners marked. This is the loop that makes threshold/scoring tuning free:
// one capture, then as many detector runs as you like with no reflash.
//
// Also runs ORB (orientation + BRIEF-256 descriptor, see src/orb.h) on every
// detected corner and prints each descriptor plus the nearest-neighbour
// Hamming distance to every other corner — a quick sanity check of whether
// the descriptors on a real frame are actually distinctive before wiring
// matching into the firmware.
#include "fastcorner.h"
#include "orb.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
using namespace std;

static const int W = 240, H = 320, DW = W / 2, DH = H / 2;

static inline unsigned char luma(unsigned short p) {
    p = __builtin_bswap16(p);                     // see main.cpp
    unsigned r5 = (p >> 11) & 0x1F, g6 = (p >> 5) & 0x3F, b5 = p & 0x1F;
    unsigned v = (r5 * 79 + g6 * 76 + b5 * 30) >> 5;
    return v > 255 ? 255 : (unsigned char) v;
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s frame.bin out.ppm [t]\n", argv[0]); return 2; }
    const int t = (argc > 3) ? atoi(argv[3]) : FC_THRESHOLD_DEFAULT;

    vector<unsigned char> buf(W * H * 2);
    FILE *f = fopen(argv[1], "rb");
    if (!f || fread(buf.data(), 1, buf.size(), f) != buf.size()) { fprintf(stderr, "bad input\n"); return 1; }
    fclose(f);

    vector<unsigned char> gray(W * H), det(DW * DH), tmp(DW * DH);
    for (int i = 0; i < W * H; i++)
        gray[i] = luma((unsigned short)(buf[2 * i] | (buf[2 * i + 1] << 8)));
    fast_downsample2x2(gray.data(), det.data(), W, H);
    fast_blur3x3(det.data(), tmp.data(), DW, DH);

    fc_result_t r;
    fast_corner_detect(det.data(), DW, DH, t, 1, &r);
    printf("t=%d  candidates=%d  corners=%d  pool_full=%d\n", t, r.candidates, r.count, r.pool_full);

    // ORB on every corner, run on the same half-res blurred image the
    // detector just scored (det, after fast_blur3x3 above).
    vector<int16_t> xs(r.count), ys(r.count);
    for (int i = 0; i < r.count; i++) { xs[i] = r.corners[i].x; ys[i] = r.corners[i].y; }
    vector<orb_feature_t> feats(r.count);
    const int nf = orb_compute_batch(det.data(), DW, DH, xs.data(), ys.data(), r.count,
                                     feats.data(), r.count);
    printf("orb: %d/%d corners got a descriptor (rest too close to the border)\n", nf, r.count);
    for (int i = 0; i < nf; i++) {
        int nn = -1, nn_dist = 257;
        for (int j = 0; j < nf; j++) {
            if (j == i) continue;
            const int d = orb_hamming(feats[i].desc, feats[j].desc);
            if (d < nn_dist) { nn_dist = d; nn = j; }
        }
        printf("  #%2d (%3d,%3d) angle=%6.1fdeg  nearest=#%-2d dist=%d/256\n",
              i, feats[i].x, feats[i].y, feats[i].angle * 180.0 / M_PI, nn, nn_dist);
    }

    // render: true colour + magenta plus markers at the same coords main.cpp stamps
    vector<unsigned char> px(W * H * 3);
    for (int i = 0; i < W * H; i++) {
        unsigned short v = __builtin_bswap16((unsigned short)(buf[2 * i] | (buf[2 * i + 1] << 8)));
        unsigned r5 = (v >> 11) & 0x1F, g6 = (v >> 5) & 0x3F, b5 = v & 0x1F;
        px[3 * i] = (r5 << 3) | (r5 >> 2);
        px[3 * i + 1] = (g6 << 2) | (g6 >> 4);
        px[3 * i + 2] = (b5 << 3) | (b5 >> 2);
    }
    for (int i = 0; i < r.count; i++) {
        const int cx = r.corners[i].x * 2, cy = r.corners[i].y * 2;
        for (int o = -2; o <= 2; o++) {
            for (int k = 0; k < 2; k++) {
                const int x = k ? cx : cx + o, y = k ? cy + o : cy;
                if (x < 0 || x >= W || y < 0 || y >= H) continue;
                const int b = (y * W + x) * 3;
                px[b] = 255; px[b + 1] = 0; px[b + 2] = 255;
            }
        }
    }
    FILE *o = fopen(argv[2], "wb");
    fprintf(o, "P6\n%d %d\n255\n", W, H);
    fwrite(px.data(), 1, px.size(), o);
    fclose(o);
    printf("wrote %s\n", argv[2]);
    return 0;
}
