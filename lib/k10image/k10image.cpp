#include "k10image.h"

#include <cstddef>

// RGB565 -> 8-bit luma, BT.601 (0.299 R, 0.587 G, 0.114 B), weighting the raw
// 5/6/5 fields directly so there is no bit-expansion step at all.
//
// Two bugs lived here and together they were the main source of the "corners
// march along the colour contours of a flat wall" symptom:
//
//  1. The 5->8 expansion `(v<<3)|(v<<1)|(v>>2)` is NOT monotonic. Measured over
//     v = 0..31 its step ranges from -17 to +55 (a correct `(v<<3)|(v>>2)` steps
//     8 or 9 every time). A smooth red or blue ramp across a wall came out of it
//     as a sawtooth with 55-level cliffs that sometimes ran *backwards* —
//     synthetic step edges with jagged, FAST-friendly jogs on them.
//  2. The 5:2:1 weights put 5/8 of the luma on red, the channel with the
//     coarsest quantisation, and only 2/8 on green, the finest. That made a
//     single red LSB worth up to 23 luma levels — right through t=24 — and
//     turned pure chroma gradients into apparent luma edges.
//
// Worst-case luma jump from a 1-LSB channel change: was R=23.1 G=1.2 B=4.6,
// now R=2.7 G=2.9 B=1.0.
//
// Coefficients: 0.299 * (255/31) * 32 ~= 79, 0.587 * (255/63) * 32 ~= 76,
// 0.114 * (255/31) * 32 ~= 30. Max sum 8167 >> 5 = 255 (clamped).
uint8_t rgb565_luma(uint16_t p) {
    // The camera frame is stored BYTE-SWAPPED RGB565 (big-endian on the wire),
    // which is what the ILI9341/LVGL path wants — the display memcpy's it raw
    // and looks perfectly correct, which is exactly why this hid for so long.
    // But the ESP32 is little-endian, so reading it as a uint16_t scrambles the
    // channels. With V = RRRRRGGG GGGBBBBB stored as bytes [b0, b1], a native
    // read yields N = b1<<8 | b0, and decoding N gives:
    //     "red"   = G2G1G0 B4B3      (green's low bits + blue's high bits)
    //     "green" = B2B1B0 R4R3R2
    //     "blue"  = R1R0 G5G4G3
    // i.e. low-order colour bits land in high-order luma positions, so any
    // small colour change produces a huge luma jump. That is what put hard
    // banded "contours" all over flat surfaces and gave FAST its false corners.
    //
    // Measured on a real frame off the device: mean |neighbour difference|
    // 17.71 scrambled vs 5.32 correct. (An earlier "the GC2145 feed is noisy,
    // mean |nd| 15-19" figure WAS this bug, not sensor noise.)
    p = __builtin_bswap16(p);

    const uint16_t r5 = (p >> 11) & 0x1F;
    const uint16_t g6 = (p >> 5) & 0x3F;
    const uint16_t b5 = p & 0x1F;
    const uint16_t v = (uint16_t)(r5 * 79 + g6 * 76 + b5 * 30) >> 5;
    return (v > 255) ? (uint8_t) 255 : (uint8_t) v;
}

void k10_to_grayscale(const uint16_t *rgb, uint8_t *gray, int n) {
    for (int i = 0; i < n; i++) gray[i] = rgb565_luma(rgb[i]);
}

void rotate_qvga_to_portrait(const uint16_t *src, uint16_t *dst) {
    const int w = 240, h = 320;
    for (int y = 0; y < h; ++y) {
        uint16_t *d = dst + (size_t) y * w;
        for (int x = 0; x < w; ++x)
            d[x] = src[(size_t) (w - 1 - x) * h + y];
    }
}
