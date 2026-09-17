// fastcorner.h — FAST-9 corner detector (Rosten & Drummond).
//
// A pixel is a corner when 9 *contiguous* pixels of the radius-3 circle-16
// are all brighter (or all darker) than the center +/- threshold. The 16
// ring points are sampled once and folded into two 16-bit polarity masks,
// so the 9-consecutive check is a circular 9-bit run over a 16-bit word
// (answered by a duplicated-mask AND chain), leaving the inner loop as
// loads, compares and a handful of bit ops per pixel.
#pragma once

#include <stdint.h>

#define FC_MAX_CORNERS 96      // hard cap on reported corners
#define FC_MIN_SPACING 4       // Chebyshev suppression radius (px)

#define FC_THRESHOLD_MIN   8
#define FC_THRESHOLD_MAX  96
#define FC_THRESHOLD_DEFAULT 24

typedef struct {
    int16_t x;
    int16_t y;
} fc_corner_t;

typedef struct {
    fc_corner_t corners[FC_MAX_CORNERS];
    int         count;
    int         candidates;  // raw FAST hits before ranking/suppression (diagnostic)
    int         pool_full;   // 1 = candidate pool saturated, tail dropped in scan
                             //     order (raise the threshold; see fastcorner.cpp)
} fc_result_t;

// Detect corners in an 8-bit grayscale image of w*h bytes (row-major).
//
// FAST-9 proposes candidates; each is then scored by its Shi-Tomasi
// min-eigenvalue and the pool is ranked strongest-first before suppression, so
// edges (which FAST fires on wherever they have a one-pixel jog, and which have
// far more pixels than corners do) rank below genuine corners.
//
// threshold : FAST-9 t in [FC_THRESHOLD_MIN, FC_THRESHOLD_MAX] (values clamped)
// stride    : scan stride, 1 = every pixel, 2 = quarter the pixels (fast)
// out       : filled; corners within FC_MIN_SPACING of an already-emitted,
//             stronger corner are dropped, and count is capped at FC_MAX_CORNERS
void fast_corner_detect(const uint8_t *gray, int w, int h,
                        int threshold, int stride, fc_result_t *out);

// Separable 3x3 box blur of an 8-bit grayscale image, in place.
//
// FAST-9 differences a center against a radius-3 ring, so sensor noise passes
// straight into the response: on the K10's GC2145 the raw mean |neighbour
// difference| is 15-19 gray levels, which puts the noise floor close to
// FC_THRESHOLD_MAX and floods the detector with thousands of false hits. A
// 3x3 box (5x noise reduction) drops the floor to ~3 levels first.
//
// gray : w*h bytes, row-major; blurred in place
// tmp  : w*h bytes of scratch (contents on return are undefined)
void fast_blur3x3(uint8_t *gray, uint8_t *tmp, int w, int h);

// 2x2 box downsample: dst[(y * w/2) + x] = average of the 2x2 source block
// at (2x, 2y). dst must hold (w/2) * (h/2) bytes.
//
// Detecting on the downsampled image changes the feature scale: FAST's radius-3
// ring then spans a 6-pixel neighbourhood of the original frame, so large-object
// corners are found while 1-2 pixel texture is averaged away. It is also 4x
// cheaper to scan, and the box average itself halves the noise.
void fast_downsample2x2(const uint8_t *src, uint8_t *dst, int w, int h);
