// orb.h — ORB feature extraction: orientation + BRIEF-256 descriptor.
//
// Takes the corners fastcorner.cpp already found and turns each into a
// rotation-aware 256-bit descriptor:
//
//   1. orientation: intensity centroid over a circular patch (Rosin) —
//      angle = atan2(m01, m10), the same method as the original ORB paper.
//   2. descriptor: 256 fixed intensity-pair tests (rejection-sampled from an
//      isotropic Gaussian, NOT OpenCV's learned/greedy-selected pattern —
//      this is the untrained "steered BRIEF" baseline from the ORB paper,
//      generated once at startup with a fixed seed so it's reproducible run
//      to run), rotated by the keypoint's own angle before sampling so the
//      descriptor is (approximately) rotation-invariant.
//
// Matching is Hamming distance on the 256-bit descriptor (orb_hamming).
// This file has no OpenCV dependency and no camera/LVGL dependency — it only
// needs an 8-bit grayscale patch, exactly like fastcorner.h.
#pragma once

#include <stdint.h>

#define ORB_DESC_BYTES     32   // 256-bit descriptor
#define ORB_PATCH_RADIUS    9   // patch is (2r+1)x(2r+1); see orb.cpp for why

typedef struct {
    uint8_t desc[ORB_DESC_BYTES];
    float   angle;   // radians, intensity-centroid orientation
    int16_t x, y;    // pixel coords in the image orb_compute was called on
} orb_feature_t;

// Compute the ORB orientation + descriptor for one keypoint at (cx, cy) in an
// 8-bit grayscale image of w*h (row-major). The image should already be
// smoothed (fast_blur3x3) — BRIEF is a raw intensity compare and is exactly
// as noise-sensitive as FAST is.
//
// Returns 0 on success. Returns -1 if the patch would run past the image
// border (needs ORB_PATCH_RADIUS + 1 px of margin on every side, since the
// pattern is rejection-sampled to stay within radius ORB_PATCH_RADIUS even
// after rotation) — the caller should just skip that keypoint.
int orb_compute(const uint8_t *gray, int w, int h, int cx, int cy, orb_feature_t *out);

// Compute descriptors for n keypoints (xs[i], ys[i]). Keypoints too close to
// the border are silently skipped (see orb_compute), so the number written
// can be less than n. Returns the number written (<= max_out).
int orb_compute_batch(const uint8_t *gray, int w, int h,
                      const int16_t *xs, const int16_t *ys, int n,
                      orb_feature_t *out, int max_out);

// Hamming distance between two 256-bit descriptors (popcount of the XOR).
// 0 = identical; ~128 = the expected distance between unrelated descriptors;
// under ~64 is the usual "good match" cutoff in the ORB literature.
int orb_hamming(const uint8_t a[ORB_DESC_BYTES], const uint8_t b[ORB_DESC_BYTES]);
