// k10image.h — RGB565 -> 8-bit luma conversion for the UNIHIKER K10 camera.
//
// Carries two hard-won, measured fixes (see k10image.cpp for the full story):
// the K10 camera frame is byte-swapped on the wire, and the naive 5:2:1
// luma weighting with bit-expansion is non-monotonic. Both bugs together
// were the root cause of corners tracing colour contours on flat surfaces
// instead of finding real geometry. No camera/LVGL dependency — takes and
// returns plain buffers.
#pragma once

#include <stdint.h>

// Convert one RGB565 pixel (as delivered by the K10's GC2145 camera, i.e.
// byte-swapped on the wire) to 8-bit BT.601 luma.
uint8_t rgb565_luma(uint16_t p);

// Convert n RGB565 pixels to 8-bit luma. gray must hold n bytes.
void k10_to_grayscale(const uint16_t *rgb, uint8_t *gray, int n);
