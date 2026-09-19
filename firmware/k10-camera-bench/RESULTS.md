# Hardware results

Measured on the connected UNIHIKER K10, 2026-09-19. Each result covers 200
frames after 20 warm-up frames. LCD/LVGL and feature detection/tracking are
excluded; every mode produces the same 240x320 RGB565 preview and blurred
120x160 luma buffer.

| Mode | Capture | Preview preparation | Gray | Gray downsample | Blur | Total | FPS |
|---|---:|---:|---:|---:|---:|---:|---:|
| Stock QVGA 1/2 | 54.53 ms | 6.77 ms | 16.23 ms | 2.02 ms | 2.51 ms | 82.07 ms | 12.18 |
| Wide QVGA 1/3 | 116.08 ms | 6.77 ms | 16.23 ms | 2.02 ms | 2.68 ms | 143.79 ms | 6.96 |
| 640x480 + software 2x2/rotate | 194.62 ms | 86.15 ms | 16.22 ms | 2.29 ms | 2.92 ms | 302.20 ms | 3.31 |

All successful runs processed 200/200 valid frames. Active preprocessing was
27.54 ms/frame for stock QVGA, 27.70 ms/frame for wide QVGA, and 107.58
ms/frame for VGA software reduction. Wide QVGA therefore changes almost none
of the CPU-side work; its lower frame rate comes from waiting for the sensor.

VGA consumed approximately 460,800 additional bytes of PSRAM, matching the
difference between a 640x480 and 240x320 RGB565 camera framebuffer. Internal
heap headroom was effectively unchanged.

## Recommendation

Use wide QVGA 1/3 for the tracking firmware. It provides the verified 1.5x
larger sensor window with the same working-buffer sizes and nearly identical
CPU preprocessing cost. It is about 2.1x faster than VGA plus software
downsampling and saves about 450 KiB of PSRAM per camera framebuffer.

Use VGA only if the visibly better 2x2 software antialiasing is worth reducing
the camera/preprocessing ceiling to roughly 3.3 fps, or if later processing
actually needs the additional source pixels.
