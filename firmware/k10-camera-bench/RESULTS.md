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
| 1280x960 window, sensor 1/4 | 171.49 ms | 8.96 ms | 16.21 ms | 2.29 ms | 2.51 ms | 201.47 ms | 4.96 |
| Full 1600x1200, sensor 1/5 | 259.13 ms | 8.90 ms | 16.21 ms | 2.29 ms | 2.51 ms | 289.05 ms | 3.46 |

All successful runs processed 200/200 valid frames. Active preprocessing was
27.54 ms/frame for stock QVGA, 27.70 ms/frame for wide QVGA, and 107.58
ms/frame for VGA software reduction. Wide QVGA therefore changes almost none
of the CPU-side work; its lower frame rate comes from waiting for the sensor.

Both new sensor modes completed 200/200 valid frames, retained the QVGA camera
buffer size, and produced visually complete 320x240 frames without the repeated
or torn region seen in the earlier invalid portrait-window experiment. The 1/4
mode covers 80% of each full-sensor dimension; the 1/5 mode uses the complete
1600x1200 active area and therefore retains the full lens/sensor FOV.

VGA consumed approximately 460,800 additional bytes of PSRAM, matching the
difference between a 640x480 and 240x320 RGB565 camera framebuffer. Internal
heap headroom was effectively unchanged.

## Recommendation

For the best speed/FOV balance, use wide QVGA 1/3. For maximum FOV, use the new
full-sensor 1/5 mode: it preserves QVGA memory and only adds about 2.2 ms of CPU
rotation versus stock, but its larger sensor scan lowers the ceiling to 3.46
fps. Sensor 1/4 is the useful middle point at 80% sensor coverage and 4.96 fps.

Wide QVGA 1/3 remains about 2.1x faster than VGA plus software downsampling and
saves about 450 KiB of PSRAM per camera framebuffer.

Use VGA only if the visibly better 2x2 software antialiasing is worth reducing
the camera/preprocessing ceiling to roughly 3.3 fps, or if later processing
actually needs the additional source pixels.
