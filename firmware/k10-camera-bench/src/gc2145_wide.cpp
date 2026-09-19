#include <Arduino.h>
#include <sensor.h>

#ifndef GC2145_FOV_MODE
#define GC2145_FOV_MODE 0
#endif

#if GC2145_FOV_MODE < 0 || GC2145_FOV_MODE > 3
#error "GC2145_FOV_MODE must be 0 (stock), 1 (1/3), 2 (1/4), or 3 (full 1/5)"
#endif

// GC2145_FOV_MODE values for a QVGA-sized camera buffer:
//   0: packaged stock mode, centered 480x640 window at 1/2
//   1: centered 720x960 portrait window at 1/3
//   2: centered 1280x960 landscape window at 1/4
//   3: full 1600x1200 landscape sensor at 1/5

extern "C" int gc2145_original_init(sensor_t *sensor);

static int (*packaged_set_framesize)(sensor_t *, framesize_t) = nullptr;

static int write8(sensor_t *s, int reg, int value) {
    return s->set_reg(s, reg, 0xff, value);
}

static int set_wide_framesize(sensor_t *s, framesize_t size) {
    int rc = packaged_set_framesize(s, size);
    if (rc || size != FRAMESIZE_QVGA || GC2145_FOV_MODE == 0)
        return rc;

    const bool portrait = GC2145_FOV_MODE == 1;
    const int win_w = GC2145_FOV_MODE == 3 ? 1600 :
                      GC2145_FOV_MODE == 2 ? 1280 : 720;
    const int win_h = GC2145_FOV_MODE == 3 ? 1200 : 960;
    const int col_s = (1600 - win_w) / 2;
    const int row_s = (1200 - win_h) / 2;
    const int selector = GC2145_FOV_MODE == 3 ? 0x55 :
                         GC2145_FOV_MODE == 2 ? 0x44 : 0x33;
    const int output_w = portrait ? 240 : 320;
    const int output_h = portrait ? 320 : 240;

    rc |= write8(s, 0xfe, 0x00);
    rc |= write8(s, 0x90, 0x01);
    rc |= write8(s, 0x09, row_s >> 8);
    rc |= write8(s, 0x0a, row_s & 0xff);
    rc |= write8(s, 0x0b, col_s >> 8);
    rc |= write8(s, 0x0c, col_s & 0xff);
    rc |= write8(s, 0x0d, (win_h + 8) >> 8);
    rc |= write8(s, 0x0e, (win_h + 8) & 0xff);
    rc |= write8(s, 0x0f, (win_w + 16) >> 8);
    rc |= write8(s, 0x10, (win_w + 16) & 0xff);
    rc |= write8(s, 0x99, selector);
    for (int reg = 0x9b; reg <= 0xa2; ++reg) rc |= write8(s, reg, 0x00);
    rc |= write8(s, 0x95, output_h >> 8);
    rc |= write8(s, 0x96, output_h & 0xff);
    rc |= write8(s, 0x97, output_w >> 8);
    rc |= write8(s, 0x98, output_w & 0xff);
    s->status.framesize = size;
    return rc;
}

extern "C" int gc2145_init(sensor_t *sensor) {
    int rc = gc2145_original_init(sensor);
    if (!rc) {
        packaged_set_framesize = sensor->set_framesize;
        sensor->set_framesize = set_wide_framesize;
    }
    return rc;
}
