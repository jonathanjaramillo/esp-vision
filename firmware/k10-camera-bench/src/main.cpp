// K10 camera downsampling benchmark.
//
// MODE 0: stock QVGA: GC2145 480x640 window -> 240x320 at 1/2.
// MODE 1: wide QVGA:  GC2145 720x960 window -> 240x320 at 1/3.
// MODE 2: VGA capture (480x640 portrait), software box-filter to 240x320.
//
// All modes produce the two buffers k10-lk-track needs:
//   preview: 240x320 RGB565
//   vision:  120x160 blurred luma
// No LCD is initialized: this measures camera + preprocessing without the
// common ~75 ms SPI/LVGL display cost obscuring the difference.

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <who_camera.h>

#ifndef BENCH_MODE
#define BENCH_MODE 0
#endif

static constexpr int OUT_W = 240;
static constexpr int OUT_H = 320;
static constexpr int DET_W = OUT_W / 2;
static constexpr int DET_H = OUT_H / 2;
static constexpr int WARMUP_FRAMES = 20;
static constexpr int BENCH_FRAMES = 200;

static uint16_t *preview;
static uint8_t *gray;
static uint8_t *det;
static uint8_t *blur_tmp;

struct Totals {
    uint64_t capture_us;
    uint64_t preview_us;
    uint64_t gray_us;
    uint64_t downsample_us;
    uint64_t blur_us;
    uint64_t total_us;
    uint32_t frames;
    uint32_t bad_frames;
    uint32_t checksum;
};

static inline uint8_t rgb565_luma(uint16_t p) {
    // Camera bytes are swapped relative to the CPU's uint16_t view.
    p = (uint16_t)((p << 8) | (p >> 8));
    int r = (p >> 11) & 31;
    int g = (p >> 5) & 63;
    int b = p & 31;
    return (uint8_t)((77 * ((r << 3) | (r >> 2)) +
                      150 * ((g << 2) | (g >> 4)) +
                       29 * ((b << 3) | (b >> 2)) + 128) >> 8);
}

static inline uint16_t average565(uint16_t a, uint16_t b,
                                  uint16_t c, uint16_t d) {
    // Averaging packed channels independently prevents carry between fields.
    uint32_t rb = (a & 0xf81f) + (b & 0xf81f) +
                  (c & 0xf81f) + (d & 0xf81f);
    uint32_t g  = (a & 0x07e0) + (b & 0x07e0) +
                  (c & 0x07e0) + (d & 0x07e0);
    return (uint16_t)(((rb >> 2) & 0xf81f) | ((g >> 2) & 0x07e0));
}

static void resize_vga_to_portrait(const uint16_t *src, int sw,
                                   uint16_t *dst) {
    // VGA arrives 640x480 landscape whereas QVGA arrives 240x320 portrait.
    // Box-filter VGA to 320x240 and rotate clockwise into 240x320.
    for (int y = 0; y < OUT_H; ++y) {
        uint16_t *d = dst + (size_t)y * OUT_W;
        int sx = 2 * y;
        for (int x = 0; x < OUT_W; ++x) {
            int sy = 2 * (OUT_W - 1 - x);
            const uint16_t *s0 = src + (size_t)sy * sw + sx;
            const uint16_t *s1 = s0 + sw;
            d[x] = average565(s0[0], s0[1], s1[0], s1[1]);
        }
    }
}

static void rotate_qvga_to_portrait(const uint16_t *src, uint16_t *dst) {
    // Raw sensor output is 320x240 landscape; rotate clockwise to 240x320.
    for (int y = 0; y < OUT_H; ++y) {
        uint16_t *d = dst + (size_t)y * OUT_W;
        for (int x = 0; x < OUT_W; ++x)
            d[x] = src[(size_t)(OUT_W - 1 - x) * OUT_H + y];
    }
}

static void to_gray(const uint16_t *src, uint8_t *dst) {
    for (int i = 0; i < OUT_W * OUT_H; ++i) dst[i] = rgb565_luma(src[i]);
}

static void downsample_gray_2x2(const uint8_t *src, uint8_t *dst) {
    for (int y = 0; y < DET_H; ++y) {
        const uint8_t *s0 = src + (size_t)(2 * y) * OUT_W;
        const uint8_t *s1 = s0 + OUT_W;
        uint8_t *d = dst + (size_t)y * DET_W;
        for (int x = 0; x < DET_W; ++x) {
            int sx = 2 * x;
            d[x] = (uint8_t)((s0[sx] + s0[sx + 1] +
                              s1[sx] + s1[sx + 1] + 2) >> 2);
        }
    }
}

static void blur3x3(uint8_t *img, uint8_t *tmp) {
    // Separable [1 1 1] box blur, matching the work shape used by the tracker.
    for (int y = 0; y < DET_H; ++y) {
        const uint8_t *s = img + (size_t)y * DET_W;
        uint8_t *d = tmp + (size_t)y * DET_W;
        d[0] = (uint8_t)((2 * s[0] + s[1] + 1) / 3);
        for (int x = 1; x < DET_W - 1; ++x)
            d[x] = (uint8_t)((s[x - 1] + s[x] + s[x + 1] + 1) / 3);
        d[DET_W - 1] = (uint8_t)((s[DET_W - 2] + 2 * s[DET_W - 1] + 1) / 3);
    }
    for (int y = 0; y < DET_H; ++y) {
        uint8_t *d = img + (size_t)y * DET_W;
        const uint8_t *a = tmp + (size_t)(y ? y - 1 : 0) * DET_W;
        const uint8_t *b = tmp + (size_t)y * DET_W;
        const uint8_t *c = tmp + (size_t)(y + 1 < DET_H ? y + 1 : y) * DET_W;
        for (int x = 0; x < DET_W; ++x)
            d[x] = (uint8_t)((a[x] + b[x] + c[x] + 1) / 3);
    }
}

static camera_config_t camera_config() {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = CAMERA_PIN_D0; c.pin_d1 = CAMERA_PIN_D1;
    c.pin_d2 = CAMERA_PIN_D2; c.pin_d3 = CAMERA_PIN_D3;
    c.pin_d4 = CAMERA_PIN_D4; c.pin_d5 = CAMERA_PIN_D5;
    c.pin_d6 = CAMERA_PIN_D6; c.pin_d7 = CAMERA_PIN_D7;
    c.pin_xclk = CAMERA_PIN_XCLK;
    c.pin_pclk = CAMERA_PIN_PCLK;
    c.pin_vsync = CAMERA_PIN_VSYNC;
    c.pin_href = CAMERA_PIN_HREF;
    c.pin_sscb_sda = CAMERA_PIN_SIOD;
    c.pin_sscb_scl = CAMERA_PIN_SIOC;
    c.pin_pwdn = CAMERA_PIN_PWDN;
    c.pin_reset = CAMERA_PIN_RESET;
    c.xclk_freq_hz = XCLK_FREQ_HZ;
    c.pixel_format = PIXFORMAT_RGB565;
    // In this bundled IDF 4.4 camera binary, requesting SVGA is the stable
    // way to obtain the GC2145's 640x480 RGB565 mode. A VGA request is
    // silently configured with the QVGA-sized DMA geometry.
    c.frame_size = BENCH_MODE == 2 ? FRAMESIZE_SVGA : FRAMESIZE_QVGA;
    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    return c;
}

static const char *mode_name() {
    if (BENCH_MODE == 0) return "stock_qvga";
    if (BENCH_MODE == 1) return "wide_qvga_sensor_1_3";
    if (BENCH_MODE == 2) return "vga_software_2x2";
    if (BENCH_MODE == 3) return "sensor_1_4_1280x960";
    return "sensor_1_5_full_1600x1200";
}

static void dump_b64(const char *tag, const uint8_t *data, size_t len,
                     int w, int h) {
    static const char b64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    Serial.printf("\n--FOV %s %d %d %u\n", tag, w, h, (unsigned)len);
    char line[66];
    for (size_t i = 0; i < len; i += 48) {
        size_t n = min((size_t)48, len - i);
        int o = 0;
        for (size_t j = 0; j < n; j += 3) {
            uint32_t a = data[i + j];
            uint32_t b = j + 1 < n ? data[i + j + 1] : 0;
            uint32_t c = j + 2 < n ? data[i + j + 2] : 0;
            uint32_t v = (a << 16) | (b << 8) | c;
            line[o++] = b64[(v >> 18) & 63];
            line[o++] = b64[(v >> 12) & 63];
            line[o++] = j + 1 < n ? b64[(v >> 6) & 63] : '=';
            line[o++] = j + 2 < n ? b64[v & 63] : '=';
        }
        line[o++] = '\n';
        Serial.write((const uint8_t *)line, o);
        if (((i / 48) & 15) == 0) delay(1);
    }
    Serial.println("--END");
}

static void run_benchmark() {
    Totals t = {};
    for (int n = -WARMUP_FRAMES; n < BENCH_FRAMES; ++n) {
        uint32_t all0 = micros();
        uint32_t c0 = micros();
        camera_fb_t *fb = esp_camera_fb_get();
        uint32_t capture = micros() - c0;
        if (!fb) {
            if (n >= 0) t.bad_frames++;
            continue;
        }

        int expected_w = BENCH_MODE == 2 ? 640 : OUT_W;
        int expected_h = BENCH_MODE == 2 ? 480 : OUT_H;
        bool geometry_ok = (int)fb->width == expected_w &&
                           (int)fb->height == expected_h &&
                           fb->len >= (size_t)expected_w * expected_h * 2;
        const uint16_t *work = (const uint16_t *)fb->buf;

        uint32_t p0 = micros();
        if (BENCH_MODE == 2 && geometry_ok) {
            resize_vga_to_portrait(work, expected_w, preview);
            work = preview;
        } else if (BENCH_MODE >= 3 && geometry_ok) {
            rotate_qvga_to_portrait(work, preview);
            work = preview;
        } else if (geometry_ok) {
            memcpy(preview, work, OUT_W * OUT_H * 2);
            work = preview;
        }
        uint32_t preview_time = micros() - p0;

        uint32_t g0 = micros();
        if (geometry_ok) to_gray(work, gray);
        uint32_t gray_time = micros() - g0;
        uint32_t d0 = micros();
        if (geometry_ok) downsample_gray_2x2(gray, det);
        uint32_t downsample_time = micros() - d0;
        uint32_t b0 = micros();
        if (geometry_ok) blur3x3(det, blur_tmp);
        uint32_t blur_time = micros() - b0;

        esp_camera_fb_return(fb);
        uint32_t total = micros() - all0;
        if (n >= 0) {
            t.frames++;
            if (!geometry_ok) t.bad_frames++;
            t.capture_us += capture;
            t.preview_us += preview_time;
            t.gray_us += gray_time;
            t.downsample_us += downsample_time;
            t.blur_us += blur_time;
            t.total_us += total;
            // Prevent optimization and provide a weak stuck-frame indicator.
            t.checksum = t.checksum * 16777619u ^ det[(n * 997u) % (DET_W * DET_H)];
        }
    }

    double f = t.frames ? t.frames : 1;
    Serial.println("RESULT_BEGIN");
    Serial.printf("mode=%s\n", mode_name());
    Serial.printf("frames=%u bad_frames=%u checksum=%08x\n",
                  t.frames, t.bad_frames, t.checksum);
    Serial.printf("capture_us=%.1f preview_us=%.1f gray_us=%.1f ",
                  t.capture_us / f, t.preview_us / f, t.gray_us / f);
    Serial.printf("downsample_us=%.1f blur_us=%.1f total_us=%.1f\n",
                  t.downsample_us / f, t.blur_us / f, t.total_us / f);
    Serial.printf("fps=%.3f cpu_preprocess_pct=%.1f\n",
                  1000000.0 * f / t.total_us,
                  100.0 * (t.preview_us + t.gray_us + t.downsample_us + t.blur_us) /
                      t.total_us);
    Serial.printf("heap_free=%u heap_largest=%u psram_free=%u psram_largest=%u\n",
                  heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                  heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    Serial.println("RESULT_END");

    if (BENCH_MODE >= 3) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            dump_b64(BENCH_MODE == 3 ? "SENSOR14" : "SENSOR15", fb->buf,
                     320 * 240 * 2, 320, 240);
            esp_camera_fb_return(fb);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1800);
    Serial.printf("\nK10 camera benchmark: %s\n", mode_name());

    preview = (uint16_t *)heap_caps_malloc(OUT_W * OUT_H * 2, MALLOC_CAP_SPIRAM);
    gray = (uint8_t *)heap_caps_malloc(OUT_W * OUT_H, MALLOC_CAP_SPIRAM);
    det = (uint8_t *)heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
    blur_tmp = (uint8_t *)heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
    if (!preview || !gray || !det || !blur_tmp) {
        Serial.println("FATAL: buffer allocation failed");
        return;
    }

    camera_config_t c = camera_config();
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        Serial.printf("FATAL: esp_camera_init=0x%x\n", (unsigned)err);
        return;
    }
    run_benchmark();
}

void loop() {
    delay(1000);
}
