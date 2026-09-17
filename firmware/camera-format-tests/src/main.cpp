// UNIHIKER K10 camera format probe.
//
// This initializes the GC2145 through esp_camera directly, captures a short
// run, and reports the actual framebuffer format, dimensions, byte length,
// capture interval, and sample statistics. Build this folder once for
// PIXFORMAT_GRAYSCALE and once for PIXFORMAT_YUV422.

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_err.h>
#include <who_camera.h>

#ifndef CAMERA_TEST_PIXEL_FORMAT
#define CAMERA_TEST_PIXEL_FORMAT PIXFORMAT_GRAYSCALE
#endif

#define TEST_FRAMES 30
#define FRAME_SIZE FRAMESIZE_QVGA
#define FB_COUNT 2

static const char *format_name(pixformat_t format) {
    switch (format) {
        case PIXFORMAT_GRAYSCALE: return "GRAYSCALE";
        case PIXFORMAT_YUV422:    return "YUV422";
        case PIXFORMAT_RGB565:    return "RGB565";
        default:                  return "other";
    }
}

static const char *err_name(esp_err_t err) {
    const char *name = esp_err_to_name(err);
    return name ? name : "unknown";
}

static camera_config_t camera_config() {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;
    c.pin_d0 = CAMERA_PIN_D0;
    c.pin_d1 = CAMERA_PIN_D1;
    c.pin_d2 = CAMERA_PIN_D2;
    c.pin_d3 = CAMERA_PIN_D3;
    c.pin_d4 = CAMERA_PIN_D4;
    c.pin_d5 = CAMERA_PIN_D5;
    c.pin_d6 = CAMERA_PIN_D6;
    c.pin_d7 = CAMERA_PIN_D7;
    c.pin_xclk = CAMERA_PIN_XCLK;
    c.pin_pclk = CAMERA_PIN_PCLK;
    c.pin_vsync = CAMERA_PIN_VSYNC;
    c.pin_href = CAMERA_PIN_HREF;
    // This K10 framework carries the older esp32-camera headers, where these
    // fields are still spelled pin_sscb_* (the newer spelling is pin_sccb_*).
    c.pin_sscb_sda = CAMERA_PIN_SIOD;
    c.pin_sscb_scl = CAMERA_PIN_SIOC;
    c.pin_pwdn = CAMERA_PIN_PWDN;
    c.pin_reset = CAMERA_PIN_RESET;
    c.xclk_freq_hz = XCLK_FREQ_HZ;
    c.pixel_format = CAMERA_TEST_PIXEL_FORMAT;
    c.frame_size = FRAME_SIZE;
    c.jpeg_quality = 12;
    c.fb_count = FB_COUNT;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    return c;
}

static void print_bytes(const uint8_t *data, size_t len) {
    const size_t n = len < 32 ? len : 32;
    Serial.print("first_bytes=");
    for (size_t i = 0; i < n; ++i) {
        if (i) Serial.print(' ');
        if (data[i] < 16) Serial.print('0');
        Serial.print(data[i], HEX);
    }
    Serial.println();
}

static void print_stats(const uint8_t *data, size_t len, size_t stride,
                        size_t offset, const char *label) {
    if (len <= offset || stride == 0) return;

    uint8_t min_value = 255;
    uint8_t max_value = 0;
    uint64_t sum = 0;
    size_t count = 0;
    for (size_t i = offset; i < len; i += stride) {
        const uint8_t v = data[i];
        if (v < min_value) min_value = v;
        if (v > max_value) max_value = v;
        sum += v;
        ++count;
    }

    Serial.printf("%s: samples=%u min=%u max=%u mean=%.2f\n",
                  label, (unsigned) count, min_value, max_value,
                  count ? (double) sum / (double) count : 0.0);
}

static bool expected_layout_ok(const camera_fb_t *fb) {
    const size_t pixels = fb->width * fb->height;
    const size_t expected = (CAMERA_TEST_PIXEL_FORMAT == PIXFORMAT_GRAYSCALE)
                                ? pixels
                                : pixels * 2;
    // Some camera drivers may add padding, so reject only undersized frames.
    return fb->format == CAMERA_TEST_PIXEL_FORMAT && fb->len >= expected;
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("=== K10 camera pixel-format test ===");
    Serial.printf("requested_format=%s frame_size=QVGA fb_count=%d xclk=%dHz\n",
                  format_name(CAMERA_TEST_PIXEL_FORMAT), FB_COUNT, XCLK_FREQ_HZ);
    Serial.printf("pins: xclk=%d sda=%d scl=%d pclk=%d vsync=%d href=%d\n",
                  CAMERA_PIN_XCLK, CAMERA_PIN_SIOD, CAMERA_PIN_SIOC,
                  CAMERA_PIN_PCLK, CAMERA_PIN_VSYNC, CAMERA_PIN_HREF);

    camera_config_t config = camera_config();
    const esp_err_t init_err = esp_camera_init(&config);
    if (init_err != ESP_OK) {
        Serial.printf("CAMERA_INIT_FAIL err=0x%x name=%s\n",
                      (unsigned) init_err, err_name(init_err));
        Serial.println("RESULT=FAIL");
        return;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor) {
        Serial.printf("sensor_pid=0x%04x version=0x%04x\n",
                      sensor->id.PID, sensor->id.VER);
    }

    size_t good = 0;
    size_t bad = 0;
    uint64_t interval_sum_us = 0;
    uint32_t interval_min_us = UINT32_MAX;
    uint32_t interval_max_us = 0;
    uint32_t previous_us = micros();
    bool first_seen = false;

    for (int i = 0; i < TEST_FRAMES; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        const uint32_t now_us = micros();
        const uint32_t interval_us = now_us - previous_us;
        previous_us = now_us;

        if (!fb) {
            ++bad;
            Serial.printf("frame=%d CAPTURE_FAIL\n", i);
            delay(20);
            continue;
        }

        if (i > 0) {
            interval_sum_us += interval_us;
            if (interval_us < interval_min_us) interval_min_us = interval_us;
            if (interval_us > interval_max_us) interval_max_us = interval_us;
        }

        const bool layout_ok = expected_layout_ok(fb);
        if (layout_ok) ++good; else ++bad;
        Serial.printf("frame=%d format=%s width=%u height=%u len=%u interval_us=%u layout=%s\n",
                      i, format_name(fb->format), (unsigned) fb->width,
                      (unsigned) fb->height, (unsigned) fb->len,
                      (unsigned) interval_us, layout_ok ? "OK" : "BAD");

        if (!first_seen) {
            first_seen = true;
            print_bytes(fb->buf, fb->len);
            // Print while this framebuffer is still owned by the test. The
            // driver may reuse its storage immediately after return.
            if (CAMERA_TEST_PIXEL_FORMAT == PIXFORMAT_GRAYSCALE) {
                print_stats(fb->buf, fb->len, 1, 0, "gray");
            } else if (CAMERA_TEST_PIXEL_FORMAT == PIXFORMAT_YUV422) {
                print_stats(fb->buf, fb->len, 2, 0, "yuv_lane_0");
                print_stats(fb->buf, fb->len, 2, 1, "yuv_lane_1");
            }
        }

        esp_camera_fb_return(fb);
    }

    const double avg_interval_ms = (TEST_FRAMES > 1)
        ? (double) interval_sum_us / (double) (TEST_FRAMES - 1) / 1000.0
        : 0.0;
    Serial.printf("summary good=%u bad=%u avg_interval_ms=%.2f min_interval_us=%u max_interval_us=%u\n",
                  (unsigned) good, (unsigned) bad, avg_interval_ms,
                  interval_min_us == UINT32_MAX ? 0 : (unsigned) interval_min_us,
                  (unsigned) interval_max_us);

    const bool passed = good >= (TEST_FRAMES - 2) && bad <= 2;
    Serial.printf("RESULT=%s\n", passed ? "PASS" : "FAIL");
    Serial.println("Test complete; reset or reflash to test another format.");
}

void loop() {
    delay(1000);
}
