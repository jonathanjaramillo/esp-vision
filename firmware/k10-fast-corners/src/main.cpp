// k10-fast-corners — FAST-9 corner detection on the UNIHIKER K10.
//
// Pipeline (one FreeRTOS task, pinned to core 1, priority 5 — LVGL is driven
// exclusively by it):
//   1. pull an RGB565 240x320 PORTRAIT frame from the camera queue
//   2. convert to 8-bit grayscale (internal SRAM)
//   3. FAST-9 detect (+ spatial suppression)
//   4a. copy the frame 1:1 into the portrait display buffer (PSRAM)
//   4b. and stamp corner markers
//   5. invalidate the full-screen lv_img bound to that buffer, repaint a
//      small HUD strip on the LVGL canvas, single lv_task_handler() pass
//
// Serial commands (typed into the monitor, handled in loop()):
//   d : dump the 120x160 luma image the detector actually sees, base64
//   D : dump the raw 240x320 RGB565 camera frame, base64
//   (decode either with tools/capture.py — see PLAN.md "Capturing a real frame")
//   s : toggle the display on/off — the ~75ms/frame SPI flush is the actual
//       frame-rate ceiling, so turning the screen off (no on-device preview
//       needed during WiFi streaming) buys real throughput; see PLAN.md
//       "Screen on/off toggle"
//
// Buttons (callbacks run on the button task — they only touch the atomics):
//   A  tap : threshold +4     (more selective, fewer corners)
//   B  tap : threshold -4     (lower t, more corners)
//   A+B    : toggle detection stride 1 <-> 2 (stride 2 = 1/4 the work)
//   A or B, held >= LONG_PRESS_MS (600ms): toggle the display on/off — same
//   action as the 's' serial command; see PLAN.md "Screen on/off toggle".
//
// LED: green >= 8 fps, yellow >= 4 fps, red below (display ceiling ~9 fps).
//
// WiFi streaming (see PLAN.md "Streaming to a host for visual odometry"):
//   Every frame's ORB features (position/angle/descriptor) and the
//   accelerometer (sampled independently, ~50 Hz) are sent as UDP datagrams
//   to STREAM_HOST_IP (a shell env var — see PLAN.md — edit it if the host's
//   IP changes; no reflash needed to try a new value, just re-export + pio run).
//   tools/stream_recv.py decodes both. Wire format is documented there and in
//   PLAN.md; keep the two in sync if it changes.
//
//   Unicast, not subnet broadcast: broadcast was tried first (so neither side
//   needs the other's IP) and measured DOA on this network — the K10 reports
//   a correct IP/mask/broadcast address and ICMP unicast to it works, but no
//   broadcast packet ever arrived at a listener on the same WiFi, consistent
//   with the broadcast/multicast-over-WiFi filtering many mesh/home routers
//   do to save airtime (they still forward unicast between clients fine).
//   See PLAN.md for how this was diagnosed.
//
// Conventions from k10-smoke (do NOT violate):
//   * never call lv_task_handler() outside the framework Canvas methods
//   * never touch the canvas from button callbacks
//   * keep loop() light
//   * include WiFi.h before unihiker_k10.h (it pulls in TFT_eSPI, which
//     pollutes the WiFi/esp_http_server include chain if they come after —
//     see k10-smoke/PLAN.md gotcha 4)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <unihiker_k10.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>
#include <math.h>

#include "fastcorner.h"
#include "orb.h"
#include "k10image.h"
#include "k10stream.h"

#define CAM_W 240                 // native portrait frame from the K10 camera
#define CAM_H 320
#define DET_W (CAM_W / 2)         // detection runs at half res (see PLAN.md)
#define DET_H (CAM_H / 2)         // so FAST sees ~6 px features, not 3 px
#define SCR_W 240                 // portrait display — matches CAM_W x CAM_H 1:1
#define SCR_H 320

#define MARK_COLOR 0xF81F         // RGB565 magenta
#define HUD_COLOR  0x00FFFF       // 0xRRGGBB cyan

UNIHIKER_K10 k10;

static lv_obj_t    *feed_img = NULL;  // full-screen feed image (behind canvas)
static lv_img_dsc_t feed_dsc;         // descriptor bound to dispBuf
static QueueHandle_t  xQueueCam = NULL;
static uint8_t       *grayBuf  = NULL;   // CAM_W*CAM_H, internal SRAM
static uint8_t       *detBuf   = NULL;   // DET_W*DET_H downscaled luma
static uint8_t       *detTmp   = NULL;   // DET_W*DET_H blur scratch
static uint16_t      *dispBuf  = NULL;   // SCR_W*SCR_H, PSRAM
static fc_result_t    corners;
static orb_feature_t  s_feats[FC_MAX_CORNERS];   // ORB descriptors, one per corner that got one

// Tunables — written by button callbacks (button task), read by the pipeline.
static volatile int32_t g_threshold = FC_THRESHOLD_DEFAULT;
static volatile int32_t g_stride    = 1;   // 1 = every pixel of the half-res image
static volatile int32_t g_screen_on = 1;   // toggled by the 's' serial command — see PLAN.md

// Stats (written by pipeline task, read by loop()).
static volatile uint32_t g_fps        = 0;
static volatile uint32_t g_detect_us  = 0;
static volatile uint32_t g_orb_us     = 0;
static volatile uint32_t g_draw_us    = 0;
static volatile int32_t  g_corner_cnt = 0;
static volatile int32_t  g_cand_cnt   = 0;   // raw hits before suppression/cap
static volatile int32_t  g_pool_full  = 0;   // candidate pool saturated (see fastcorner.cpp)
static volatile int32_t  g_orb_cnt    = 0;   // corners that actually got a descriptor (border-skipped otherwise)
static volatile int32_t  g_bx0 = 0, g_bx1 = 0, g_by0 = 0, g_by1 = 0;
static volatile int32_t  g_fb_w = 0, g_fb_h = 0, g_fb_len = 0;

// Frame dump: set by loop() from a serial command, serviced by the pipeline
// task (0 = idle, 'd' = detector luma, 'D' = raw RGB565).
static volatile int32_t g_dump_req = 0;

// ---------------------------------------------------------------------------
// WiFi UDP streaming — ORB features + accelerometer, broadcast on the LAN.
//
// Broadcast (not a connection to a fixed host IP) so neither side needs the
// other's address configured: the K10 just needs to be on the same subnet as
// whatever's listening (tools/stream_recv.py). Two independent streams on two
// ports, sent from two different places at two different natural rates:
//   - ORB features: once per camera frame, from the pipeline task (~8 fps).
//   - Accelerometer: from loop(), at ACCEL_PERIOD_MS independent of the
//     camera — a future visual-INERTIAL pipeline wants IMU samples faster
//     than frames, and k10.getAccelerometerX/Y/Z() just returns a cached
//     value from the vendor lib's own background task, so sampling it here
//     costs nothing and never blocks on I2C.
//
// Every struct is `packed` and every field is fixed-width so
// tools/stream_recv.py's struct.unpack format string stays a byte-for-byte
// match — keep the two in sync if this changes (see PLAN.md).
// ---------------------------------------------------------------------------

// WIFI_SSID / WIFI_PASSWORD / STREAM_HOST_IP come from the shell environment
// via platformio.ini's build_flags — export them before `pio run`, never
// hardcode them here (see PLAN.md).
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef STREAM_HOST_IP
#define STREAM_HOST_IP ""   // dotted-quad string, e.g. "172.16.35.211"
#endif
#define ORB_STREAM_PORT   5005
#define ACCEL_STREAM_PORT 5006
#define ACCEL_PERIOD_MS   20     // 50 Hz

static WiFiUDP  udpOrb;
static WiFiUDP  udpAccel;
static IPAddress g_dest;
static volatile bool g_wifi_up = false;
static uint32_t g_orb_seq = 0;
static uint32_t g_accel_seq = 0;
static volatile uint32_t g_orb_send_ok = 0, g_orb_send_fail = 0;  // diagnostic — see PLAN.md

// ---------------------------------------------------------------------------
// Buttons
//
// A and B each only get one physical gesture free (A+B is already the stride
// toggle), so the screen toggle rides on a long-press: the Button class only
// gives pressed/unpressed callbacks, no built-in long-press, so pressed just
// stamps a timestamp and the actual action happens on release, decided by
// how long it was held. A quick tap is unchanged (threshold +/-4); holding
// past LONG_PRESS_MS toggles the screen — same action g_screen_on/'s' already
// drives (see PLAN.md "Screen on/off toggle").
// ---------------------------------------------------------------------------

#define LONG_PRESS_MS 600

static volatile uint32_t g_btnA_down_ms = 0;
static volatile uint32_t g_btnB_down_ms = 0;

static void on_button_a_pressed(void) { g_btnA_down_ms = millis(); }
static void on_button_b_pressed(void) { g_btnB_down_ms = millis(); }

static void on_button_a_released(void) {
    if (millis() - g_btnA_down_ms >= LONG_PRESS_MS) {
        g_screen_on = !g_screen_on;
        return;
    }
    int32_t t = g_threshold + 4;
    if (t > FC_THRESHOLD_MAX) t = FC_THRESHOLD_MAX;
    g_threshold = t;
}

static void on_button_b_released(void) {
    if (millis() - g_btnB_down_ms >= LONG_PRESS_MS) {
        g_screen_on = !g_screen_on;
        return;
    }
    int32_t t = g_threshold - 4;
    if (t < FC_THRESHOLD_MIN) t = FC_THRESHOLD_MIN;
    g_threshold = t;
}

static void on_button_ab(void) {
    g_stride = (g_stride == 1) ? 2 : 1;
}

// ---------------------------------------------------------------------------
// Image helpers
// ---------------------------------------------------------------------------

// RGB565 -> luma conversion (byte-swap + BT.601 weighting, see lib/k10image)
// lives in k10image.h/cpp now — used below as k10_to_grayscale().

// The K10 camera (GC2145, QVGA via the framework register_camera) already
// delivers a 240x320 PORTRAIT frame that matches the ILI9341 screen 1:1 —
// no rotation is needed; camera coordinates == display coordinates.
// (An earlier build assumed a 320x240 landscape frame and rotated it,
// which scrambled the image into 4 squished sub-frames.)

// ---------------------------------------------------------------------------
// Frame dump (base64 over CDC)
//
// The whole reason this exists: what FAST responds to on a real wall cannot be
// guessed from a 2.8" screen showing 5x5 markers, and synthetic frames did not
// reproduce it (noise dithers away exactly the quantisation contours we were
// chasing). One real frame is worth any number of fixtures — and once the luma
// image is on the host, thresholds and scoring can be tuned with no reflash.
//
// Runs in the pipeline task, so it must respect the same two rules as every
// other CDC write here: never write more than availableForWrite() will take
// (USBCDC::write() spins forever on a full ring), and vTaskDelay() between
// chunks so IDLE gets a slice and the TWDT stays quiet.
// ---------------------------------------------------------------------------

static void dump_b64(const char *tag, const uint8_t *data, size_t len,
                     int w, int h) {
    static const char *B64 =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    char hdr[64];
    const int hlen = snprintf(hdr, sizeof(hdr), "\n--BEGIN %s %d %d %u\n",
                              tag, w, h, (unsigned) len);
    while (Serial.availableForWrite() < hlen) vTaskDelay(pdMS_TO_TICKS(2));
    Serial.write((const uint8_t *) hdr, (size_t) hlen);

    // 48 input bytes -> 64 base64 chars + newline, one tidy line at a time.
    char line[66];
    for (size_t i = 0; i < len; i += 48) {
        const size_t n = (len - i < 48) ? (len - i) : 48;
        int o = 0;
        for (size_t j = 0; j < n; j += 3) {
            const uint32_t b0 = data[i + j];
            const uint32_t b1 = (j + 1 < n) ? data[i + j + 1] : 0;
            const uint32_t b2 = (j + 2 < n) ? data[i + j + 2] : 0;
            const uint32_t v = (b0 << 16) | (b1 << 8) | b2;
            line[o++] = B64[(v >> 18) & 0x3F];
            line[o++] = B64[(v >> 12) & 0x3F];
            line[o++] = (j + 1 < n) ? B64[(v >> 6) & 0x3F] : '=';
            line[o++] = (j + 2 < n) ? B64[v & 0x3F] : '=';
        }
        line[o++] = '\n';
        while (Serial.availableForWrite() < o) vTaskDelay(pdMS_TO_TICKS(2));
        Serial.write((const uint8_t *) line, (size_t) o);
        if (((i / 48) & 0x0F) == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }

    const char *end = "--END\n";
    while (Serial.availableForWrite() < 6) vTaskDelay(pdMS_TO_TICKS(2));
    Serial.write((const uint8_t *) end, 6);
}

// Stamp a 5x5 plus marker at display coords (dx, dy), clipped to bounds.
static void stamp_marker(uint16_t *buf, int dx, int dy) {
    if (dx < 2 || dx >= SCR_W - 2 || dy < 2 || dy >= SCR_H - 2) return;
    const int base = (size_t) dy * SCR_W + dx;
    for (int o = -2; o <= 2; o++) {
        buf[base + o]            = MARK_COLOR;   // horizontal arm
        buf[base + o * SCR_W]    = MARK_COLOR;   // vertical arm
    }
}



// ---------------------------------------------------------------------------
// Pipeline task (core 0)
// ---------------------------------------------------------------------------

static void pipeline_task(void *arg) {
    (void) arg;
    uint32_t win_start = micros();
    uint32_t win_frames = 0;

    for (;;) {
        camera_fb_t *fb;
        if (xQueueReceive(xQueueCam, &fb, portMAX_DELAY) != pdTRUE || !fb) {
            if (fb) esp_camera_fb_return(fb);
            continue;
        }

        const uint16_t *rgb = (const uint16_t *) fb->buf;
        if (g_fb_w == 0) { g_fb_w = fb->width; g_fb_h = fb->height; g_fb_len = fb->len; }

        k10_to_grayscale(rgb, grayBuf, CAM_W * CAM_H);
        // Detect at half resolution. The 2x2 box average suppresses 1-2 px
        // texture and halves the sensor noise, and FAST's radius-3 ring then
        // spans ~6 px of the original frame — so markers land on large-object
        // corners rather than fine detail. One 3x3 pass cleans up the rest.
        fast_downsample2x2(grayBuf, detBuf, CAM_W, CAM_H);
        fast_blur3x3(detBuf, detTmp, DET_W, DET_H);
        const uint32_t t1 = micros();
        fast_corner_detect(detBuf, DET_W, DET_H,
                           (int) g_threshold, (int) g_stride, &corners);
        g_detect_us = micros() - t1;
        g_cand_cnt = corners.candidates;
        g_pool_full = corners.pool_full;

        // ORB: orientation + BRIEF-256 descriptor for every corner that has a
        // full patch margin (ORB_PATCH_RADIUS=9, vs. fast_corner_detect's own
        // border inset of 4 — corners closer to the edge than that are simply
        // skipped; see PLAN.md). Runs on detBuf, the same half-res, blurred
        // luma the detector just scored.
        {
            int16_t xs[FC_MAX_CORNERS], ys[FC_MAX_CORNERS];
            for (int i = 0; i < corners.count; i++) {
                xs[i] = corners.corners[i].x;
                ys[i] = corners.corners[i].y;
            }
            const uint32_t t1b = micros();
            g_orb_cnt = orb_compute_batch(detBuf, DET_W, DET_H, xs, ys, corners.count,
                                          s_feats, FC_MAX_CORNERS);
            g_orb_us = micros() - t1b;
            if (g_wifi_up) {
                const bool ok = k10stream_send_orb(udpOrb, g_dest, ORB_STREAM_PORT,
                                                   g_orb_seq, s_feats, g_orb_cnt,
                                                   DET_W, DET_H, t1b);
                if (ok) g_orb_send_ok++; else g_orb_send_fail++;
            }
        }

        if (corners.count > 0) {
            int mnx = 32767, mxx = -1, mny = 32767, mxy = -1;
            for (int i = 0; i < corners.count; i++) {
                int cx = corners.corners[i].x * 2, cy = corners.corners[i].y * 2;
                if (cx < mnx) mnx = cx; if (cx > mxx) mxx = cx;
                if (cy < mny) mny = cy; if (cy > mxy) mxy = cy;
            }
            g_bx0 = mnx; g_bx1 = mxx; g_by0 = mny; g_by1 = mxy;
        }

        // 4+5. Display: stamp markers + refresh the feed image + HUD strip —
        // skippable at runtime (the 's' serial command, see PLAN.md). The SPI
        // flush this drives is the actual frame-rate ceiling (~75ms of a
        // ~125ms period, vs. ~15-20ms for detect+orb combined; see PLAN.md
        // "Screen on/off toggle"), so turning it off is what buys real
        // throughput for a WiFi-streaming/VO session with no on-device
        // preview needed.
        static int32_t prev_screen_on = 1;
        const int32_t screen_on = g_screen_on;
        if (screen_on && !prev_screen_on) {
            // ON-transition: the OFF branch below paints an opaque full-
            // screen black rect ("SCREEN OFF") on the canvas layer, which
            // sits ABOVE feed_img (see setup()). The per-frame path only
            // ever repaints the small 20px HUD strip, so without this the
            // rest of that opaque rect stays stuck on screen forever,
            // hiding the camera feed underneath even though feed_img itself
            // is updating fine every frame. clearLocalCanvas restores full
            // transparency so the feed shows through everywhere again.
            k10.canvas->clearLocalCanvas(0, 0, SCR_W, SCR_H);
        }
        if (screen_on) {
            memcpy(dispBuf, rgb, (size_t) SCR_W * SCR_H * 2);
            for (int i = 0; i < corners.count; i++) {
                // half-res detection coords -> full-res display coords
                stamp_marker(dispBuf, corners.corners[i].x * 2, corners.corners[i].y * 2);
            }

            const uint32_t t2 = micros();
            lv_obj_invalidate(feed_img);
            k10.canvas->canvasRectangle(0, 0, SCR_W, 20, 0x000000, 0x000000, true);
            char hud[48];
            snprintf(hud, sizeof(hud), "FAST-9 t=%ld s%ld %d cnr %d orb",
                     (long) g_threshold, (long) g_stride, corners.count, (int) g_orb_cnt);
            k10.canvas->canvasText(hud, 2, 3, HUD_COLOR, Canvas::eCNAndENFont16, 50, false);
            k10.canvas->updateCanvas();
            g_draw_us = micros() - t2;
        } else {
            g_draw_us = 0;
            if (prev_screen_on) {
                // One last draw on the off-transition so the screen doesn't
                // freeze on a stale frame — shows the board is alive and why
                // the preview stopped, then no further LVGL calls happen
                // until it's switched back on.
                k10.canvas->canvasRectangle(0, 0, SCR_W, SCR_H, 0x000000, 0x000000, true);
                k10.canvas->canvasText("SCREEN OFF", 2, 140, HUD_COLOR,
                                       Canvas::eCNAndENFont16, 50, false);
                k10.canvas->canvasText("streaming...", 2, 160, HUD_COLOR,
                                       Canvas::eCNAndENFont16, 50, false);
                k10.canvas->updateCanvas();
            }
        }
        prev_screen_on = screen_on;

        // Dump before returning the frame buffer — 'D' needs the raw frame, and
        // both need to correspond to the corner list just printed.
        const int32_t req = g_dump_req;
        if (req) {
            g_dump_req = 0;
            if (req == 'd') {
                dump_b64("GRAY", detBuf, (size_t) DET_W * DET_H, DET_W, DET_H);
            } else {
                dump_b64("RGB565", (const uint8_t *) rgb,
                         (size_t) CAM_W * CAM_H * 2, CAM_W, CAM_H);
            }
        }

        esp_camera_fb_return(fb);
        // Yield so the IDLE task gets a slice (or TWDT fires when frames are
        // always queued: sensor rate > pipeline rate).
        vTaskDelay(pdMS_TO_TICKS(1));

        g_corner_cnt = corners.count;
        win_frames++;
        const uint32_t now = micros();
        if (now - win_start >= 500000) {
            g_fps = (uint32_t) ((float) win_frames * 1000000.0f / (now - win_start));
            win_start = now;
            win_frames = 0;
        }
    }
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Arduino hooks
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(2000);  // USB-CDC settle
    Serial.println();
    Serial.println("== k10-fast-corners ==");

    k10.begin();
    k10.initScreen(2);              // portrait 240x320

    // Feed image first so the LVGL canvas (created next) renders ABOVE it.
    feed_img = lv_img_create(lv_scr_act());
    lv_obj_set_pos(feed_img, 0, 0);
    lv_obj_set_size(feed_img, SCR_W, SCR_H);

    k10.creatCanvas();              // HUD overlay (240x320 alpha canvas)
    k10.canvas->canvasSetLineWidth(1);
    k10.setScreenBackground(0x000000);

    grayBuf = (uint8_t *) heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_INTERNAL);
    if (!grayBuf) grayBuf = (uint8_t *) heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_SPIRAM);
    detBuf = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
    if (!detBuf) detBuf = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_SPIRAM);
    detTmp = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
    if (!detTmp) detTmp = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_SPIRAM);
    dispBuf = (uint16_t *) heap_caps_malloc((size_t) SCR_W * SCR_H * 2, MALLOC_CAP_SPIRAM);
    if (!grayBuf || !detBuf || !detTmp || !dispBuf) {
        Serial.printf("buffer alloc failed: gray=%p det=%p tmp=%p disp=%p\n",
                      (void *) grayBuf, (void *) detBuf, (void *) detTmp, (void *) dispBuf);
        return;
    }

    // Bind the feed image to our display buffer and show a cleared frame.
    memset(&feed_dsc, 0, sizeof(feed_dsc));
    feed_dsc.header.always_zero = 0;
    feed_dsc.header.w = SCR_W;
    feed_dsc.header.h = SCR_H;
    feed_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    feed_dsc.data_size = (size_t) SCR_W * SCR_H * 2;
    feed_dsc.data = (const uint8_t *) dispBuf;
    lv_img_set_src(feed_img, &feed_dsc);

    xQueueCam = xQueueCreate(2, sizeof(camera_fb_t *));
    register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, xQueueCam);

    k10.buttonA->setPressedCallback(on_button_a_pressed);
    k10.buttonA->setUnPressedCallback(on_button_a_released);
    k10.buttonB->setPressedCallback(on_button_b_pressed);
    k10.buttonB->setUnPressedCallback(on_button_b_released);
    k10.buttonAB->setPressedCallback(on_button_ab);

    // WiFi + UDP streaming (ORB features + accelerometer) to STREAM_HOST_IP.
    // Non-fatal on failure — the detector/display work with no network at
    // all, they just won't stream. Blocks up to 15s (same policy as k10-smoke).
    g_dest.fromString(STREAM_HOST_IP);
    g_wifi_up = k10stream_wifi_begin(WIFI_SSID, WIFI_PASSWORD, udpOrb, udpAccel);
    if (g_wifi_up) {
        Serial.printf("wifi up: %s -> %s  orb->udp:%d  accel->udp:%d\n",
                      WiFi.localIP().toString().c_str(), g_dest.toString().c_str(),
                      ORB_STREAM_PORT, ACCEL_STREAM_PORT);
    } else {
        Serial.println("wifi connect failed/timed out — streaming disabled, running local-only");
    }

    Serial.printf("ready. gray=%lu KB, disp=%lu KB\n",
                  (unsigned long)(CAM_W * CAM_H / 1024),
                  (unsigned long)((size_t) SCR_W * SCR_H * 2 / 1024));

    // LVGL is driven exclusively by this task; the button task never touches it.
    xTaskCreatePinnedToCore(pipeline_task, "fc_pipe", 8 * 1024,
                            NULL, 5, NULL, 0);  // core 0 for now (core 1 hang TBD)
}

void loop() {
    // Serial commands. Only latch the request; the pipeline task owns the
    // buffers and does the writing (loop() must stay light).
    while (Serial.available()) {
        const int c = Serial.read();
        if (c == 'd' || c == 'D') g_dump_req = c;
        if (c == 's') {
            g_screen_on = !g_screen_on;
            Serial.printf("screen %s\n", g_screen_on ? "ON" : "OFF");
        }
    }

    // Accelerometer stream: independent of, and much faster than, the camera
    // frame rate (see the WiFi streaming block above for why). getAccelerometerX
    // /Y/Z() just read a value the vendor lib's own background task already
    // keeps current, so this never blocks.
    static uint32_t last_accel = 0;
    const uint32_t now_ms = millis();
    if (g_wifi_up && now_ms - last_accel >= ACCEL_PERIOD_MS) {
        last_accel = now_ms;
        k10stream_send_accel(udpAccel, g_dest, ACCEL_STREAM_PORT, g_accel_seq,
                             k10.getAccelerometerX(), k10.getAccelerometerY(),
                             k10.getAccelerometerZ(), micros());
    }

    static uint32_t last = 0;
    const uint32_t now = now_ms;
    if (now - last < 1000) return;
    last = now;

    const uint32_t fps = g_fps;
    const int n = (int) g_corner_cnt;
    // Format into a stack buffer, write only when the CDC ring has room:
    // USBCDC::write() spins forever on a full/drain-blocked EP, so never call
    // it with a full ring.
    char line[256];
    const int llen = snprintf(line, sizeof(line),
                            "fps=%lu detect=%lu us orb=%lu us(%ld/%d) draw=%lu us(scr %s) t=%ld s%ld corr=%d cand=%ld "
                            "bx=[%ld..%ld] by=[%ld..%ld] fb=%ldx%ld len=%ld wifi=%s udp_ok=%lu udp_fail=%lu%s\n",
                            (unsigned long) fps, (unsigned long) g_detect_us,
                            (unsigned long) g_orb_us, (long) g_orb_cnt, n,
                            (unsigned long) g_draw_us, g_screen_on ? "ON" : "OFF",
                            (long) g_threshold, (long) g_stride, n,
                            (long) g_cand_cnt,
                            (long) g_bx0, (long) g_bx1, (long) g_by0, (long) g_by1,
                            (long) g_fb_w, (long) g_fb_h, (long) g_fb_len,
                            g_wifi_up ? WiFi.localIP().toString().c_str() : "down",
                            (unsigned long) g_orb_send_ok, (unsigned long) g_orb_send_fail,
                            g_pool_full ? " POOL-FULL" : "");
    if (Serial.availableForWrite() >= llen) {
        Serial.write((const uint8_t *) line, (size_t) llen);
    }

    // LED: green >= 20 fps, yellow >= 8, red below (written on change only).
    static uint32_t led = 999999;
    const uint32_t want = (fps >= 8) ? 0x00FF00 : (fps >= 4) ? 0xFFA500 : 0xFF0000;
    if (want != led) {
        led = want;
        k10.rgb->write(0, (want >> 16) & 0xFF, (want >> 8) & 0xFF, want & 0xFF);
        k10.rgb->show();
    }
}
