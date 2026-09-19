// k10-lk-track — FAST-9 corner detection + pyramidal Lucas-Kanade tracking
// on the UNIHIKER K10.
//
// Pipeline (one FreeRTOS task, core 0, priority 5 — LVGL is driven
// exclusively by it, same convention as k10-fast-corners):
//   1. pull an RGB565 240x320 PORTRAIT frame from the camera queue
//   2. convert to 8-bit grayscale (internal SRAM)
//   3. downsample 2x2 + blur -> the half-res detector buffer, which doubles
//      as pyramid level 0 (see lib/lktrack) — no extra copy
//   4. build pyramid levels 1-2 (quarter, eighth res) from level 0
//   5. lkt_track(): refine every already-tracked point from the previous
//      frame's pyramid to this one — only ever touches small patches around
//      each point, never the whole frame (see lib/lktrack/lktrack.h)
//   6. replenish: if the live track count drops below a floor, or every N
//      frames, run FAST-9 on level 0 and add new points (strongest first,
//      skipping anywhere already close to a live track)
//   7. draw: raw frame + trail + point markers into the display buffer, one
//      lv_task_handler() pass (skippable at runtime — see 's'/long-press)
//
// Streams its tracks + accelerometer over UDP to a host for visual odometry,
// the same way k10-fast-corners streams ORB features (see PLAN.md "Streaming
// to a host"). Serial 'd'/'D' still dump frames for offline tuning, same
// tools/capture.py workflow as the other K10 vision projects.
//
// Controls: A tap threshold +4, B tap threshold -4, A+B toggles trails, and
// A or B held >= LONG_PRESS_MS (600ms) toggles the display on/off — same
// action as the 's' serial command, and the way to buy back the ~75ms/frame
// SPI flush while streaming.
//
// Conventions from k10-fast-corners / k10-smoke (do NOT violate):
//   * never call lv_task_handler() outside the framework Canvas methods
//   * never touch the canvas or LVGL from button callbacks
//   * keep loop() light; guard every CDC write with availableForWrite()
//   * vTaskDelay(1) at the end of every pipeline iteration (TWDT)
//   * own the camera queue (register_camera), not initBgCamerImage()
//   * the K10 QVGA frame is 240x320 portrait, matches the screen 1:1 — no
//     rotation, camera coords == display coords
//   * include WiFi.h before unihiker_k10.h (TFT_eSPI pollutes the WiFi
//     include chain if it comes first — k10-smoke/PLAN.md gotcha 4)

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <unihiker_k10.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

#include "fastcorner.h"
#include "lktrack.h"
#include "k10image.h"
#include "k10stream.h"

#define CAM_W 240                 // native portrait frame from the K10 camera
#define CAM_H 320
#define DET_W (CAM_W / 2)         // pyramid level 0 = detector buffer (half res)
#define DET_H (CAM_H / 2)
#define L1_W  (DET_W / 2)
#define L1_H  (DET_H / 2)
#define L2_W  (L1_W / 2)
#define L2_H  (L1_H / 2)
#define SCR_W 240                 // portrait display — matches CAM_W x CAM_H 1:1
#define SCR_H 320

#define TRAIL_COLOR  0x07FF        // RGB565 cyan
#define MARK_COLOR   0xF81F        // RGB565 magenta
#define HUD_COLOR    0x00FFFF      // 0xRRGGBB cyan (canvas text takes 24-bit)

#define MIN_SPACING       4        // Chebyshev px (level 0), same scale as FC_MIN_SPACING
#define REPLENISH_FLOOR   20       // re-detect when live tracks drop below this
#define REPLENISH_PERIOD  30       // ...or at least this often (frames, ~4s @ 8fps)

UNIHIKER_K10 k10;

static lv_obj_t    *feed_img = NULL;
static lv_img_dsc_t feed_dsc;
static QueueHandle_t xQueueCam = NULL;

static uint8_t  *grayBuf   = NULL;          // CAM_W*CAM_H, PSRAM (see setup())
static uint8_t  *detBuf[2] = {NULL, NULL};  // DET_W*DET_H, pyramid level 0, ping-pong
static uint8_t  *detTmp    = NULL;          // DET_W*DET_H blur scratch
static uint8_t  *l1Buf[2]  = {NULL, NULL};  // pyramid level 1, ping-pong
static uint8_t  *l2Buf[2]  = {NULL, NULL};  // pyramid level 2, ping-pong
static uint16_t *dispBuf   = NULL;          // SCR_W*SCR_H, PSRAM

static lkt_pyramid_t pyr[2];
static lkt_state_t   trackState;
static fc_result_t   corners;

// Tunables — written by button callbacks (button task), read by the pipeline.
static volatile int32_t g_threshold  = FC_THRESHOLD_DEFAULT;
static volatile int32_t g_stride     = 1;
static volatile int32_t g_trails_on  = 1;
static volatile int32_t g_screen_on  = 1;   // 's' / long-press — see PLAN.md

// Stats (written by pipeline task, read by loop()).
static volatile uint32_t g_fps          = 0;
static volatile uint32_t g_detect_us    = 0;
static volatile uint32_t g_track_us     = 0;
static volatile uint32_t g_draw_us      = 0;
static volatile int32_t  g_active_cnt   = 0;
static volatile int32_t  g_added_cnt    = 0;
static volatile int32_t  g_cand_cnt     = 0;
static volatile int32_t  g_lost_eig     = 0;  // cumulative — see the 'r' serial command to reset
static volatile int32_t  g_lost_ssd     = 0;
static volatile int32_t  g_lost_bounds  = 0;

// Runtime-tunable LK lost gates (see lib/lktrack "UNVERIFIED ON REAL
// FOOTAGE" — these need tuning against real device footage, and a reflash
// per attempt would make that painfully slow). '[' / ']' lower/raise the
// eigenvalue gate over serial; see loop().
static float g_min_eig = LKT_MIN_EIG;
static float g_max_ssd = LKT_MAX_MEAN_SSD;

// Frame dump: set by loop() from a serial command, serviced by the pipeline
// task (0 = idle, 'd' = detector luma, 'D' = raw RGB565).
static volatile int32_t g_dump_req = 0;

// ---------------------------------------------------------------------------
// WiFi UDP streaming — LK tracks + accelerometer. Same shape as
// k10-fast-corners' ORB stream (see its PLAN.md for why UDP, why two ports,
// and why unicast to STREAM_HOST_IP rather than subnet broadcast, which
// measured DOA on this network):
//   - TRCK: one packet per processed frame, from the pipeline task.
//   - ACCL: from loop() at ACCEL_PERIOD_MS, independent of the camera.
// Ports: TRCK 5007 (5005 is taken by ORBF there), ACCL 5006 — deliberately
// the SAME accel port as k10-fast-corners so one host-side receiver can
// listen identically for either firmware.
// ---------------------------------------------------------------------------

// WIFI_SSID / WIFI_PASSWORD / STREAM_HOST_IP come from the shell environment
// via platformio.ini's build_flags — export them before `pio run`, never
// hardcode them here.
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef STREAM_HOST_IP
#define STREAM_HOST_IP ""   // dotted-quad string, e.g. "172.16.35.211"
#endif
#define TRACK_STREAM_PORT 5007
#define ACCEL_STREAM_PORT 5006
#define ACCEL_PERIOD_MS   20     // 50 Hz, same as k10-fast-corners

static WiFiUDP  udpTrack;
static WiFiUDP  udpAccel;
static IPAddress g_dest;
static volatile bool g_wifi_up = false;
static uint32_t g_track_seq = 0;
static uint32_t g_accel_seq = 0;
static volatile uint32_t g_track_send_ok = 0, g_track_send_fail = 0;

// ---------------------------------------------------------------------------
// Buttons — pattern from k10-fast-corners: callbacks only touch atomics,
// never LVGL. A/B taps keep adjusting the detector threshold; the screen
// toggle rides on a long-press of either (the Button class has no built-in
// long-press, so pressed() just stamps a timestamp and release() decides by
// how long it was held).
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
    g_trails_on = !g_trails_on;
}

// ---------------------------------------------------------------------------
// Frame dump (base64 over CDC) — identical helper to k10-fast-corners; see
// its PLAN.md "Capturing a real frame" for the tools/capture.py workflow.
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

// ---------------------------------------------------------------------------
// Display helpers — draw straight into dispBuf (no LVGL cost per pixel).
// ---------------------------------------------------------------------------

static inline void put_px(uint16_t *buf, int x, int y, uint16_t color) {
    if ((unsigned) x < (unsigned) SCR_W && (unsigned) y < (unsigned) SCR_H) {
        buf[(size_t) y * SCR_W + x] = color;
    }
}

// Stamp a 5x5 plus marker, clipped per-pixel (a track's marker can sit near
// or past the edge right before it's dropped as out-of-bounds next frame).
static void stamp_marker(uint16_t *buf, int cx, int cy, uint16_t color) {
    for (int o = -2; o <= 2; o++) {
        put_px(buf, cx + o, cy, color);
        put_px(buf, cx, cy + o, color);
    }
}

// Bresenham, clipped per-pixel — trail segments can run off-screen once a
// track nears the frame edge, unlike stamp_marker's bounded plus shape.
static void draw_line(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put_px(buf, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Draw one track's trail (oldest -> newest, level-0 coords scaled x2 to
// display coords) plus a marker at its current head position.
static void draw_track(uint16_t *buf, const lkt_track_t *t) {
    const int n = t->trail_count;
    if (n == 0) return;
    int px = 0, py = 0;
    for (int k = 0; k < n; k++) {
        const int idx = (n < LKT_TRAIL_LEN) ? k : (t->trail_head + k) % LKT_TRAIL_LEN;
        const int x = t->trail_x[idx] * 2, y = t->trail_y[idx] * 2;
        if (k > 0 && g_trails_on) draw_line(buf, px, py, x, y, TRAIL_COLOR);
        px = x; py = y;
    }
    stamp_marker(buf, px, py, MARK_COLOR);
}

// ---------------------------------------------------------------------------
// Pipeline task (core 0)
// ---------------------------------------------------------------------------

static void pipeline_task(void *arg) {
    (void) arg;
    uint32_t win_start = micros();
    uint32_t win_frames = 0;
    uint32_t frame_no = 0;
    bool have_prev = false;
    int cur_idx = 0;

    for (;;) {
        camera_fb_t *fb;
        if (xQueueReceive(xQueueCam, &fb, portMAX_DELAY) != pdTRUE || !fb) {
            if (fb) esp_camera_fb_return(fb);
            continue;
        }

        const uint16_t *rgb = (const uint16_t *) fb->buf;
        const int ci = cur_idx;
        const int pi = 1 - ci;

        k10_to_grayscale(rgb, grayBuf, CAM_W * CAM_H);
        fast_downsample2x2(grayBuf, detBuf[ci], CAM_W, CAM_H);
        fast_blur3x3(detBuf[ci], detTmp, DET_W, DET_H);
        lkt_pyramid_build(&pyr[ci], detBuf[ci], DET_W, DET_H, l1Buf[ci], l2Buf[ci]);

        const uint32_t t0 = micros();
        if (have_prev) {
            lkt_lost_stats_t lost = {0, 0, 0};
            lkt_track(&trackState, &pyr[pi], &pyr[ci], &lost);
            g_lost_eig += lost.lost_eig;
            g_lost_ssd += lost.lost_ssd;
            g_lost_bounds += lost.lost_bounds;
        }
        g_track_us = micros() - t0;
        g_active_cnt = lkt_active_count(&trackState);

        // Replenish: only run the detector (the one step that scans every
        // pixel of the frame) when the track count actually needs it.
        g_added_cnt = 0;
        g_cand_cnt = 0;
        if (g_active_cnt < REPLENISH_FLOOR || (frame_no % REPLENISH_PERIOD) == 0) {
            const uint32_t td0 = micros();
            fast_corner_detect(detBuf[ci], DET_W, DET_H,
                               (int) g_threshold, (int) g_stride, &corners);
            g_detect_us = micros() - td0;
            g_cand_cnt = corners.candidates;

            int16_t xs[FC_MAX_CORNERS], ys[FC_MAX_CORNERS];
            for (int i = 0; i < corners.count; i++) {
                xs[i] = corners.corners[i].x;
                ys[i] = corners.corners[i].y;
            }
            g_added_cnt = lkt_add(&trackState, xs, ys, corners.count, MIN_SPACING);
            g_active_cnt = lkt_active_count(&trackState);
        }

        // Stream this frame's tracks (position/id/age, sub-pixel Q4) once the
        // tracker has settled — after tracking *and* replenishing, so a host
        // consumer sees the full live set, not the survivors before new seeds
        // were added. Timestamped now, like the ORB stream timestamps its own
        // detector buffer — close enough to the LK pass that just ran for a
        // host to co-timeline it against the 50 Hz accel stream.
        if (g_wifi_up) {
            const bool ok = k10stream_send_tracks(udpTrack, g_dest, TRACK_STREAM_PORT,
                                                 g_track_seq, &trackState,
                                                 DET_W, DET_H, micros());
            if (ok) g_track_send_ok++; else g_track_send_fail++;
        }

        // Draw: raw frame + every active track's trail + marker. Skippable at
        // runtime ('s' / long-press) — the SPI flush it drives is the frame-rate
        // ceiling (~75ms of a ~125ms period, same as k10-fast-corners), so
        // dropping the preview is what buys VO sessions real fps. Everything
        // above (detect/LK/stream) is untouched by the toggle.
        static int32_t prev_screen_on = 1;
        const int32_t screen_on = g_screen_on;
        if (screen_on && !prev_screen_on) {
            // The OFF branch paints an opaque full-screen rect on the canvas
            // layer, which sits above feed_img; without restoring
            // transparency the rest of it stays stuck over the live feed.
            k10.canvas->clearLocalCanvas(0, 0, SCR_W, SCR_H);
        }
        if (screen_on) {
            memcpy(dispBuf, rgb, (size_t) SCR_W * SCR_H * 2);
            for (int i = 0; i < LKT_MAX_TRACKS; i++) {
                if (trackState.tracks[i].active) draw_track(dispBuf, &trackState.tracks[i]);
            }

            const uint32_t t2 = micros();
            lv_obj_invalidate(feed_img);
            k10.canvas->canvasRectangle(0, 0, SCR_W, 20, 0x000000, 0x000000, true);
            char hud[48];
            snprintf(hud, sizeof(hud), "LK t=%ld s%ld %d trk +%ld",
                     (long) g_threshold, (long) g_stride, (int) g_active_cnt, (long) g_added_cnt);
            k10.canvas->canvasText(hud, 2, 3, HUD_COLOR, Canvas::eCNAndENFont16, 50, false);
            k10.canvas->updateCanvas();
            g_draw_us = micros() - t2;
        } else {
            g_draw_us = 0;
            if (prev_screen_on) {
                // One last draw on the off-transition, so the screen shows why
                // the preview stopped rather than freezing on a stale frame;
                // then no further LVGL work happens until it's switched back on.
                k10.canvas->canvasRectangle(0, 0, SCR_W, SCR_H, 0x000000, 0x000000, true);
                k10.canvas->canvasText("SCREEN OFF", 2, 140, HUD_COLOR,
                                       Canvas::eCNAndENFont16, 50, false);
                k10.canvas->canvasText("streaming...", 2, 160, HUD_COLOR,
                                       Canvas::eCNAndENFont16, 50, false);
                k10.canvas->updateCanvas();
            }
        }
        prev_screen_on = screen_on;

        const int32_t req = g_dump_req;
        if (req) {
            g_dump_req = 0;
            if (req == 'd') {
                dump_b64("GRAY", detBuf[ci], (size_t) DET_W * DET_H, DET_W, DET_H);
            } else {
                dump_b64("RGB565", (const uint8_t *) rgb,
                         (size_t) CAM_W * CAM_H * 2, CAM_W, CAM_H);
            }
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(1));  // TWDT: always yield once per frame

        have_prev = true;
        cur_idx = pi;
        frame_no++;
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
    Serial.println("== k10-lk-track ==");

    k10.begin();
    k10.initScreen(2);              // portrait 240x320

    feed_img = lv_img_create(lv_scr_act());
    lv_obj_set_pos(feed_img, 0, 0);
    lv_obj_set_size(feed_img, SCR_W, SCR_H);

    k10.creatCanvas();
    k10.canvas->canvasSetLineWidth(1);
    k10.setScreenBackground(0x000000);

    // LK does ~2300 random bilinear patch reads per tracked point per frame
    // (see lib/lktrack) — unlike fast_corner_detect's sequential scan, that
    // pattern is a latency cliff in PSRAM, not just a bandwidth hit. Report
    // where each pyramid buffer actually landed so a PSRAM fallback shows up
    // as "buffer X is in PSRAM" in the boot log, not as unexplained slowness.
    bool internal[2][3];  // [ping-pong index][detBuf, l1Buf, l2Buf]
    // grayBuf is scratch for one sequential pass (k10_to_grayscale, then
    // fast_downsample2x2 reads it once more) — PSRAM's sequential bandwidth
    // is fine here, unlike the pyramid buffers' random patch reads above, so
    // it goes straight to PSRAM rather than competing for internal SRAM.
    // This project's ping-pong pyramid (detBuf/l1Buf/l2Buf x2, ~30 KB more
    // than k10-fast-corners' single-buffered version) left only ~11 KB/7.6 KB
    // largest-block of internal SRAM free after WiFi.begin() claimed its own
    // ~68 KB — too little for the camera driver to keep capturing, so it
    // wedged permanently ("Failed to get the frame on time!" forever, fps=0)
    // the moment WiFi came up, even though k10-fast-corners' identical camera
    // config survives the same WiFi cost fine with more headroom to spare.
    // Freeing grayBuf's 75 KB restores comparable headroom (device-verified
    // 2026-09-17 — see PLAN.md "WiFi + camera bug").
    grayBuf = (uint8_t *) heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_SPIRAM);
    detTmp = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
    if (!detTmp) detTmp = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_SPIRAM);
    for (int i = 0; i < 2; i++) {
        detBuf[i] = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_INTERNAL);
        internal[i][0] = detBuf[i] != NULL;
        if (!detBuf[i]) detBuf[i] = (uint8_t *) heap_caps_malloc(DET_W * DET_H, MALLOC_CAP_SPIRAM);
        l1Buf[i] = (uint8_t *) heap_caps_malloc(L1_W * L1_H, MALLOC_CAP_INTERNAL);
        internal[i][1] = l1Buf[i] != NULL;
        if (!l1Buf[i]) l1Buf[i] = (uint8_t *) heap_caps_malloc(L1_W * L1_H, MALLOC_CAP_SPIRAM);
        l2Buf[i] = (uint8_t *) heap_caps_malloc(L2_W * L2_H, MALLOC_CAP_INTERNAL);
        internal[i][2] = l2Buf[i] != NULL;
        if (!l2Buf[i]) l2Buf[i] = (uint8_t *) heap_caps_malloc(L2_W * L2_H, MALLOC_CAP_SPIRAM);
    }
    dispBuf = (uint16_t *) heap_caps_malloc((size_t) SCR_W * SCR_H * 2, MALLOC_CAP_SPIRAM);

    bool ok = grayBuf && detTmp && dispBuf;
    for (int i = 0; i < 2; i++) ok = ok && detBuf[i] && l1Buf[i] && l2Buf[i];
    if (!ok) {
        Serial.println("buffer alloc failed");
        return;
    }
    for (int i = 0; i < 2; i++) {
        if (!internal[i][0] || !internal[i][1] || !internal[i][2]) {
            Serial.printf("WARNING: pyramid[%d] fell back to PSRAM (det=%d l1=%d l2=%d) "
                          "-- LK's random patch reads will be slow there\n",
                          i, internal[i][0], internal[i][1], internal[i][2]);
        }
    }

    memset(&feed_dsc, 0, sizeof(feed_dsc));
    feed_dsc.header.always_zero = 0;
    feed_dsc.header.w = SCR_W;
    feed_dsc.header.h = SCR_H;
    feed_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    feed_dsc.data_size = (size_t) SCR_W * SCR_H * 2;
    feed_dsc.data = (const uint8_t *) dispBuf;
    lv_img_set_src(feed_img, &feed_dsc);

    lkt_init(&trackState);

    xQueueCam = xQueueCreate(2, sizeof(camera_fb_t *));
    register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, xQueueCam);

    k10.buttonA->setPressedCallback(on_button_a_pressed);
    k10.buttonA->setUnPressedCallback(on_button_a_released);
    k10.buttonB->setPressedCallback(on_button_b_pressed);
    k10.buttonB->setUnPressedCallback(on_button_b_released);
    k10.buttonAB->setPressedCallback(on_button_ab);

    // WiFi + UDP streaming (LK tracks + accelerometer) to STREAM_HOST_IP.
    // Non-fatal on failure — detection/tracking/display all work with no
    // network at all, they just won't stream. Blocks up to 15s (same policy
    // as k10-fast-corners/k10-smoke).
    g_dest.fromString(STREAM_HOST_IP);
    g_wifi_up = k10stream_wifi_begin(WIFI_SSID, WIFI_PASSWORD, udpTrack, udpAccel);
    if (g_wifi_up) {
        Serial.printf("wifi up: %s -> %s  track->udp:%d  accel->udp:%d\n",
                      WiFi.localIP().toString().c_str(), g_dest.toString().c_str(),
                      TRACK_STREAM_PORT, ACCEL_STREAM_PORT);
    } else {
        Serial.println("wifi connect failed/timed out — streaming disabled, running local-only");
    }

    Serial.printf("ready. gray=%lu KB, det=%lu KB x2, disp=%lu KB\n",
                  (unsigned long) (CAM_W * CAM_H / 1024),
                  (unsigned long) (DET_W * DET_H / 1024),
                  (unsigned long) ((size_t) SCR_W * SCR_H * 2 / 1024));

    xTaskCreatePinnedToCore(pipeline_task, "lk_pipe", 8 * 1024,
                            NULL, 5, NULL, 0);
}

void loop() {
    while (Serial.available()) {
        const int c = Serial.read();
        if (c == 'd' || c == 'D') g_dump_req = c;
        if (c == 's') {
            g_screen_on = !g_screen_on;
            Serial.printf("screen %s\n", g_screen_on ? "ON" : "OFF");
        }
        if (c == 't') {
            g_trails_on = !g_trails_on;
            Serial.printf("trails %s\n", g_trails_on ? "ON" : "OFF");
        }
        // Runtime gate tuning (see lib/lktrack — LKT_MIN_EIG/LKT_MAX_MEAN_SSD
        // are unverified-on-real-footage estimates; bring-up needs to move
        // these against real corners without a reflash per attempt).
        if (c == '[' || c == ']') {
            g_min_eig *= (c == '[') ? 0.7f : 1.4f;
            lkt_set_gates(g_min_eig, -1.0f);
            Serial.printf("min_eig=%.2f\n", g_min_eig);
        }
        if (c == '{' || c == '}') {
            g_max_ssd *= (c == '{') ? 0.7f : 1.4f;
            lkt_set_gates(-1.0f, g_max_ssd);
            Serial.printf("max_ssd=%.2f\n", g_max_ssd);
        }
        if (c == 'r') {
            g_lost_eig = g_lost_ssd = g_lost_bounds = 0;
            Serial.println("lost counters reset");
        }
    }

    // Accelerometer stream: independent of, and much faster than, the camera
    // frame rate. getAccelerometerX/Y/Z() just read a value the vendor lib's
    // own background task already keeps current, so this never blocks.
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
    char line[256];
    const int llen = snprintf(line, sizeof(line),
                            "fps=%lu track=%lu us detect=%lu us draw=%lu us "
                            "t=%ld s%ld trk=%ld +%ld cand=%ld trails=%s scr=%s "
                            "lost(eig=%ld ssd=%ld bounds=%ld) gate(eig=%.1f ssd=%.1f) "
                            "wifi=%s udp_ok=%lu udp_fail=%lu\n",
                            (unsigned long) fps, (unsigned long) g_track_us,
                            (unsigned long) g_detect_us, (unsigned long) g_draw_us,
                            (long) g_threshold, (long) g_stride,
                            (long) g_active_cnt, (long) g_added_cnt, (long) g_cand_cnt,
                            g_trails_on ? "ON" : "OFF",
                            g_screen_on ? "ON" : "OFF",
                            (long) g_lost_eig, (long) g_lost_ssd, (long) g_lost_bounds,
                            (double) g_min_eig, (double) g_max_ssd,
                            g_wifi_up ? WiFi.localIP().toString().c_str() : "down",
                            (unsigned long) g_track_send_ok,
                            (unsigned long) g_track_send_fail);
    if (Serial.availableForWrite() >= llen) {
        Serial.write((const uint8_t *) line, (size_t) llen);
    }

    // LED: green >= 8 fps, yellow >= 4, red below (written on change only).
    static uint32_t led = 999999;
    const uint32_t want = (fps >= 8) ? 0x00FF00 : (fps >= 4) ? 0xFFA500 : 0xFF0000;
    if (want != led) {
        led = want;
        k10.rgb->write(0, (want >> 16) & 0xFF, (want >> 8) & 0xFF, want & 0xFF);
        k10.rgb->show();
    }
}
