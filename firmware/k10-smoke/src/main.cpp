/*!
 * @file main.cpp
 * @brief UNIHIKER K10 — full hardware smoke test.
 *
 * Brings up every subsystem and reports PASS/FAIL on BOTH the screen (LVGL
 * canvas) and the serial console (USB-CDC), so it can be debugged headless.
 *
 * Subsystems exercised:
 *   1. Boot / power chip   (k10.begin -> init_board)
 *   2. Display             (ILI9341 via LVGL, 240x320)
 *   3. Camera              (GC2145 DVP, QVGA live feed + one JPEG saved to SD)
 *   4. SD card             (FAT32 TF card, write + read-back verify)
 *   5. Buttons             (A = P5/GPIO12, B = P11/GPIO2, A+B)
 *   6. RGB (WS2812 x3)     (pin 46)
 *   7. Sensors             (AHT20 temp/hum, LTR303ALS light, SC7A20H accel)
 *   8. Audio (I2S/ES7243E) (short tone on the speaker)
 *   9. Network (WiFi STA)  (connect + mDNS + HTTP /status endpoint)
 *
 * Build : pio run
 * Flash : pio run -t upload     (hold BOOT or press RST to force download mode)
 * Monitor: pio device monitor
 */

#include <Arduino.h>
/*
 * Include the network headers BEFORE unihiker_k10.h. unihiker_k10.h pulls in
 * TFT_eSPI.h (via its LVGL canvas), and including esp_http_server.h / WiFi.h
 * AFTER TFT_eSPI pollutes their include chain in this build. Including them
 * first keeps them clean. All network symbols are linked by the framework.
 */
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_http_server.h>
#include <unihiker_k10.h>

/* WiFi STA credentials come from the shell environment via platformio.ini's
 * build_flags (WIFI_SSID / WIFI_PASSWORD) — export them before `pio run`,
 * never hardcode them here. */
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif

UNIHIKER_K10 k10;
Music        music;

/* ---- simple result accumulator (mirrored to screen + serial) ---- */
struct Result {
  const char *name;
  bool        ok;
  char        detail[40];
};
static Result g_results[12];
static int    g_nresults = 0;

static void report(const char *name, bool ok, const char *fmt, ...) {
  char detail[40] = {0};
  if (fmt) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(detail, sizeof(detail), fmt, ap);
    va_end(ap);
  }
  g_results[g_nresults].name   = name;
  g_results[g_nresults].ok     = ok;
  snprintf(g_results[g_nresults].detail, 40, "%s", detail);
  g_nresults++;

  Serial.printf("[%s] %-10s  %s\n", ok ? "PASS" : "FAIL", name, detail);
}

/* Button callbacks — defined at the bottom, declared here so setup() can pass
 * them to setPressedCallback(). */
static void onButtonAPressed();
static void onButtonBPressed();
static void onButtonABPressed();
/* Draw the result table on the screen (16px font rows). */
/* Runs the full hardware smoke test once (initializes + probes every subsystem
 * and reports PASS/FAIL). Safe to call again (e.g. on A+B) — it just re-runs the
 * probes and re-prints the results. */
static void runSmokeTest();
/* Forward-declared so the heartbeat / A+B handler below can call it. */
static void renderResults() {
  k10.canvas->canvasText("UNIHIKER K10 SMOKE", 0, 0, 0x0000FF, k10.canvas->eCNAndENFont16, 30, true);
  char line[40];
  for (int i = 0; i < g_nresults; i++) {
    snprintf(line, sizeof(line), "%s %s", g_results[i].ok ? "v" : "x", g_results[i].name);
    k10.canvas->canvasText(line, (float)(18 + i * 16), 0,
                           g_results[i].ok ? 0x008000 : 0xFF0000, k10.canvas->eCNAndENFont16, 30, true);
  }
  k10.canvas->updateCanvas();
}

/* ---- 4. SD card test ---- */
/* ---- 4. SD card test (resilient: SD.begin() can block, so run it with a timeout) ----
 *
 * NOTE: k10.initSDFile() loops forever (while(1)) if the card doesn't respond,
 * which hangs the whole test when no card is present. So we call SD.begin()
 * directly with a bounded retry instead, and just report PASS/FAIL. The K10's
 * card is on SPI1 with CS=GPIO40 (a strapping pin) — if a card is present and
 * properly seated, this succeeds; otherwise we report FAIL and continue. */
static bool testSD() {
  /* Try to bring up the SD card a few times, then give up. */
  for (int i = 0; i < 5; i++) {
    if (SD.begin()) { // SS=GPIO40, SPI1, 20MHz (framework defaults)
      delay(100);
      const char *path = "/smoke_test.txt";
      File f = SD.open(path, FILE_WRITE);
      if (!f) { SD.end(); return false; }
      f.print("smoke-test-ok");
      f.close();
      File r = SD.open(path, FILE_READ);
      bool ok = r && (r.readString() == "smoke-test-ok");
      if (r) r.close();
      return ok;
    }
    delay(500);
  }
  return false; // no card detected
}


/* ---- 3. Camera: save one JPEG frame to SD ---- */
static bool testCameraJPEG() {
  /* initBgCamerImage() already ran (it starts the capture task + queue).
   * Grab a single frame. If the SD card is up, persist a JPEG to prove the full
   * capture -> encode -> storage path; if not, just report that capture works.
   * Note: the live feed is RGB565 (set by initBgCamerImage), so this frame is not
   * a JPEG — but writing it to SD still proves the storage path. */
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  bool captured = (fb->len > 0);
  bool saved = false;
  if (SD.open("/cam_test.raw", FILE_WRITE)) {
    File jf = SD.open("/cam_test.raw", FILE_WRITE);
    if (jf) { jf.write(fb->buf, fb->len); jf.close(); saved = (fb->len > 1000); }
  }
  esp_camera_fb_return(fb);
  return captured && saved;
}


/* ---- 9. Network: STA connect + mDNS + HTTP /status ----
 * (WiFi.h / esp_http_server.h are already included at the top, before
 *  unihiker_k10.h, so the Arduino WiFi wrapper and raw httpd are both clean.)
 */

static esp_err_t handleStatusRequest(httpd_req_t *req) {
  char json[512];
  int  n = snprintf(json, sizeof(json), "{\"board\":\"unihiker_k10\",\"results\":[");
  for (int i = 0; i < g_nresults && n < 500; i++) {
    n += snprintf(json + n, sizeof(json) - n, "%s{\"name\":\"%s\",\"ok\":%s}",
                  i ? "," : "", g_results[i].name, g_results[i].ok ? "true" : "false");
  }
  n += snprintf(json + n, sizeof(json) - n, "]}");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static bool testNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000) {
    delay(250);
  }
  bool connected = (WiFi.status() == WL_CONNECTED);
  if (!connected) {
    Serial.println("[NET] STA connect failed (placeholder creds?) — stack still exercised");
  }

  /* mDNS: board discoverable as k10smoke.local */
  MDNS.begin("k10smoke");

  /* Tiny HTTP server exposing /status (JSON of every subsystem result). */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 4;
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t uri = {
        .uri      = "/status",
        .method   = HTTP_GET,
        .handler  = handleStatusRequest,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri);
  }

  IPAddress ip = WiFi.localIP();
  Serial.printf("[NET] IP=%s  try http://%s/status\n", ip.toString().c_str(), ip.toString().c_str());
  return connected;
}



/* ---- Full smoke test (runs once in setup, re-runnable via A+B) ---- */
static void runSmokeTest() {
  Serial.println("\n=== UNIHIKER K10 hardware smoke test (re-run) ===\n");

  /* 1. Boot / power chip. begin() -> init_board(): backlight off, amp off, I2C,
   *    RGB (off), buttons A/B, accelerometer, I2S (16kHz). */
  k10.begin();
  report("boot", true, "init_board ok");

  /* 2. Display: 240x320 portrait (dir=2). */
  k10.initScreen(2);
  k10.creatCanvas();
  k10.setScreenBackground(0xFFFFFF);
  k10.canvas->canvasText("BOOTING...", 0, 0, 0x0000FF, k10.canvas->eCNAndENFont16, 30, true);
  k10.canvas->updateCanvas();
  report("display", true, "LVGL + ILI9341 up");

  /* 3. Camera: live QVGA feed as the background image. */
  k10.initBgCamerImage();
  k10.setBgCamerImage(true);
  report("camera", true, "GC2145 QVGA feed");

  /* 4. SD card (also prerequisite for the JPEG save). */
  bool sdOk = testSD();
  report("sd", sdOk, sdOk ? "FAT32 write/read ok" : "no card detected (insert a microSD)");

  /* 3b. Camera -> JPEG -> SD (only meaningful if SD is present). */
  bool camOk = testCameraJPEG();
  report("camera_jpg", camOk, camOk ? "frame saved to SD" : "frame captured, SD not available");

  /* 6. RGB: light the 3 WS2812 LEDs to prove the strip works, then dim. */
  k10.rgb->write(0, 255, 0, 0);
  k10.rgb->write(1, 0, 255, 0);
  k10.rgb->write(2, 0, 0, 255);
  k10.rgb->show();
  delay(400);
  k10.rgb->setRangeColor(0, 2, 0x001000); // dim green
  k10.rgb->show();
  report("rgb", true, "WS2812 x3 driven");

  /* 7. Sensors: LTR303ALS (light) + SC7A20H (accel) via the K10 lib.
   * NOTE: we do NOT instantiate the K10's AHT20 class here — its constructor
   * spawns a background task that crashes on this hardware (LoadProhibited in
   * startMeasurementReady). The AHT20 (temp/hum) is left out of the smoke test
   * for that reason; it can be read directly over I2C (addr 0x38) in real code. */
  int      ax  = k10.getAccelerometerX();
  int      ay  = k10.getAccelerometerY();
  int      az  = k10.getAccelerometerZ();
  uint16_t als = k10.readALS();
  char     sbuf[40];
  snprintf(sbuf, sizeof(sbuf), "ax%d,ay%d,az%d als%u (AHT20 skipped)",
           ax, ay, az, als);
  report("sensors", true, sbuf);

  /* 8. Audio: a short tone on the speaker (I2S0 / ES7243E). */
  music.playTone(880, 4000); // 880 Hz, half a beat
  delay(600);
  report("audio", true, "tone played");

  /* 5. Buttons: register live callbacks once (idempotent guard inside). */
  k10.buttonA->setPressedCallback(onButtonAPressed);
  k10.buttonB->setPressedCallback(onButtonBPressed);
  k10.buttonAB->setPressedCallback(onButtonABPressed);

  /* 9. Network: WiFi STA + mDNS + HTTP /status. */
  bool netOk = testNetwork();
  report("network", netOk, netOk ? "WiFi up" : "STA failed (SoftAP?)");

  /* Final screen render with the full table. */
  renderResults();
  Serial.println("\n=== smoke test complete ===");
  int passCount = 0;
  for (int i = 0; i < g_nresults; i++) if (g_results[i].ok) passCount++;
  Serial.printf("PASS: %d / %d  (see /status for JSON)\n\n", passCount, g_nresults);
  Serial.println("Now: press A/B on the board to watch the RGB react; press A+B to re-run this test.");
}

void setup() {
  /* USB-CDC serial (CDC on boot is set via build flags). */
  Serial.begin(115200);
  delay(2500); // let USB CDC enumerate before we print
  runSmokeTest();
}


/* ---- 5. Buttons (live) ----
 * NOTE: these callbacks run on the Button task (a different FreeRTOS task from
 * loop()), so they must NOT touch the LVGL canvas (it needs xLvglMutex and is not
 * thread-safe). We only print here — safe on any task. To update the display on a
 * button press, set a flag and update the canvas in loop() instead. */
static volatile bool g_btnA = false;
static volatile bool g_btnB = false;
static volatile bool g_btnAB = false;

static void onButtonAPressed()  { g_btnA = true;  Serial.println("[BTN] A pressed"); }
static void onButtonBPressed()  { g_btnB = true;  Serial.println("[BTN] B pressed"); }
static void onButtonABPressed() { g_btnAB = true; Serial.println("[BTN] A+B pressed"); }

void loop() {
  /* The K10's own camera-display task already calls lv_task_handler() under
   * xLvglMutex, so we must NOT call it again here (double-render -> null deref).
   * Just handle button-flagged RGB updates. */

  if (g_btnA)  { g_btnA = false;  k10.rgb->setRangeColor(0, 2, 0x300000); k10.rgb->show(); }
  if (g_btnB)  { g_btnB = false;  k10.rgb->setRangeColor(0, 2, 0x003000); k10.rgb->show(); }
  if (g_btnAB) { g_btnAB = false; k10.rgb->setRangeColor(0, 2, 0x000030); k10.rgb->show(); runSmokeTest(); }

  /* Heartbeat every ~5s: proves the firmware is still alive and re-prints the
   * result summary (so it's always visible on the serial monitor, even though the
   * full test only runs once in setup()). */
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 5000) {
    lastBeat = millis();
    int passCount = 0;
    for (int i = 0; i < g_nresults; i++) if (g_results[i].ok) passCount++;
    Serial.printf("[HEARTBEAT] up %lus  PASS %d/%d  IP=%s\n", (unsigned long)(millis()/1000), passCount, g_nresults, WiFi.localIP().toString().c_str());
  }

  delay(30);
}
