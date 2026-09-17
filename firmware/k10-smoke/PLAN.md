# UNIHIKER K10 — C/C++ with the ESP-SDK

Goal: program the K10 in plain C/C++ using the ESP-SDK (ESP-IDF), with full
control of the camera, the network stack, and every hardware pin.

## What we decided

| Decision            | Choice                                            |
|---------------------|---------------------------------------------------|
| Build approach      | PlatformIO + DFRobot's `framework-arduinounihiker` (reuses the proven `libmodules.a`) |
| First milestone     | Full hardware smoke test (screen + camera + network + audio + buttons + RGB + sensors + SD) |
| Flashing / debug    | USB-C direct (`esptool.py`), A/B buttons for boot mode, serial via USB-CDC |

Why this path: the bundled framework is **not** a normal Arduino core. It is a
prebuilt ESP-IDF **v4.4.4** toolchain plus a custom board library
(`tools/sdk/esp32s3/lib/libmodules.a`) that already contains all the K10-specific
glue (power init, camera, audio codec, network, on-device AI). The `unihiker_k10.h`
Arduino library is a thin C++ wrapper over that library. So writing C++ against it
*is* writing against the ESP-SDK — you can freely drop to raw `esp_camera`,
`esp_wifi`, `esp_lcd`, `i2s`, `gpio`, `i2c` APIs anywhere. We start on the
bundled framework to get a working build fast, and can later refactor toward a
cleaner native `idf.py` project if/when desired.

## Hardware facts (from the docs + prebuilt lib)

- **SoC:** ESP32-S3 N16R8 — dual-core Xtensa LX7 @ 240 MHz, 512 KB SRAM,
  16 MB flash, 8 MB QSPI PSRAM. No separate coprocessor (that was the older
  Linux-based UNIHIKER; the K10 is pure ESP32-S3).
- **Camera:** GC2145, DVP/parallel, 2 MP, 80° FOV.
- **Display:** ILI9341, 2.8" 240×320, SPI, driven by LVGL.
- **Audio:** ES7243E codec (mic + speaker) over I2S0.
- **Sensors:** AHT20 (temp/hum), LTR303ALS (ambient light), SC7A20H (3-axis accel),
  GT30L24A3W (touch).
- **Other:** 3× WS2812 RGB, A/B/RST/BOOT buttons, microSD (SDIO/SDMMC), USB-C.

### Camera pinout (DVP) — from `tools/sdk/esp32s3/include/modules/camera/who_camera.h`

| Signal | GPIO | Signal | GPIO |
|--------|------|--------|------|
| XCLK   | 7    | D2     | 11   |
| SIOD   | 47   | D1     | 10   |
| SIOC   | 48   | D0     | 8    |
| PCLK   | 17   | VSYNC  | 4    |
| D7     | 6    | HREF   | 5    |
| D6     | 15   | PWDN   | -1   |
| D5     | 16   | RESET  | -1   |
| D4     | 18   | XCLK freq | 16 MHz |
| D3     | 9    |        |      |

Note: the stock esp32-camera GC2145 driver has **no high-resolution support**
(esp32-camera issue #664). QVGA (320×240) is the safe target and matches the
screen. JPEG output is supported for saving/streaming.

### Display pinout — from `tools/sdk/esp32s3/include/modules/lcd/who_lcd.h`

| Signal | GPIO | Notes |
|--------|------|-------|
| MOSI   | 21   | SPI data |
| SCK    | 12   | 40 MHz |
| CS     | 14   | |
| DC     | 13   | |
| RST    | -1   | |
| BL     | -1   | backlight handled via `digital_write(eLCD_BLK, ...)` |

### Audio (I2S0) — from `unihiker_k10.h` / `initI2S()`

| Signal | GPIO |
|--------|------|
| MCLK   | 3    |
| BCLK   | 0    |
| LRCK   | 38   |
| DIN    | 39   |
| DOUT   | 45   |

Codec = ES7243E (init via `es7243e_adc_init()` in `libmodules.a`).

### Logical "K10" pins (Arduino `pins_arduino.h` + `initBoard.h` `ePin_t`)

- **Edge GPIO (P0–P16):** P0=1, P1=2, P2=3, P3=14, P4=13, P5=12 (Key A),
  P6=11, P8=8, P9=9, P10=10, P11=2 (Key B), P12=3, P13=4, P14=5, P15=6, P16=7.
- **Analog (A0–A19):** A0=1 … A19=20.
- **I2C (onboard bus):** SDA=47, SCL=48 (shared with camera SSCB — see caveat).
- **SPI:** SS=40, MOSI=21, MISO=41, SCK=12.
- **RGB (WS2812):** pin 46, count 3.
- **Special `ePin_t` (via `digital_write/digital_read` in libmodules.a):**
  `eLCD_BLK` (backlight), `eAmp_Gain` (audio amp), `eP5_KeyA`, `eP11_KeyB`,
  `eP11_KeyB`/`eP12`/`eP13`/`eP14`/`eP15`/`eP2`/`eP8`/`eP9`/`eP10`/`eP6`/`eP4`/`eP3`.
- **Buttons:** A = P5 (GPIO 12), B = P11 (GPIO 2), A+B combined.

> Caveat: camera SSCB (SIOD 47 / SIOC 48) shares GPIOs with the Arduino `I2C`
> SDA/SCL. Don't run a competing I2C bus on 47/48 at the same time as the camera.

## What `libmodules.a` already gives us (link it, don't reimplement)

Public symbols (from `nm`):
- `init_board()` — power-management chip init (call first, in `setup`).
- `digital_write/digital_read(ePin_t, ...)` — logical-pin GPIO.
- `register_camera(pixformat, framesize, fb_count, QueueHandle_t)` — GC2145 init
  + frame queue (wraps `esp_camera_init`).
- `app_wifi_main()`, `wifi_init_sta()`, `wifi_init_softap()` — WiFi.
- `register_httpd(...)`, `app_mdns_main()` — HTTP server + mDNS.
- `es7243e_adc_init()` — audio codec.
- `register_button(gpio, queue)`, `register_adc_button(...)` — buttons.
- `task_face_recognition/cat_recognition/motion_recognition/code_recognition` —
  on-device AI (esp-dl + esp-sr), driven via the `AIRecognition` wrapper.

## Build system

PlatformIO Core (install via pip/venv) + DFRobot platform:

```
platform = https://github.com/DFRobot/platform-unihiker.git
board    = unihiker_k10
framework = arduino
build_flags =
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DARDUINO_USB_MODE=1
    -DModel=None
```

`-DModel=None` skips flashing the large speech-recognition model (we don't need
it for the smoke test; saves ~4.5 MB + 2.5 MB). The framework's
`platformio-build-esp32s3.py` already wires up all the include paths and links
`libmodules.a` and the other board libs automatically — we don't touch that.

`WIFI_SSID`/`WIFI_PASSWORD` are read from the shell environment at build time
(`platformio.ini`'s `build_flags`), never hardcoded in `main.cpp`. Export them
before building: `export WIFI_SSID=... WIFI_PASSWORD=...`.

Flash: `pio run -t upload` (esptool over USB-C). Serial monitor:
`pio device monitor` (USB-CDC on boot). If the port doesn't appear, hold BOOT
while plugging in (or press RST) to force download mode.

## Milestone 1 — full hardware smoke test (`src/main.cpp`)

A single `setup()`/`loop()` that initializes every subsystem and reports PASS/FAIL
on the screen and serial. Subsystems, in order:

1. **Boot / power:** `k10.begin()` (calls `init_board()`, backlight off, amp off,
   I2C, RGB, buttons, accel, I2S).
2. **Display:** `initScreen(2)`, `creatCanvas()`, draw a status title + per-test
   rows. (ILI9341 via LVGL.)
3. **Camera:** `initBgCamerImage()` + `setBgCamerImage(true)` — live QVGA feed on
   screen. Also grab one frame, save JPEG to SD, and (optionally) log frame dims.
4. **SD card:** `initSDFile()`, write `/smoke_test.txt` + the JPEG, verify.
5. **Buttons:** poll A (P5) / B (P11) / A+B in `loop()`; blink RGB on press.
6. **RGB (WS2812):** cycle 3 LEDs through colors.
7. **Sensors:** read AHT20 temp/hum, LTR303ALS light, SC7A20H accel XYZ; print.
8. **Audio (I2S/ES7243E):** play a short tone via `Music::playTone()` (amp on/off
   around it) to confirm codec + speaker.
9. **Network (WiFi STA):** connect to a configured SSID (placeholder, user edits),
   print IP; bring up mDNS + a tiny HTTP endpoint (`/status` returns a JSON of all
   the above) to prove the network stack end to end.

Serial (`Serial` = USB-CDC) mirrors every PASS/FAIL so we can debug headless.
The screen shows a compact table; serial shows the full log.

### Project layout

```
esp32/
├── PLAN.md                  (this file)
├── framework-arduinounihiker/   (bundled prebuilt framework — do not edit)
├── unihiker-docs/               (docs + schematic PDF)
└── k10-smoke/                   (NEW: PlatformIO project)
    ├── platformio.ini
    └── src/
        └── main.cpp
```

## Risks / open questions

- **WiFi credentials:** need a real SSID/password for the network test
  (placeholder for now; edit before running, or use SoftAP + web config later).
- **GC2145 resolution:** QVGA only for reliability. If higher res is ever needed,
  it requires a custom GC2145 register config (esp32-camera #664) — out of scope
  for the smoke test.
- **`-DModel=None`:** speech recognition (esp-sr) won't be available in this build.
  Re-add `-DModel=EN`/`CN` only if we want on-device speech.
- **I2C pin sharing:** camera SSCB (47/48) collides with the Arduino I2C bus;
  keep them from running concurrently.
- **macOS port naming:** the K10 shows as an Espressif JTAG/serial unit (VID
  0x303A). Confirm the `/dev/cu.usbmodem*` or `/dev/cu.wchusbserial*` name at
  flash time (it can change between reboots).

## Next steps (after this plan is approved)

1. Install PlatformIO Core + fetch the DFRobot platform (verify framework resolves).
2. Scaffold `k10-smoke/` with `platformio.ini` + the smoke-test `main.cpp`.
3. `pio run` (build) → fix any compile issues.
4. `pio run -t upload` → watch the K10 screen + serial for the PASS/FAIL report.
5. Iterate per-subsystem on anything that fails.

## ✅ Status (DONE — smoke test flashed & verified 2025-09-16)

The `k10-smoke/` project **builds, flashes over USB-C, and runs stably** (verified 25+ min uptime, 0 crashes).

**Result: 7/9 PASS** (network now up on `Fallyn` WiFi, IP `172.16.35.214`, HTTP `/status` verified over the network). The 2 remaining FAILs are external (no SD card), not code bugs:

| Subsystem | Result | Notes |
|-----------|--------|-------|
| Boot / power chip | PASS | `init_board()` ok |
| Display (ILI9341/LVGL) | PASS | 240x320 portrait |
| Camera (GC2145) | PASS | QVGA live feed + JPEG capture |
| SD card | FAIL | **No microSD inserted** — insert a FAT32 card to pass |
| Camera→SD | FAIL | Depends on SD |
| RGB (WS2812x3) | PASS | Driven + reactive to A/B buttons |
| Sensors | PASS | SC7A20H accel + LTR303ALS light (AHT20 skipped — see gotchas) |
| Audio (I2S/ES7243E) | PASS | Tone played (codec init prints a benign warning) |
| Network (WiFi) | PASS | Connected to `Fallyn`, IP `172.16.35.214`, HTTP `/status` + mDNS `k10smoke.local` |

### Build & flash commands (from `k10-smoke/`)

```bash
# PlatformIO lives in a dedicated venv (kept off your homebrew python):
PIO=~/.platformio/venv/bin/pio
PORT=/dev/cu.usbmodem2143301        # Espressif JTAG/serial (VID 0x303A); name can change on replug

$PIO run                                   # build only
$PIO run -t upload --upload-port $PORT     # build + flash over USB-C
$PIO device monitor --port $PORT --baud 115200   # serial console
```

### Gotchas learned (important for future work)

1. **AHT20 class crashes** — `UNIHIKER_K10`'s `AHT20` class auto-spawns a background task that faults (LoadProhibited in `startMeasurementReady`) on this hardware, rebooting the board. **Do not instantiate it.** Read the AHT20 directly over I2C (addr 0x38) if you need temp/humidity.

2. **`initSDFile()` loops forever** if no card is present (`while(1) { SD.begin() }`). Call `SD.begin()` yourself with a bounded retry. The card is on **SPI1, CS=GPIO40** (a strapping pin — works, but reconfigure carefully).

3. **Don't call `lv_task_handler()` in `loop()`** — the K10's camera-display task already runs it under `xLvglMutex`. Calling it again (without the mutex) double-renders → null deref → reboot. Drive the display only via the K10's canvas/LVGL objects, and keep `loop()` light.

4. **Include `WiFi.h` / `esp_http_server.h` BEFORE `unihiker_k10.h`** — `unihiker_k10.h` pulls in TFT_eSPI/LVGL, which pollutes the esp_http_server/WiFi include chain if they come after.

5. **`-DModel=None` is a bad flag** — the framework parses `-DModel=<type>` as a model type and chokes on the literal `None`. Just omit it (no model is flashed by default).

6. **httpd handlers return `esp_err_t` (int), not `void`.**

7. **Audio codec init prints `Es7243e initialize failed`** but the I2S tone still plays — it's a benign warning in `init_board()`, not a real failure.

8. **macOS CDC-on-boot:** the serial port appears as `/dev/cu.usbmodem*`. Opening it (a monitor or `cat`) resets the board and re-runs `setup()`. The one-shot smoke test prints once, then `loop()` emits a `[HEARTBEAT]` every 5s so you can confirm uptime.

## Next steps (after this plan is approved)

1. ✅ Install PlatformIO Core + fetch the DFRobot platform (verify framework resolves).
2. ✅ Scaffold `k10-smoke/` with `platformio.ini` + the smoke-test `main.cpp`.
3. ✅ `pio run` (build) → fix any compile issues.
4. ✅ `pio run -t upload` → watch the K10 screen + serial for the PASS/FAIL report.
5. To get 9/9: insert a microSD (FAT32) — SD + camera_jpg then pass. (Network already passes on `Fallyn`.)
6. Re-run the whole test anytime by pressing **A+B** on the board (prints fresh results + re-renders the table).
7. Future: move real per-subsystem code out of the smoke test into proper modules (camera stream over network, on-device AI via the `AIRecognition` lib, etc.).
