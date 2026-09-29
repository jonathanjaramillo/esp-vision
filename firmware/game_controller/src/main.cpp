#include <Arduino.h>
#include <unihiker_k10.h>

#include "GameController.h"

namespace {

constexpr int SCREEN_W = 240;
constexpr int SCREEN_H = 320;
constexpr int PLOT_LEFT = 20;
constexpr int PLOT_RIGHT = 220;
constexpr int PLOT_TOP = 112;
constexpr int PLOT_BOTTOM = 300;
constexpr uint8_t HISTORY_SIZE = 4;
constexpr uint8_t TRAIL_SIZE = 36;
constexpr uint32_t POLL_INTERVAL_MS = 50;

UNIHIKER_K10 k10;
GameController controller;

struct Point {
  int16_t x;
  int16_t y;
};

Point trail[TRAIL_SIZE];
uint8_t trailCount = 0;
uint8_t trailNext = 0;
char pressHistory[HISTORY_SIZE][24] = {};
bool buttonWasDown[GameController::BUTTON_COUNT] = {};
uint8_t historyNext = 0;
uint32_t lastPollMs = 0;
uint8_t joystickX = 128;
uint8_t joystickY = 128;
bool controllerOnline = false;

const char *buttonName(GameController::Button button) {
  static const char *names[] = {"A", "B", "C", "D", "OK"};
  return names[(uint8_t)button];
}

void recordPress(GameController::Button button) {
  snprintf(pressHistory[historyNext], sizeof(pressHistory[historyNext]),
           "%s pressed", buttonName(button));
  historyNext = (historyNext + 1) % HISTORY_SIZE;
  Serial.printf("[BUTTON] %s pressed\n", buttonName(button));
}

void addTrailPoint(uint8_t x, uint8_t y) {
  // Scale 0..255 to the plot rectangle. Invert Y so high values appear higher.
  trail[trailNext].x = PLOT_LEFT + ((int32_t)x * (PLOT_RIGHT - PLOT_LEFT)) / 255;
  trail[trailNext].y = PLOT_BOTTOM - ((int32_t)y * (PLOT_BOTTOM - PLOT_TOP)) / 255;
  trailNext = (trailNext + 1) % TRAIL_SIZE;
  if (trailCount < TRAIL_SIZE) trailCount++;
}

void drawScreen() {
  Canvas *canvas = k10.canvas;
  canvas->canvasRectangle(0, 0, SCREEN_W, SCREEN_H, 0xFFFFFF, 0xFFFFFF, true);
  canvas->canvasText("Joystick controller", 4, 2, 0x000000,
                     Canvas::eCNAndENFont16, 38, false);

  char values[40];
  snprintf(values, sizeof(values), "X: %3u   Y: %3u", joystickX, joystickY);
  canvas->canvasText(values, 4, 22, 0x000000, Canvas::eCNAndENFont16, 38, false);

  canvas->canvasText(controllerOnline ? "Recent presses:" : "Controller not found",
                     4, 42, controllerOnline ? 0x000000 : 0xCC0000,
                     Canvas::eCNAndENFont16, 38, false);
  for (uint8_t row = 0; row < HISTORY_SIZE; row++) {
    const uint8_t index = (historyNext + row) % HISTORY_SIZE;
    const char *entry = pressHistory[index][0] ? pressHistory[index] : "-";
    canvas->canvasText(entry, 8, 60 + row * 14, 0x204080,
                       Canvas::eCNAndENFont16, 30, false);
  }

  canvas->canvasRectangle(PLOT_LEFT, PLOT_TOP,
                          PLOT_RIGHT - PLOT_LEFT + 1,
                          PLOT_BOTTOM - PLOT_TOP + 1,
                          0x202020, 0xF4F7FA, true);
  canvas->canvasLine((PLOT_LEFT + PLOT_RIGHT) / 2, PLOT_TOP,
                     (PLOT_LEFT + PLOT_RIGHT) / 2, PLOT_BOTTOM, 0xB0B0B0);
  canvas->canvasLine(PLOT_LEFT, (PLOT_TOP + PLOT_BOTTOM) / 2,
                     PLOT_RIGHT, (PLOT_TOP + PLOT_BOTTOM) / 2, 0xB0B0B0);

  for (uint8_t i = 1; i < trailCount; i++) {
    const uint8_t from = (trailNext + TRAIL_SIZE - trailCount + i - 1) % TRAIL_SIZE;
    const uint8_t to = (from + 1) % TRAIL_SIZE;
    canvas->canvasLine(trail[from].x, trail[from].y,
                       trail[to].x, trail[to].y, 0x0080FF);
  }
  if (trailCount) {
    const uint8_t latest = (trailNext + TRAIL_SIZE - 1) % TRAIL_SIZE;
    canvas->canvasCircle(trail[latest].x, trail[latest].y, 4,
                         0xD02020, 0xD02020, true);
  }

  canvas->canvasText("0", PLOT_LEFT - 10, PLOT_BOTTOM - 10, 0x303030,
                     Canvas::eCNAndENFont16, 16, false);
  canvas->canvasText("255", PLOT_RIGHT - 26, PLOT_TOP - 14, 0x303030,
                     Canvas::eCNAndENFont16, 20, false);
  canvas->updateCanvas();
}

void pollController() {
  uint8_t x;
  uint8_t y;
  if (controller.readAxes(x, y)) {
    joystickX = x;
    joystickY = y;
    controllerOnline = true;
    addTrailPoint(x, y);
  } else {
    controllerOnline = false;
  }

  for (uint8_t i = 0; i < GameController::BUTTON_COUNT; i++) {
    uint8_t event;
    if (!controller.readButtonEvent((GameController::Button)i, event)) {
      controllerOnline = false;
      buttonWasDown[i] = false;
      continue;
    }
    controllerOnline = true;
    if (event == GameController::PRESS_DOWN && !buttonWasDown[i]) {
      recordPress((GameController::Button)i);
    }
    buttonWasDown[i] = (event == GameController::PRESS_DOWN);
  }
}

} // namespace

void setup() {
  Serial.begin(115200);

  // K10.begin() initializes the shared I2C bus and board hardware. The camera
  // is intentionally left off because its control bus shares SDA/SCL pins.
  k10.begin();
  k10.initScreen(2);
  k10.creatCanvas();
  k10.setScreenBackground(0xFFFFFF);

  controller.begin(Wire, GameController::DEFAULT_I2C_ADDRESS);

  uint8_t x;
  uint8_t y;
  controllerOnline = controller.readAxes(x, y);
  if (controllerOnline) {
    joystickX = x;
    joystickY = y;
    addTrailPoint(x, y);
    Serial.println("[CONTROLLER] Found at I2C address 0x5A");
  } else {
    Serial.println("[CONTROLLER] No response at I2C address 0x5A");
  }

  drawScreen();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;
    pollController();
    drawScreen();
  }
}
