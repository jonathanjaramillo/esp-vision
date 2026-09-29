#ifndef GAME_CONTROLLER_H
#define GAME_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>

class GameController {
public:
  static const uint8_t DEFAULT_I2C_ADDRESS = 0x5A;

  enum Button : uint8_t {
    BUTTON_A = 0,
    BUTTON_B,
    BUTTON_C,
    BUTTON_D,
    BUTTON_OK,
    BUTTON_COUNT
  };

  enum ButtonEvent : uint8_t {
    PRESS_DOWN = 0,
    PRESS_UP = 1,
    PRESS_REPEAT = 2,
    SINGLE_CLICK = 3,
    DOUBLE_CLICK = 4,
    LONG_PRESS_START = 5,
    LONG_PRESS_HOLD = 6,
    NONE_PRESS = 8
  };

  // Uses a bus that the application has already initialized (K10.begin()).
  void begin(TwoWire &wire, uint8_t address = DEFAULT_I2C_ADDRESS);

  bool readAxes(uint8_t &x, uint8_t &y);
  bool readButtonEvent(Button button, uint8_t &event);

private:
  static const uint8_t AXIS_X_REG = 0x10;
  static const uint8_t AXIS_Y_REG = 0x11;
  static const uint8_t BUTTON_REGS[BUTTON_COUNT];

  TwoWire *_wire = nullptr;
  uint8_t _address = DEFAULT_I2C_ADDRESS;

  bool readRegister(uint8_t reg, uint8_t &value);
};

#endif
