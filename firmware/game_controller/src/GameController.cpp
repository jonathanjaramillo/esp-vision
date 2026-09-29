#include "GameController.h"

const uint8_t GameController::BUTTON_REGS[GameController::BUTTON_COUNT] = {
    0x22, // A
    0x23, // B
    0x21, // C
    0x24, // D
    0x20  // OK
};

void GameController::begin(TwoWire &wire, uint8_t address) {
  _wire = &wire;
  _address = address;
}

bool GameController::readRegister(uint8_t reg, uint8_t &value) {
  if (_wire == nullptr) return false;

  _wire->beginTransmission(_address);
  if (_wire->write(reg) != 1) {
    _wire->endTransmission();
    return false;
  }
  // Preserve the controller's original transaction pattern: stop after the
  // register selector, then start a separate read request.
  if (_wire->endTransmission() != 0) {
    return false;
  }

  const uint8_t received = _wire->requestFrom(_address, (uint8_t)1, (uint8_t)true);
  if (received != 1 || !_wire->available()) return false;

  value = (uint8_t)_wire->read();
  return true;
}

bool GameController::readAxes(uint8_t &x, uint8_t &y) {
  uint8_t nextX;
  uint8_t nextY;
  if (!readRegister(AXIS_X_REG, nextX) || !readRegister(AXIS_Y_REG, nextY)) {
    return false;
  }
  x = nextX;
  y = nextY;
  return true;
}

bool GameController::readButtonEvent(Button button, uint8_t &event) {
  if ((uint8_t)button >= BUTTON_COUNT) return false;
  uint8_t nextEvent;
  if (!readRegister(BUTTON_REGS[(uint8_t)button], nextEvent)) return false;
  event = nextEvent;
  return true;
}
