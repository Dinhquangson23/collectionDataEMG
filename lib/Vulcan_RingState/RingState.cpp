#include "RingState.h"

RingState::RingState() {

}

void RingState::begin() {

}

int8_t RingState::sync(int8_t _stateControl) {
  static int8_t lastState = 0;
  static bool isFirstSaveStateControl = true;
  static uint32_t lastTime = millis();
  isFirstSaveStateControl = _stateControl == lastState ? false:true;
  lastState = _stateControl;
  if (isFirstSaveStateControl) {
    lastTime = millis();
  }
  if (millis() - lastTime > 3000) {
    _stateControl = 0;
  }

  return _stateControl;
}

void RingState::onConnect() {
  onConnectBle = true;
}

void RingState::onDisconnect() {
  onConnectBle = false;
}

uint8_t RingState::getMode() {
  return mode;
}

void RingState::setMode(uint8_t _mode) {
  mode = _mode;
}
