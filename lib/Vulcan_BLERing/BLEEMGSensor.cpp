#include "BLEEMGSensor.h"

void  BLEEMGSensor::getStringPref(Preferences& pref, 
  const char* key, char* data, size_t maxLen, const char* defaultVal) {
  
  size_t len = pref.getBytes(key, data, maxLen);
  if (len == 0) {
    len = strlen(defaultVal);
    memcpy(data, defaultVal, len);
  }
  data[len] = '\0';
}

BLEEMGSensor::BLEEMGSensor(BLEService* service, EMGSensor& pEMGSensor): 
    BLEServiceManager(service), emgSensor(pEMGSensor) {
  addReadWriteNotify(SIGNAL_UUID, 
    authed([this](BLECharacteristic* p) {onReadSignal(p);}), 
    authed([this](BLECharacteristic* p) {onWriteSignal(p);}));
  addReadWrite(THRESHOLD_UUID, 
    authed([this](BLECharacteristic* p) {onReadThreshold(p);}), 
    authed([this](BLECharacteristic* p) {onWriteThreshold(p);}));

  addReadWrite(LOGIC_UUID, 
    authed([this](BLECharacteristic* p) {onReadLogic(p);}), 
    authed([this](BLECharacteristic* p) {onWriteLogic(p);}));

  addReadWriteNotify(STATE_CONTROL_UUID, 
    authed([this](BLECharacteristic* p) {onReadStateControl(p);}), 
    doNothing);
  addReadWriteNotify(BATTERY_UUID, 
    authed([this](BLECharacteristic* p) {onReadBattery(p);}), 
    authed([this](BLECharacteristic* p) {onWriteBattery(p);}));
  // Increasing WDT timeout to 10s to fix esp_ota_begin issue
  // https://github.com/espressif/esp-idf/issues/1479
  // https://github.com/espressif/esp-idf/issues/578
  //esp_task_wdt_init(10, false);
}

void BLEEMGSensor::begin() {
  BLEServiceManager::begin();
  pref.begin("ble-info", false);
  if (pref.getBytes("threshold", threshold, sizeof(threshold))) {
    Serial.printf("Threshold: %d, %d, %d\n", threshold[0], threshold[1], threshold[2]);
    if (threshold[0] == 0) {
      Serial.println("Calibration is false.\nYou must be calibration again!!!");
    } else emgSensor.setThreshold(threshold);
  }
  emgSensor.setModeLogicControl(pref.getUShort("modeLogic", 1));
}

uint8_t BLEEMGSensor::getModeReadSensor() {
  return onRead;
}

void BLEEMGSensor::onReadSignal(BLECharacteristic* pChar) {
  // int16_t count = emgSensor.readData(); //
  int16_t count = emgSensor.readSD();
  switch (count)
  {
  case -2: 
    pChar->setValue(START_SEND_SENSOR);
    break;
  case -1: 
    pChar->setValue(END_SEND_SENSOR);
    break;
  
  default:
    pChar->setValue(emgSensor.rev, LENGTH_DATA_REV);
    Serial.println(count);
    break;
  }

}

// 0: Stop read sensor
// TimeRead
void BLEEMGSensor::onWriteSignal(BLECharacteristic* pChar) {
  const char* s = pChar->getValue().c_str();
  Serial.printf("Value: %s\t length: %d\n", s, strlen(s));
  if (strlen(s) > 2) {
    char p[3];
    strncpy(p, s, 3);
    uint8_t timeRead = atoi(p);
    onRead = timeRead == 0 ? 0:1;
    Serial.printf("timeRead: %d\t onRead: %d\n", timeRead, onRead);
    emgSensor.setTimeReadSensor(timeRead);
  }
}

void BLEEMGSensor::onNotifySignal(uint8_t _state) {
	// 1: firtRead
	// 2: onRead
	// 3: endRead
  BLECharacteristic* pCharacteristic = service->getCharacteristic(SIGNAL_UUID);
  switch (_state)
  {
  case 1:
    pCharacteristic->setValue(START_READ_SENSOR);
    pCharacteristic->notify();
    break;
  case 3:
    pCharacteristic->setValue(END_READ_SENSOR);
    pCharacteristic->notify();
    onRead = 0;
    break;
  case 2:
    pCharacteristic->setValue(emgSensor.buffer, SIZE_ONE_PACKET_REV);
    pCharacteristic->notify();
    break;
  default:
    break;
  }
}

void BLEEMGSensor::onReadThreshold(BLECharacteristic* pChar) {
  uint8_t _size = sizeof(threshold);
  uint8_t buff[6];
  memcpy((void*)buff, (void*)threshold, sizeof(threshold));
  pChar->setValue(buff, 6);
  Serial.printf("Threshold: %d, %d, %d\n", threshold[0], threshold[1], threshold[2]);
}

void BLEEMGSensor::onWriteThreshold(BLECharacteristic* pChar) {
    const char* s = pChar->getValue().c_str();
  if (strlen(s) > 8) {
		for (uint8_t i = 0; i < 3; i++) {
			char p[4];
			strncpy(p, s + i*4, 4);
      threshold[i] = atoi(p);
		}
    Serial.printf("Threshold: %d, %d, %d\n", threshold[0], threshold[1], threshold[2]);
    if (threshold[0] == 0) {
      Serial.println("Calibration is false.\nYou must be calibration again!!!");
    } else {
      emgSensor.setThreshold(threshold);
      pref.putBytes("threshold", threshold, sizeof(threshold));
    } 
  }
}

void BLEEMGSensor::onReadLogic(BLECharacteristic* pChar) {
  uint8_t _modeLogic = emgSensor.getModeLogicControl();
  pChar->setValue(&_modeLogic, 1);
}

void BLEEMGSensor::onWriteLogic(BLECharacteristic* pChar) {
  const char* s = pChar->getValue().c_str();
  if (strlen(s) > 0) {
    uint8_t modeLogic = atoi(s);
    pref.putUShort("modeLogic", modeLogic);
    emgSensor.setModeLogicControl(modeLogic);
  }
}

//Read State Control
// 1: relax
// 2: hold
// 3: flex
void BLEEMGSensor::onReadStateControl(BLECharacteristic* pChar) {
  uint8_t _stateControll = emgSensor.getStateControl() + 2;
  pChar->setValue(&_stateControll, 1);
}

void BLEEMGSensor::onNotifyStateControl(int8_t _stateControl) {
  uint8_t stateControl = _stateControl + 2;
  BLECharacteristic* pCharacteristic = service->getCharacteristic(STATE_CONTROL_UUID);
  pCharacteristic->setValue(&stateControl, 1);
  pCharacteristic->notify();
}

void BLEEMGSensor::onReadBattery(BLECharacteristic* pChar) {
  pChar->setValue(&batteryPercent, 1);
}

void BLEEMGSensor::onWriteBattery(BLECharacteristic* pChar) {

}

void BLEEMGSensor::onNotifyBattery(uint8_t _battery) {
  BLECharacteristic* pCharacteristic = service->getCharacteristic(BATTERY_UUID);
  if (batteryPercent == _battery) return;
  batteryPercent = _battery;
  pCharacteristic->setValue(&batteryPercent, 1);
  pCharacteristic->notify();
}
