#include "BLERingManager.h"

BLERingManager::BLERingManager(RingState& pRingState, EMGSensor& pEMGSensor)
  :ringState(pRingState)
  ,emgSensor(pEMGSensor){
}

void BLERingManager::onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
  stateConnect = true;
  Serial.println("Connected");

  // Update connection configuration with peer
  // latency = 0; min_int = 0x40; max_int = 0x64; timeout = 4s;
  pServer->updateConnParams(param->connect.remote_bda, 0x40, 0x64, 0, 400);

  // The longer distance, the more time it takes to smooth,
  // which might break the authentication from app
  distance = 40; // meter
  ringState.onConnect(); 
}

void BLERingManager::onDisconnect(BLEServer* pServer) {
  stateConnect = false;
  Serial.println("Disconnected");
  ringState.onDisconnect();
  ringService->resetOTA();
}

void BLERingManager::setPower() {
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
}

void BLERingManager::setupAdvertising() {
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SENSOR_SERVICE_UUID);
  advertising->addServiceUUID(RING_SERVICE_UUID);
  
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);  // for iPhone connections issue
  advertising->setMaxPreferred(0x12);

  BLEAdvertisementData data;
  data.setName(ringService->getName());
  //data.setManufacturerData(handService->getFrameNumber());
  advertising->setScanResponseData(data);
  BLEDevice::startAdvertising();
}

void BLERingManager::startAdvertising() {
  BLEDevice::startAdvertising();
}

void BLERingManager::startServer() {
  // Init without device name, which will be set later 
  // accordingly to handService.getName()
  BLEDevice::init("");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(this);

  ringService = new BLERing(server->createService(RING_SERVICE_UUID), ringState);
  emgSensorService = new BLEEMGSensor(server->createService(SENSOR_SERVICE_UUID), emgSensor);

  ringService->begin();
  emgSensorService->begin();

  // Restore original name after firmware upgrade
  esp_ble_gap_set_device_name(ringService->getName());

  Serial.println("BLE started");
}

void BLERingManager::begin() {
  startServer();
  setPower();
  setupAdvertising();
}

bool BLERingManager::waitConnect() {
  uint32_t lastTime = millis();
  while ((millis() - lastTime < TIME_CONNECT) && (!stateConnect)) {
    emgSensor.pixels.setPixelColor(0, emgSensor.pixels.Color(255, 255, 255));
    emgSensor.pixels.show();
    delay(300);
    emgSensor.pixels.clear();
    emgSensor.pixels.show();
    delay(1000);
  }

  if (stateConnect) {
    emgSensor.pixels.setPixelColor(0, emgSensor.pixels.Color(0, 255, 0));
    emgSensor.pixels.show();
    delay(200);
    emgSensor.pixels.clear();
    emgSensor.pixels.show();
    delay(500);
    emgSensor.pixels.setPixelColor(0, emgSensor.pixels.Color(0, 255, 0));
    emgSensor.pixels.show();
    delay(200);
  }
  return stateConnect;
}

bool BLERingManager::isOnOTA() {
  return ringService->isOnOTA();
}

void BLERingManager::notifyData(uint8_t _state) {
  // if ((_state != 1) && (_state != 3)) return;
  emgSensorService->onNotifySignal(_state);
}

void BLERingManager::sendControl(int8_t stateControl) {
  emgSensorService->onNotifyStateControl(stateControl);
}

void BLERingManager::notifyBatery(uint8_t battery) {
  emgSensorService->onNotifyBattery(battery);
}

uint8_t BLERingManager::getModeReadSensor() {
  return emgSensorService->getModeReadSensor();
}

uint8_t BLERingManager::getModeRing() {
  return ringState.getMode();
}

BLERingManager::~BLERingManager() {
  free(ringService);
  free(emgSensorService);
}