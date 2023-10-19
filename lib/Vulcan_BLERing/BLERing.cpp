#include "BLERing.h"
#include "freertos/FreeRTOS.h"

void  BLERing::getStringPref(Preferences& pref, 
  const char* key, char* data, size_t maxLen, const char* defaultVal) {
  
  size_t len = pref.getBytes(key, data, maxLen);
  if (len == 0) {
    len = strlen(defaultVal);
    memcpy(data, defaultVal, len);
  }
  data[len] = '\0';
}

static esp_ble_adv_data_t ADVERTISING_CONFIG = {
  .set_scan_rsp = false,
  .include_name = true,
  .include_txpower = true,
  .min_interval = 0x0006,
  .max_interval = 0x0010,
  .appearance = 0x00,
  .manufacturer_len = 0,
  .p_manufacturer_data = nullptr,
  .service_data_len = 0,
  .p_service_data = nullptr,
  .service_uuid_len = 0,
  .p_service_uuid = nullptr,
  .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

BLERing::BLERing(BLEService* service, RingState& pRingState): 
    BLEServiceManager(service), ringState(pRingState) {
  // addReadWriteNotify(OTA_UUID,
  //   authed([this](BLECharacteristic* p) {onReadOTA(p);}),
  //   authed([this](BLECharacteristic* p) {onWriteOTA(p);}));

  addReadWrite(NAME_CHAR_UUID, 
    authed([this](BLECharacteristic* p) {onReadName(p);}), 
    authed([this](BLECharacteristic* p) {onWriteName(p);}));

  // addReadWrite(HARDWARE_CHAR_UUID, 
  //   authed([this](BLECharacteristic* p) {onReadHardware(p);}), 
  //   authed([this](BLECharacteristic* p) {onWriteHardware(p);}));
  
  // Increasing WDT timeout to 10s to fix esp_ota_begin issue
  // https://github.com/espressif/esp-idf/issues/1479
  // https://github.com/espressif/esp-idf/issues/578
  //esp_task_wdt_init(10, false);
}

void BLERing::begin() {
  BLEServiceManager::begin();
  // SPIFFS.begin(true);

  updateInProgress = false;
  
  pref.begin("ble-info", false);
  getStringPref(pref, "name", name, MAX_NAME_LENGTH, INIT_NAME);
  Serial.printf("BLE MyoBand: %s\n",name);

  getStringPref(pref, "hardware", hardware, MAX_HARDWARE_LENGTH, INIT_HARDWARE);
  Serial.printf("Hardware Hand: %s\n",hardware);
}

void BLERing::onReadOTA(BLECharacteristic* pChar) {
  pChar->setValue(VERSION);
}

void BLERing::onWriteOTA(BLECharacteristic* pChar) {
  // OTA save on esp_ota
  std::string rxData = pChar->getValue();
  if (rxData == UPDATE_START_MSG) {
    updateInProgress = true;
    Serial.println("Begin OTA");
    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    Serial.println("Found partition");
    esp_err_t result = esp_ota_begin(partition, OTA_SIZE_UNKNOWN, &otaHandler);
    if (result == ESP_OK) {
      Serial.println("OTA operation commenced successfully");
    } else {
      Serial.print("Failed to commence OTA operation, error: ");
      Serial.println(result);
    }
    packageCounter = 0;
    Serial.println("Begin OTA done");
  }
  else if (rxData == UPDATE_END_MSG) {
    Serial.println("OTA: Upload completed");
    esp_err_t result = esp_ota_end(otaHandler);
    if (result == ESP_OK) {
      Serial.println("Newly written OTA app image is valid.");
    } else {
      Serial.print("Failed to validate OTA app image, error: ");
      Serial.println(result);
    }
    if (esp_ota_set_boot_partition(esp_ota_get_next_update_partition(NULL)) == ESP_OK) {
      delay(1000);
      esp_restart();
    } else {
      Serial.println("OTA Error: Invalid boot partition");
    }
  }
  else {
    Serial.print(rxData.length()); 
    Serial.print("\t");

    if (esp_ota_write(otaHandler, rxData.c_str(), rxData.length()) == ESP_OK) {
      packageCounter++;
    }
    else {
      Serial.println("OTA is Fail, please try again!!!");
      packageCounter = UINT16_MAX;
    }
    Serial.println(packageCounter);
    onNotifyOTA(packageCounter);
    // Make the writes much more reliable
     
    // vTaskDelay(1);
  }
  
}

void BLERing::onNotifyOTA(uint16_t stateOTA) {
  BLECharacteristic* pCharacteristic = service->getCharacteristic(OTA_UUID);
  pCharacteristic->setValue(stateOTA);
  pCharacteristic->notify();
}

const char* BLERing::getName() {
  return name;
}

void BLERing::onWriteName(BLECharacteristic* pChar) {
  const char* s = pChar->getValue().c_str();
  if (strlen(s) > 0) {
    memcpy(name, s, strlen(s) + 1);
    pref.putBytes("name", s, strlen(s));
    
    // Update device name, name is change after esp reset 
    esp_ble_gap_set_device_name(s);
    // Advertise the changes to clients
    esp_ble_gap_config_adv_data(&ADVERTISING_CONFIG);
    Serial.printf("Mame: %s\n", name);
  }
}

void BLERing::onReadName(BLECharacteristic* pChar) {
  pChar->setValue(name);
}

void BLERing::onReadHardware(BLECharacteristic* pChar) {
	pChar->setValue(hardware);
}

void BLERing::onWriteHardware(BLECharacteristic* pChar) {
  const char* s = pChar->getValue().c_str();
  if (strlen(s) > LOCK_LEN) {
    char key[LOCK_LEN + 1];
    strncpy(key, s, LOCK_LEN);
    key[LOCK_LEN] = '\0';

    if (strcmp(key, LOCK) == 0) {
      Serial.println("key is valid");
      memcpy(hardware, s + LOCK_LEN, strlen(s) - LOCK_LEN);
      Serial.println(hardware);
      pref.putBytes("hardware", hardware, strlen(s) - LOCK_LEN);
    }
    Serial.println("key is unvalid");
  }
}

bool BLERing::isOnOTA() {
  return updateInProgress;
}

void BLERing::resetOTA() {
  updateInProgress = false;
}
