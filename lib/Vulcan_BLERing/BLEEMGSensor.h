#ifndef BLE_EMG_SENSOR_H
#define BLE_EMG_SENSOR_H

#include "BLEServiceManager.h"
#include <Preferences.h>
#include "EMGsensor.h"

#define SIGNAL_UUID										"ebbb06e2-e254-4989-9555-a7fc9ca8f5c4"
#define THRESHOLD_UUID								"39b2df5b-b7d4-48c6-afd2-e0095d4a999c"
#define LOGIC_UUID										"363b46ab-0bc8-4b76-ad6f-2320302d1da5"
#define STATE_CONTROL_UUID						"22e5bbd9-62d8-45eb-9ffd-4a2b88cd6c3a"
#define BATTERY_UUID									"d23b3d36-e178-4528-af4d-7e8f9139aa20"

#define END_READ_SENSOR								"END_READ_SENSOR"
#define START_READ_SENSOR							"START_READ_SENSOR"

#define END_READ_FILTER_SENSOR				"END_READ_FILTER_SENSOR"
#define START_READ_FILTER_SENSOR			"START_READ_FILTER_SENSOR"

#define END_SEND_SENSOR								"END_SEND_SENSOR"
#define START_SEND_SENSOR							"START_SEND_SENSOR"

class BLEEMGSensor:public BLEServiceManager<5>
{
  public:
		BLEEMGSensor(BLEService* service, EMGSensor& pEMGSensor);
		virtual void begin();
		void onNotifySignal(uint8_t _state);
		void onNotifyStateControl(int8_t _stateControl);
		void onNotifyBattery(uint8_t _battery);
		
		uint8_t getModeReadSensor();

  private:
		Preferences pref;
		EMGSensor& emgSensor;
		uint8_t onRead = 0;
		uint8_t batteryPercent = 0;

		uint16_t threshold[3] = {0, 0, 0};

		void onReadSignal(BLECharacteristic* pChar);
		void onWriteSignal(BLECharacteristic* pChar);

		void onReadThreshold(BLECharacteristic* pChar);
		void onWriteThreshold(BLECharacteristic* pChar);

		void onReadLogic(BLECharacteristic* pChar);
		void onWriteLogic(BLECharacteristic* pChar);		

		void onReadStateControl(BLECharacteristic* pChar);

		void onReadBattery(BLECharacteristic* pChar);
		void onWriteBattery(BLECharacteristic* pChar);


		void getStringPref(Preferences& p, const char* key, char* data, size_t maxLen, const char* defaultVal);
};

#endif