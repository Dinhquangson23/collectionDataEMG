#ifndef RING_STATE_H
#define RING_STATE_H

#include <Arduino.h>
// #include "Vulcan_RF24L01.h"
#include "EMGSensor.h"

class RingState
{
  public:
		// RingState(SPIClass &_spiNRF, RF &_rf, byte _txAddr[], byte _rxAddr[]);
		RingState();
		void begin();
		void onConnect();
		void onDisconnect();

		int8_t sync(int8_t pstateControl);

		void setMode(uint8_t _mode);
		uint8_t getMode();

		
	private:
		const char data[3] = { 'a', 'b', 'c' };  //data control OPEN, CLOSE, STOP

		bool onConnectBle = false;
		uint8_t mode = 0;
};

#endif