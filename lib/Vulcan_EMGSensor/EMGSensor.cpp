#include "EMGSensor.h"

EMGSensor* pointerToClass;

static void outsideTimerWrite(void) {
	pointerToClass->checkTimeWrite();
}


EMGSensor::EMGSensor(MCP3208 &_adc, Adafruit_NeoPixel &_pixels):
	adc(_adc),
	pixels(_pixels){
}

void EMGSensor::setupSPIFFS() {
	SPIFFS.begin(true);
}

void EMGSensor::begin() {

	pointerToClass = this;
	timerWrite = timerBegin(0, 80, true);
	timerAttachInterrupt(timerWrite, outsideTimerWrite, true);
	timerAlarmWrite(timerWrite, 1000000, true);

	for (uint8_t i = 0; i < 8; i++) {
		ftHighSensor[i].inithighpass(0, 5, fs);
		ftLowSensor[i].initlowpass(0, 0.1, fs);
	}
	// SPIFFS.begin(true);
	// Serial.printf("TB: %d\n", TB);
	// Serial.println("Setup Myo Band");
	waitRelax();
	
}

bool EMGSensor::isGetOnHand() {
	readSensor();
	filterSensor();
	if (value[0] < THRESHOLD_MAX && value[1] < THRESHOLD_MAX && value[2] < THRESHOLD_MAX && value[3] < THRESHOLD_MAX && value[4] < THRESHOLD_MAX && value[5] < THRESHOLD_MAX)
	return true;
	else return false;
}

void EMGSensor::firstReadSensor() {
	readSensor();
	filterSensor();
	setSensorDC();

	// for(uint8_t i = 0; i < CHANELS; i++) {
	// 	kalmanFt[i].setFirstValue(abs(ftHighSensor[i].highpass(data[i])));
	// }
}

void EMGSensor::readSensor() {
	static uint32_t lastTimeCycle = micros();
	sensor[0] = adc.read(MCP3208::Channel::SINGLE_0);
	sensor[1] = adc.read(MCP3208::Channel::SINGLE_1);
	sensor[2] = adc.read(MCP3208::Channel::SINGLE_2);

	sensor[3] = adc.read(MCP3208::Channel::SINGLE_3);
	sensor[4] = adc.read(MCP3208::Channel::SINGLE_4);
	sensor[5] = adc.read(MCP3208::Channel::SINGLE_5);

	// sensor[0] = adc.analogRead(MCP_CH0);
	// sensor[1] = adc.analogRead(MCP_CH1);
	// sensor[2] = adc.analogRead(MCP_CH2);
	
	// Serial.printf("%d\t%d\t%d\n", sensor[0], sensor[1], sensor[2]);
	// value[6] = micros() - lastTimeCycle;							//timeCycle
	lastTimeCycle = micros();
}

void EMGSensor::filterSensor() {

	//value[4]
	for (uint8_t i = 0; i < CHANELS; i++) {
		// UTILS_LOW_PASS_FILTER(value[i], abs(ftHighSensor[i].highpass(sensor[i])), 0.05);
		value[i] = ftHighSensor[i].highpass(sensor[i]);
	}
	// Serial.printf("%.2f\t%.2f\t%.2f\n", value[0], value[1], value[2]);
	// Serial.printf("%d\t%d\t%d\n", value[0], value[1], value[2]);
	// Serial.printf("%d\t%d\t%d\t%d\n", value[0], value[1], value[2], value[3]);
	// Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", value[0], value[1], value[2], value[3], value[4], value[5], value[6]);
}

void EMGSensor::setSensorDC() {
	for(uint8_t i = 0; i < CHANELS; i++) {
		valueDC[i] = ftLowSensor[i].lowpass(value[i]);
	}
	// Serial.printf("%.2f\t%.2f\t%.2f\n", value[0], value[1], value[2]);
}

void EMGSensor::smoothSensor() {
	for(uint8_t i = 0; i < CHANELS; i++) {
		valueOut[i] = value[i] - valueDC[i];
		// valueOut[i] = ftLowSensor1[i].lowpass(value[i]);
		UTILS_LOW_PASS_FILTER(data[i], valueOut[i], 0.05);
	}
	// Serial.printf("%.2f\t%.2f\t%.2f\n", data[0], data[1], data[2]);
}

bool EMGSensor::waitSensor(uint16_t threshold, uint16_t waitTime) {
	static uint32_t _timeReady = millis();
	readSensor();
	filterSensor();
	// setSensorDC();
  	// Serial.printf("%d\t%d\t%d\t", value[0], value[1], value[2]);
  	// Serial.printf("%.2f\t%.2f\t%.2f\t", valueDC[0], valueDC[1], valueDC[2]);
  	// Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\n", value[0], value[1], value[2], value[3], value[4], value[5]);
	vTaskDelay(100);
  if ((value[0] < threshold) && (value[1] < threshold) && (value[2] < threshold) && (value[3] < threshold) && (value[4] < threshold) && (value[5] < threshold)) {
    if (!stateWait) {
      _timeReady = millis();
      stateWait = true;
    }
    if (millis() - _timeReady > waitTime) return true;
    else return false;
  } else {
    stateWait = false;
    return false;
  }
}

void EMGSensor::waitRelax() {
	stateReady = false;
	pixels.setPixelColor(0, pixels.Color(0, 0, 250));		//Setup Ring LED BLUE
	pixels.show();
	memset(sensor, 0, sizeof(sensor));
  memset(value, 0, sizeof(value));
  memset(valueDC, 0, sizeof(valueDC));
  memset(valueOut, 0, sizeof(valueOut));
  memset(filterKalman, 0, sizeof(filterKalman));

	for (int i = 0; i < 5000; i++) {
    firstReadSensor();
  }
	Serial.println("Myoband wait get on Hand");
	while(!isGetOnHand());

	Serial.println("Myoband on Setup");

	stateWait = false;
	while (!waitSensor(threshold[0], TIME_READY));
 	// Serial.println("myoban ready");
 	// Serial.println("myoban wait relax");
  delay(200);
	// stateWait = false;
  // while (!waitSensor(threshold[0], TIME_READY));
	Serial.println("MyoBand running");
  delay(200);
	pixels.setPixelColor(0, pixels.Color(0, 255, 0));		//Ready Ring LED GREEN
	pixels.show();
	stateReady = true;
}

int8_t EMGSensor::sync() {
	if (!stateReady) return stateControl = 0;
	readSensor();
	filterSensor();
	smoothSensor();
	float output = data[0] + data[1] + data[2];
	// Serial.printf("%.2f\t%.2f\t%.2f\t", data[0], data[1], data[2]);
	// Serial.print(output);
	// Serial.printf("\n");
  if (value[0] > THRESHOLD_MAX || value[1] > THRESHOLD_MAX || value[2] > THRESHOLD_MAX) {
    waitRelax();
    return stateControl = 0;
  } else {
    if (output > threshold[2]) {
      return stateControl = close;
    } else {
      if (output < threshold[1]) {
        return stateControl = open;
      } else return stateControl = 0;
    }
  }
}

uint8_t EMGSensor::readSensorStreamBLE() {
	// stateSensor:
	// 0: don't read
	// !0: onRead

	// _state return:
	// 0: don't read
	// 2: onRead
	if (stateSensor == 0) {
		return 0;
	}
	readSensor();
	filterSensor();
	smoothSensor();
	memcpy((void *)buffer, (void *)data, SIZE_ONE_PACKET_REV);
	delay(5);
	return 2;
}

bool EMGSensor::readTemp1() {
	return temp1;
}

void EMGSensor::checkTimeWrite() {
	portENTER_CRITICAL_ISR(&timerMuxWrite);
	ets_printf("Write Sensor\n");
	readSensor();
	ets_printf("1\n");
	filterSensor();
	ets_printf("2\n");
	writeSD();
	ets_printf("3\n");
	count++;
	
	portEXIT_CRITICAL_ISR(&timerMuxWrite);
	
}

void EMGSensor::readSensorSDBLE() {
	// Serial.println("Wait time Read");
	if (timeSetRead != 0) {
		setupData();
		// setupSD();
		timeRead = micros();
		// Serial.printf("timeRead: %d\n", timeRead);
		// Serial.printf("timeSetRead: %d\n", timeSetRead);
		// Serial.println("Start Timer");
		// timerAlarmEnable(timerWrite);

		while (micros() - timeRead <= timeSetRead) {
			static uint32_t lastTime = 0;
			uint32_t nowTime = micros();
			if (nowTime - lastTime >= TB) {
				readSensor();
				filterSensor();
				value[6] = nowTime - lastTime;
				// writeSD();
				writeData();
				count++;
				lastTime = nowTime;
				// Serial.println(value[6]);
			}
			// }
		}
		// timerEnd(timerWrite);
		// timerWrite = NULL;
		// Serial.println("End Timer");
		
		closeData();
		// closeSD();
		Serial.println("End read sensor");
		Serial.printf("Total number of samples: %d\n", count);
		Serial.printf("Sampling frequency: %d\n",count/(timeSetRead/1000000));
		uint32_t timeN = millis();
		while (millis() - timeN < 10000) {
			pixels.setPixelColor(0, pixels.Color(250, 0, 250));		
			pixels.show();
			delay(300);
			pixels.clear();		
			pixels.show();
			delay(300);
			timeN = millis();
		}
		timeSetRead = 0;
	}
}

uint8_t EMGSensor::readSensorBLE() {
	if (stateSensor == 0) {
		return 0;
	}
	switch (stateSensor) {
		case 1:
			// Serial.println("Read Sensor SD");
			// return(readSensorSDBLE());
			break;
		case 2:
			return(readSensorStreamBLE());
			break;
		
		default:
			break;
	}
	return 0;
}

void EMGSensor::setTimeReadSensor(uint8_t _timeRead) {
	if (_timeRead == 0) {
		setStateSensor(0);
		Serial.println("End Read Sensor");
		return;
	}
	if (_timeRead == UINT8_MAX) {
		setStateSensor(2);
		Serial.println("Read sensor stream BLE");
	} else {
		timeSetRead = _timeRead * 1000 * 1000;
		Serial.printf("Set time read: %d s\n", _timeRead);
		setStateSensor(1);
	}
}

void EMGSensor::setStateSensor(uint8_t _stateSensor) {
	isStartSend = true;
	stateSensor = _stateSensor;
	isFirtRead = true;
}

void EMGSensor::setSampleRate(uint16_t sampleRate) {
	SWSPL_FREQ = sampleRate;
}

uint16_t EMGSensor::getVref() {
	return ADC_VREF;
}

float EMGSensor::getSampleRate() {
	return 1000000.0/timeCycle;
}

uint16_t* EMGSensor::getThreshold() {
  return threshold;
}

void EMGSensor::setThreshold(uint16_t _threshol[]) {
  memcpy((void*)threshold, (void*)_threshol, sizeof(threshold));
}

int8_t EMGSensor::getStateControl() {
	return stateControl;
}

void EMGSensor::setModeLogicControl(uint8_t _mode) {
	Serial.printf("Mode control: %d\n",_mode);
	if (!_mode) return;
	modeLogic = _mode;
	switch (_mode)
	{
		case 1:
			open = -1;
			close = 1;
			break;
		case 2: 
			open = 1;
			close = -1;
			break;
		default:
			break;
	}
}

uint8_t EMGSensor::getModeLogicControl() {
	return modeLogic;
}

bool EMGSensor::isReady() {
	return stateReady;
}

// Save data sensor by pointer
void EMGSensor::setupData() {
	clearData();
	dataPoint = (int16_t *)malloc(sizeof(int16_t) * 100000*(CHANELS + 1));
}

void EMGSensor::writeData() {
	memcpy(&dataPoint[count * (CHANELS + 1)], (void*)value, sizeof(value));
}

void EMGSensor::closeData() {
	lengthData = --count;
	setupSD();
	file.write((uint8_t*)dataPoint, lengthData * SIZE_ONE_PACKET_REV);
	file.close();
	Serial.println("Save done");
	esp_cpu_reset(1);
	// for (uint16_t i = 0; i < lengthData; i++) {
	// 	for (uint8_t j = 0; j < (CHANELS + 1); j++) {
	// 		Serial.printf("%d \t", dataPoint[i * (CHANELS + 1) + j]);
	// 	}
	// 	Serial.println();
	// }
}

void EMGSensor::openData() {
	count = 0;
}

int EMGSensor::readData() {
	if (isStartSend) {
		openData();
		isStartSend = false;
		return -2;
	}
	if (count < lengthData) {
		uint16_t buff = lengthData - count;
		buff = (buff < NUMBER_PACKET_REV) ? buff:NUMBER_PACKET_REV;
		memcpy((void*)rev, &dataPoint[count], buff * SIZE_ONE_PACKET_REV);
		count += buff;
		return buff;
	}
	else {
		clearData();
		temp1 = false;
		isStartSend = true;
		return -1;
	}
}

void EMGSensor::clearData() {
	free(dataPoint);
}

// Save data sensor by SD
void EMGSensor::setupSD() {
	file.close();
	SPIFFS.remove(FILE_DATA_EMG);
	file = SPIFFS.open(FILE_DATA_EMG, FILE_APPEND);
}

void EMGSensor::writeSD() {
	file.write((uint8_t*)value, SIZE_ONE_PACKET_REV);
}

void EMGSensor::closeSD() {
	// Serial.printf("Size data: %d byte\n", file.size());
	Serial.printf("Total number of samples: %d\n", count);
	file.close();
	
}

void EMGSensor::openSD() {
	file.close();
	file = SPIFFS.open(FILE_DATA_EMG);
}

int EMGSensor::readSD() {
	if (isStartSend) {
		openSD();
		isStartSend = false;
		count = 0;
		return -2;
	}
	if (file.available()) {
    file.read(rev, LENGTH_DATA_REV);
		for (uint8_t i = 0; i < NUMBER_PACKET_REV; i++) {
			int16_t buff[CHANELS + 1];
			memcpy((void *)buff, (void *)(rev + SIZE_ONE_PACKET_REV * i), SIZE_ONE_PACKET_REV);
			for (uint8_t j = 0; j < CHANELS + 1; j++) {
				// Serial.printf("%.2f\t", buff[j]);
				Serial.printf("%d\t", buff[j]);
				// Serial.printf("%d\t", (rev + SIZE_ONE_PACKET_REV * i)[j]);
			}
			Serial.print("\n");
		}
		count++;
		return count;
	}
	else {
		closeSD();
		temp1 = false;
		isStartSend = true;
		return -1;
	}
}
