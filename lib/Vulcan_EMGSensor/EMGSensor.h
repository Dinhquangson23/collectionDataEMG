#ifndef EMG_SENSOR_H
#define EMG_SENSOR_H

#include <SPI.h>
#include <Mcp320x.h>
// #include "MCP3x6x.h"
#include "SPIFFS.h"

#include "KickFiltersRT.h"
#include "Vulcan_Utils.h"
#include <Adafruit_NeoPixel.h>

// #define MCP3x6x_DEBUG 1

#define THRESHOLD_SETUP 			50
#define THRESHOLD_DC    			40
#define THRESHOLD_LOW   			10
#define THRESHOLD_HIGH  			20
#define THRESHOLD_MAX   			1000

#define TIME_READY 						3000

#define ADC_VREF    					3300    				// 3.3V Vref
#define CHANELS     					6      					// samples

#define SIZE_WRITE						CHANELS * 4

#define SIZE_ONE_PACKET_REV		sizeof(int16_t) * (CHANELS + 1) // array CHANELS + 1 float (CHANELS sensor + 1 time read)
#define NUMBER_PACKET_REV			10							//10 packet/send
#define LENGTH_DATA_REV				NUMBER_PACKET_REV * SIZE_ONE_PACKET_REV // <600 byte

#define FILE_DATA_EMG							"/data.txt"

#define FS 					1000			// 880
#define TB 					1000000/FS	// time delay FS (micro/s)

static portMUX_TYPE timerMuxRead = portMUX_INITIALIZER_UNLOCKED; 
static portMUX_TYPE timerMuxWrite = portMUX_INITIALIZER_UNLOCKED; 

class EMGSensor
{
	public:
		// EMGSensor(MCP3464 &_adc, Adafruit_NeoPixel &_pixels);
		EMGSensor(MCP3208 &_adc, Adafruit_NeoPixel &_pixels);
		void setupSPIFFS();
		void formatSPIFFS();
		void begin();
		int8_t sync();
		uint8_t readSensorStreamBLE();
		void readSensorSDBLE();
		// uint8_t readSensorSDBLE();
		uint8_t readSensorBLE();

		bool isReady();
		
		void setStateSensor(uint8_t _stateSensor);
		void setTimeReadSensor(uint8_t _timeRead);

		uint16_t getVref();
		float getSampleRate();
		void setSampleRate(uint16_t sampleRate);

		uint16_t* getThreshold();
		void setThreshold(uint16_t _threshol[]);

		int8_t getStateControl();
		void setModeLogicControl(uint8_t _mode);
		uint8_t getModeLogicControl();

		int readData();
		int readSD();
		void printDataSD();

		uint8_t buffer[SIZE_ONE_PACKET_REV];
		uint8_t rev[LENGTH_DATA_REV]; 			
		int16_t data[CHANELS+1] = {0};  //8 chanels + timeCycle
		int16_t dataFilter[CHANELS] = {0};

		Adafruit_NeoPixel &pixels;

		// bool readTemp();
		// void setTemp(bool _data);
		bool readTemp1();
		void setTemp1(bool _data);

		void checkTimeRead();
		void checkTimeWrite(void);
	private:
		bool temp = false;
		bool temp1 = true;

		hw_timer_t* timerRead = NULL; //khơi tạo timer
		hw_timer_t* timerWrite = NULL; //khơi tạo timer
		

		uint16_t SWSPL_FREQ;
		
		MCP3208 &adc;
		// MCP3464 &adc;

		KickFiltersRT<int16_t> ftNotch;
		KickFiltersRT<int16_t> ftHighSensor[8];
		KickFiltersRT<int16_t> ftLowSensor[8];
		KickFiltersRT<float> ftLowSensor1[8];
		const float fs = 1000;

		// State Ring
		bool stateReady = false;

		//Variable of logic Control
		uint8_t modeLogic = 1; 
		int8_t open = -1;
		int8_t close = 1;

		uint8_t stateControl = 0;

		// Variable of time read
		int8_t stateSensor = 0;

		uint32_t timeCycle;
		uint32_t timeSetRead = 0;
		uint32_t timeRead = micros();
		uint8_t chanels = 3; 							//number of channels

		//Dynamic memory pointer
		int16_t *dataPoint;

		//Variable to store sensor signal using flash memory
		File file;
		bool isFirtRead = true;
		bool isStartSend = true;

		uint32_t count = 0;
		uint32_t lengthData = 0;

		//Threshold for control processing
		uint16_t threshold[3]= {THRESHOLD_SETUP, THRESHOLD_LOW, THRESHOLD_HIGH};

		//Variable for sensor signal processing
		uint16_t sensor[CHANELS];
		int16_t value[CHANELS+1];
		int16_t valueDC[CHANELS];
		int16_t valueOut[CHANELS];
		int16_t filterKalman[CHANELS];
		int16_t output;
		int16_t outputDC[3];

		bool stateWait = false;

		bool isGetOnHand();

		//Function for sensor signal processing
		void firstReadSensor();
		void readSensor();
		void filterSensor();
		void setSensorDC();
		void smoothSensor();
		bool waitSensor(uint16_t threshold, uint16_t waitTime);
		void waitRelax();

		//Function for reading and writing sensor signals using dynamic memory
		void setupData();
		void writeData();
		void closeData();
		void openData();
		void clearData();
		
		//Function for reading and writing sensor signals using flash memory
		void setupSD();
		void writeSD();
		void openSD();
		void closeSD();

};

#endif