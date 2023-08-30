#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "esp_adc_cal.h"
#include "BLERingManager.h"
#include "EMGSensor.h"
#include "RingState.h"

#define DEBUG         

#define SPI_CS        15          // SPI slave select
#define ADC_CLK       8000000     // SPI clock 8MHz

#define LDC_CE        8

#define POWER_PIN     35

#define CHARGER_PIN   34
#define BAT_PIN       36
#define PIN_BITMASK   0x400000000 // 2^33 in hex

#define PIN_RGB       4
#define LED_EN        32

#define MIC_EN        7
#define VIBRATOR_PIN  27

#define TIME_WAKEUP   3000
#define BATTERY_MAX   4150
#define BATTERY_MIN   3200

// SPIClass spiMCP3464(HSPI);
SPIClass spiMCP3208(HSPI);
// MCP3464 adc(SPI_CS, &spiMCP3464);
MCP3208 adc(ADC_VREF, SPI_CS, &spiMCP3208);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

EMGSensor emgSensor(adc, pixels);
RingState ringState;
BLERingManager ble(ringState, emgSensor);

uint8_t stateSensor = 0;
uint8_t mode = 0;
int8_t stateControl = 0;
bool stateSleep = true;
bool stateRead = false;

void deepSleep();
void onSleep(bool _stateSleep);
void onWakeUp();
void onPower();
void onCharger();

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200); 

  gpio_deep_sleep_hold_dis();

  pinMode(LED_EN, OUTPUT);
  digitalWrite(LED_EN, HIGH);
	pinMode(LDC_CE, OUTPUT);
	digitalWrite(LDC_CE, HIGH);

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(POWER_PIN, INPUT);
  pinMode(CHARGER_PIN, INPUT_PULLDOWN);

  pixels.begin();
  pixels.setBrightness(20);
	pixels.clear();
	pixels.show();

  Serial.printf("State sleep: %d\n",stateSleep);

  onWakeUp();

  emgSensor.setupSPIFFS();

  static uint32_t timeReadSD = millis();

  while (millis() - timeReadSD < 5000) {
    // Serial.printf("timeReadSD: %d\n", timeReadSD);
    pixels.setPixelColor(0, pixels.Color(0, 220, 220));
    pixels.show();
    delay(300);
    pixels.clear();
    pixels.show();
    if (!digitalRead(POWER_PIN)) {
      pixels.setPixelColor(0, pixels.Color(230, 200, 0));
      pixels.show();
      while (emgSensor.readTemp1()) {
        onWakeUp();
        int16_t count = emgSensor.readSD();
        delay(1);
      }
    }
    delay(100);
  }
  Serial.println("Out");

  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  spiMCP3208.begin();
  spiMCP3208.beginTransaction(settings);

  ringState.begin();
  ble.begin();
  emgSensor.begin();
  Serial.println();

  stateSleep = !ble.waitConnect();
  onSleep(stateSleep);

  Serial.println("Start");
}

void loop() {
  emgSensor.readSensorSDBLE();
}

void onSleep(bool _stateSleep) {
  if (!_stateSleep) return;
  Serial.println("On deep sleep");

  digitalWrite(LED_EN, HIGH);
  pixels.setPixelColor(0, pixels.Color(255, 0 , 0));
  pixels.show();

  vTaskDelay(200);

  pixels.clear();
  pixels.show();

  digitalWrite(LED_EN, LOW);
  digitalWrite(LDC_CE, LOW);
  digitalWrite(VIBRATOR_PIN, LOW);

  gpio_hold_en(GPIO_NUM_32);
  // gpio_hold_en(GPIO_NUM_8);
  gpio_deep_sleep_hold_en();  

  deepSleep();
}

void onWakeUp() {
  #ifndef DEBUG
    while (digitalRead(CHARGER_PIN)) {
      onCharger();
      delay(1000);
    }
  #endif

  uint32_t timeWakeUp;
  if (!digitalRead(POWER_PIN)) {
    timeWakeUp = millis();
    vTaskDelay(100);
  }
  while (!digitalRead(POWER_PIN)) {
    if (millis() - timeWakeUp > TIME_WAKEUP) {
      stateSleep = !stateSleep;
      Serial.printf("State sleep: %d\n",stateSleep);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();
      while(!digitalRead(POWER_PIN));
      vTaskDelay(100);
    }
  }
  onSleep(stateSleep);
}

void onPower() {
  #ifndef DEBUG
    if (digitalRead(CHARGER_PIN)) esp_cpu_reset(1);
  #endif
  uint32_t timeWakeUp;
  if (!digitalRead(POWER_PIN)) {
    timeWakeUp = millis();
    vTaskDelay(100);
  }
  while (!digitalRead(POWER_PIN)) {
    if (millis() - timeWakeUp > TIME_WAKEUP) {
      stateSleep = !stateSleep;
      Serial.printf("State sleep: %d\n",stateSleep);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();
      while(!digitalRead(POWER_PIN));
      vTaskDelay(100);
    }
  }

  if ((!stateSleep) && (!ble.stateConnect)) {
    if (!emgSensor.isReady()) {
      stateSleep = true;
      onSleep(stateSleep);
    }
    ble.startAdvertising();
    stateSleep = !ble.waitConnect();
    if (!stateSleep) {
      if(emgSensor.isReady()) {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();
      }
    }
  }

  onSleep(stateSleep);
}

uint8_t getBattery() {
  static float Vin = analogReadMilliVolts(BAT_PIN) * 2;   //resistor voltage divider
  UTILS_LOW_PASS_FILTER(Vin, analogReadMilliVolts(BAT_PIN) * 2, 0.01);
  int16_t batteryPercent = (Vin - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100;
  batteryPercent = constrain(batteryPercent, 0, 100);

  return batteryPercent;
}

void onCharger() {
  uint8_t batteryPercent = getBattery();

  Serial.print("Battery Percent");
  Serial.print("\t");
  Serial.println(batteryPercent);

  if (batteryPercent > 95)      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  else                          pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
}

void deepSleep() {
  // parameter Hibernate mode
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_AUTO);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_OFF);

  // enable RCT_IO wakeup
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);

  #ifndef DEBUG
    esp_sleep_enable_ext1_wakeup(PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  #endif

  //start sleep
  esp_deep_sleep_start();
}