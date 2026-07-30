#include <Arduino.h>
#include <NrfWfiHandler.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>

constexpr char DEVICE_NAME[] = "LuxWfi";
constexpr uint16_t CONNECTION_INTERVAL_UNITS = 160;  // 200 ms
constexpr uint16_t SLAVE_LATENCY = 0;
constexpr uint16_t SUPERVISION_TIMEOUT_UNITS = 2000; // 20 s
constexpr int8_t TX_POWER_DBM = 0;

Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash onboardFlash(&flashTransport);
BLEBas batteryService;
NrfWfiHandler nrfWfi;

void turnOffUserLeds() {
#ifdef LED_RED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
#endif
#ifdef LED_GREEN
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
#endif
#ifdef LED_BLUE
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
#endif
}

void putOnboardFlashInDeepPowerDown() {
  onboardFlash.begin();
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delay(10);
  onboardFlash.end();
}

void requestConnectionParameters(uint16_t connectionHandle) {
  BLEConnection* connection = Bluefruit.Connection(connectionHandle);
  if (connection != nullptr) {
    connection->requestConnectionParameter(
        CONNECTION_INTERVAL_UNITS,
        SLAVE_LATENCY,
        SUPERVISION_TIMEOUT_UNITS);
  }
}

void setup() {
  turnOffUserLeds();
  putOnboardFlashInDeepPowerDown();

  nrfWfi.setSuspendSysTickDuringSleep(true);
  nrfWfi.setMaskNonWakeInterruptsDuringSleep(false);
  nrfWfi.begin();

  Bluefruit.begin(1, 0);
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(TX_POWER_DBM);
  Bluefruit.setName(DEVICE_NAME);

  batteryService.begin();
  batteryService.write(100);

  Bluefruit.Periph.setConnInterval(
      CONNECTION_INTERVAL_UNITS,
      CONNECTION_INTERVAL_UNITS);
  Bluefruit.Periph.setConnSlaveLatency(SLAVE_LATENCY);
  Bluefruit.Periph.setConnSupervisionTimeout(
      SUPERVISION_TIMEOUT_UNITS);
  Bluefruit.Periph.setConnectCallback(requestConnectionParameters);

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(
      BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.ScanResponse.clearData();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(160, 1600); // 100 ms, puis 1 s
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
}

void loop() {
  // Les IRQ du SoftDevice reveillent WFI pour les evenements BLE.
  nrfWfi.sleepOnce();
}
