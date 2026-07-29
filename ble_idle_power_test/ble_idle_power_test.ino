#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>

// Minimal BLE peripheral used to measure the idle current of an established
// connection. There is no sensor access, Serial output, notification, or
// periodic application-level data transfer.
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash onboardFlash(&flashTransport);
BLEBas batteryService;

namespace {
constexpr char DEVICE_NAME[] = "LuxIdle";

// 500 ms connection interval, with up to four skipped peripheral events:
// when the central accepts these values and there is no traffic, the
// peripheral normally needs to listen only about once every 2.5 seconds.
// This conservative starting point is compatible with Windows BLE; it can be
// increased later after the idle link has been validated.
constexpr uint16_t CONNECTION_INTERVAL_UNITS = 400;  // 400 * 1.25 ms = 500 ms
constexpr uint16_t SLAVE_LATENCY = 4;
constexpr uint16_t SUPERVISION_TIMEOUT_UNITS = 1000;  // 1000 * 10 ms = 10 s

// A low value saves energy when the PC is close. Increase to -8, -4, or 0 dBm
// if the connection is unreliable; retransmissions can cost more than a
// slightly higher transmit power.
constexpr int8_t TX_POWER_DBM = -8;

bool putOnboardFlashInDeepPowerDown() {
  uint32_t idBefore = onboardFlash.getJEDECID();
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delay(10);
  uint32_t idAfter = onboardFlash.getJEDECID();
  return idBefore != 0 &&
         (idAfter == 0x00FFFFFFUL || idAfter == 0xFFFFFFFFUL);
}

void requestLowPowerConnection(uint16_t connectionHandle) {
  BLEConnection* connection = Bluefruit.Connection(connectionHandle);
  if (connection != nullptr) {
    // The central makes the final decision and may select different values.
    connection->requestConnectionParameter(
        CONNECTION_INTERVAL_UNITS,
        SLAVE_LATENCY,
        SUPERVISION_TIMEOUT_UNITS);
  }
}

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
}  // namespace

void setup() {
  turnOffUserLeds();

  // Seeed's official low-power example explicitly places the XIAO's onboard
  // QSPI flash in deep power-down. It otherwise remains an avoidable part of
  // the board's idle-current measurement.
  onboardFlash.begin();

  // One peripheral connection and no central role.
  Bluefruit.begin(1, 0);

  putOnboardFlashInDeepPowerDown();
  onboardFlash.end();

  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(TX_POWER_DBM);
  Bluefruit.setName(DEVICE_NAME);

  // A standard service from Bluefruit makes the device an explicit GATT
  // peripheral for Windows. Its static value is never notified or updated.
  batteryService.begin();
  batteryService.write(100);

  // Advertise slowly only while waiting for the PC. Connectable advertising
  // stops automatically as soon as the connection is established.
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.setInterval(1600, 1600);  // 1 s
  Bluefruit.Advertising.restartOnDisconnect(true);

  Bluefruit.Periph.setConnInterval(
      CONNECTION_INTERVAL_UNITS, CONNECTION_INTERVAL_UNITS);
  Bluefruit.Periph.setConnSlaveLatency(SLAVE_LATENCY);
  Bluefruit.Periph.setConnSupervisionTimeout(SUPERVISION_TIMEOUT_UNITS);
  Bluefruit.Periph.setConnectCallback(requestLowPowerConnection);

  Bluefruit.Advertising.start(0);

  // Leave the SoftDevice and callback tasks running, but remove Arduino's
  // application loop from the scheduler. Calling waitForEvent() repeatedly
  // from loop() causes continuous wakeups with this FreeRTOS/SoftDevice core.
  suspendLoop();
}

void loop() {
  // Suspended in setup().
}
