#include <Wire.h>
#include <Logger.h>
#include <Timer.h>
#include <stdarg.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>
#include "Adafruit_TSL2591.h"
#include <Adafruit_TinyUSB.h>
#include <NrfClock.h>
#include <NrfWfiHandler.h>
#include <WakeHandler.h>

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash onboardFlash(&flashTransport);
BLEUart bleUart;
struct LightReading;
struct BatteryReading {
  uint16_t adc;
  uint16_t millivolts;
  uint8_t percent;
};
NrfClock nrfClock;
NrfWfiHandler nrfWfi;
WakeHandler wakeHandler;

// USB diagnostic output. BLE is the diagnostic focus; light measurements are
// only logged when a sensor fault or an autorange transition needs attention.
Logger bootLog("BOOT", LOG_TRACE);
Logger bleLog("BLE", LOG_TRACE);
Logger sensorLog("TSL", LOG_NOTICE);

const WakeMask RTC_WAKE_MASK = WakeHandler::source(31);
const WakeMask BUTTON_WAKE_MASK = WakeHandler::source(0);

// External wiring (Seeed XIAO nRF52840).
// The two discrete LEDs are active-high: their anodes connect to the named
// GPIO through their current-limiting resistors and their cathodes to GND.
const uint8_t TSL2591_INTERRUPT_PIN = D6;
const uint8_t VOLTAGE_DIVIDER_PIN = A0;
const uint8_t RED_LED_PIN = D1;
const uint8_t GREEN_LED_PIN = D2;
const uint8_t BUTTON_PIN = D10;

const unsigned long BUTTON_DEBOUNCE_MS = 40UL;
const unsigned long BUTTON_LONG_PRESS_MS = 1500UL;
// The second *press* (not its release) validates the double-click. Two seconds
// makes the gesture comfortable while still being clearly intentional.
const unsigned long BUTTON_DOUBLE_PRESS_MS = 2000UL;
const unsigned long DEFAULT_LED_BLINK_INTERVAL_MS = 5000UL;
const unsigned long STATUS_BLINK_DURATION_MS = 150UL;
const unsigned long POWER_OFF_INDICATOR_MS = 2000UL;
const unsigned long POWER_ON_INDICATOR_MS = 2000UL;

// Battery divider calibration: battery -> 4.7 MOhm -> A0 -> 10 MOhm -> GND.
// Calibrated with 4.064 V measured at the battery (average ADC: ~969).
const uint16_t ADC_REFERENCE_MILLIVOLTS = 3300;
const uint16_t ADC_MAX_VALUE = 1023;
const uint16_t BATTERY_DIVIDER_RATIO_PER_MILLE = 1470;
const uint16_t BATTERY_CALIBRATION_PER_MILLE = 885;
const uint16_t BATTERY_EMPTY_MILLIVOLTS = 3200;
const uint16_t BATTERY_FULL_MILLIVOLTS = 4200;
const uint8_t BATTERY_ADC_SAMPLE_COUNT = 8;
const uint8_t BATTERY_ADC_FILTER_NEW_PERCENT = 15;

Timer buttonDebounceTimer(BUTTON_DEBOUNCE_MS);
Timer buttonLongPressTimer(BUTTON_LONG_PRESS_MS);
Timer buttonDoublePressTimer(BUTTON_DOUBLE_PRESS_MS);
Timer statusBlinkIntervalTimer(DEFAULT_LED_BLINK_INTERVAL_MS);
Timer statusBlinkDurationTimer(STATUS_BLINK_DURATION_MS);
Timer powerOffIndicatorTimer(POWER_OFF_INDICATOR_MS);
Timer powerOnIndicatorTimer(POWER_ON_INDICATOR_MS);

unsigned long firmwareNowMs() {
  return (unsigned long)nrfClock.monotonicMs();
}

// User-tunable defaults.
// refreshMs controls the internal sensor polling cadence. Keep it low enough
// for autorange and protection logic to react quickly.
const unsigned long DEFAULT_REFRESH_MS = 100;

// In auto mode, publish when lux changes by at least this percentage.
const float DEFAULT_PUBLISH_LUX_CHANGE_PERCENT = 1.0F;

// In auto mode this is the maximum time between publishes. In interval mode
// this is the fixed publish interval.
const unsigned long DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS = 30;
const uint16_t DEFAULT_INTEGRATION_MS = 200;
const bool DEFAULT_DISCARD_AFTER_GAIN_CHANGE = true;
const uint8_t DEFAULT_LED_BRIGHTNESS = 32;
const unsigned long MIN_LED_BLINK_INTERVAL_MS = 500UL;
const unsigned long MAX_LED_BLINK_INTERVAL_MS = 60000UL;

// BLE Nordic UART Service JSON line commands:
// - {"cmd":"get"} or {"cmd":"current"} immediately republishes the latest
//   cached reading without triggering a new sensor measurement.
// - {"cmd":"config.get"} returns the runtime configuration.
// - {"cmd":"config.set","refreshMs":250,"publishLuxChangePercent":2.0,
//    "publishMaxIntervalSeconds":60,"publishMode":"auto","integrationMs":200,
//    "discardAfterGainChange":true,"ledBrightness":32,
//    "ledBlinkIntervalMs":5000} updates runtime settings.
// publishMode accepts "auto" or "interval". Runtime settings are not persisted
// across reset.

// Internal autorange, saturation, and spectral compensation constants. Change
// these only when recalibrating sensor behavior.
const uint16_t SATURATION_COUNT = 65535;
const uint16_t HIGH_COUNT = 60000;
const uint16_t HOLD_COUNT = 45000;
const uint16_t SPECTRAL_OVERLOAD_COUNT = 12000;
const uint16_t SPECTRAL_RELEASE_COUNT = 10000;
const float RAW_FALLING_RELEASE_RATIO = 0.85F;
const float RAW_RISING_CONFIRM_RATIO = 1.15F;
const float LUX_DROP_HOLD_RATIO = 0.50F;
const float LUX_RISE_HOLD_RATIO = 2.00F;
const float SPECTRAL_OVERLOAD_IR_RATIO = 0.28F;
const float SPECTRAL_RELEASE_IR_RATIO = 0.24F;
const float TRUSTED_IR_RATIO = 0.27F;
const uint16_t LOW_COUNT = 1000;
const uint16_t COMMAND_MAX_LENGTH = 256;
const uint8_t STARTUP_SETTLING_READS = 2;
unsigned long lastReadMs = 0;
unsigned long lastBleHealthLogMs = 0;
uint32_t sensorReadCount = 0;
bool bleConnectEventPending = false;
bool bleDisconnectEventPending = false;
uint16_t pendingConnectionHandle = BLE_CONN_HANDLE_INVALID;
uint8_t pendingDisconnectReason = 0;
bool devicePowered = true;
bool buttonRawPressed = false;
bool buttonStablePressed = false;
bool buttonLongPressHandled = false;
bool buttonLongPressTracking = false;
bool waitingForSecondPress = false;
bool buttonPressDebounceActive = false;
volatile bool buttonPressInterruptPending = false;
bool powerOffPending = false;
bool powerOnIndicatorActive = false;

const char *FIRMWARE_VERSION = "tsl2591-ble-nus-2026-08-02-button-led-v2";
const char *BLE_DEVICE_NAME = "LuxSensor";
const uint8_t BLE_STATIC_ADDRESS[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xC3};
const uint16_t FAST_CONNECTION_INTERVAL_MIN_UNITS = 12; // 15 ms
const uint16_t FAST_CONNECTION_INTERVAL_MAX_UNITS = 24; // 30 ms
// Windows proved unreliable with a 500 ms interval, latency 4 and -8 dBm:
// established links frequently disappeared shortly after the parameter
// update. Keep a moderate interval and attend every connection event. The
// extra radio time is preferable to repeated discovery and reconnection.
const uint16_t LOW_POWER_CONNECTION_INTERVAL_UNITS = 160; // 200 ms
const uint16_t LOW_POWER_CONNECTION_SLAVE_LATENCY = 0;
const uint16_t CONNECTION_SUPERVISION_TIMEOUT_UNITS = 2000; // 20 s
const int8_t BLE_TX_POWER_DBM = 0;
const unsigned long BLE_LOW_POWER_SETTLE_MS = 15000UL;
// The Windows client sends a heartbeat every 30 s. Leave enough margin for a
// temporarily delayed WinRT write without retaining an abandoned link
// indefinitely.
const unsigned long BLE_COMMAND_WATCHDOG_MS = 120000UL;

const tsl2591Gain_t GAIN_STEPS[] = {
  TSL2591_GAIN_LOW,
  TSL2591_GAIN_MED,
  TSL2591_GAIN_HIGH,
  TSL2591_GAIN_MAX
};
const char *GAIN_NAMES[] = {"low", "med", "high", "max"};
const float GAIN_FACTORS[] = {1.0F, 25.0F, 428.0F, 9876.0F};
const uint8_t GAIN_STEP_COUNT = sizeof(GAIN_STEPS) / sizeof(GAIN_STEPS[0]);

enum PublishMode {
  PUBLISH_MODE_AUTO,
  PUBLISH_MODE_INTERVAL
};

const PublishMode DEFAULT_PUBLISH_MODE = PUBLISH_MODE_AUTO;

const uint8_t QUALITY_SATURATED = 1 << 0;
const uint8_t QUALITY_SPECTRAL = 1 << 1;
const uint8_t QUALITY_HELD = 1 << 2;
const uint8_t QUALITY_ESTIMATED = 1 << 3;
const uint8_t QUALITY_GAIN_SETTLED = 1 << 4;
const uint8_t QUALITY_INVALID_CHANNELS = 1 << 5;

uint8_t gainIndex = 1;
float lastValidLux = NAN;
float lastValidRawLevel = NAN;
uint16_t lastValidVisible = 0;
float lastPublishedLux = NAN;
uint16_t lastPublishedVisible = 0;
float luxPerRawLevel = 1.6F;
bool spectralHoldLatched = false;
bool hasSentReading = false;
bool lastSentLuxWasValid = false;
float lastSentLux = NAN;
unsigned long lastSentMs = 0;
bool bleWasConnected = false;
uint16_t bleConnectionHandle = BLE_CONN_HANDLE_INVALID;
unsigned long lastBleCommandMs = 0;
bool bleCommandSeen = false;
bool lowPowerConnectionRequested = false;
String bleRxLine = "";
unsigned long refreshMs = DEFAULT_REFRESH_MS;
float publishLuxChangePercent = DEFAULT_PUBLISH_LUX_CHANGE_PERCENT;
unsigned long publishMaxIntervalSeconds = DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS;
PublishMode publishMode = DEFAULT_PUBLISH_MODE;
uint16_t integrationMs = DEFAULT_INTEGRATION_MS;
tsl2591IntegrationTime_t integrationTime = TSL2591_INTEGRATIONTIME_200MS;
bool discardAfterGainChange = DEFAULT_DISCARD_AFTER_GAIN_CHANGE;
uint8_t ledBrightness = DEFAULT_LED_BRIGHTNESS;
unsigned long ledBlinkIntervalMs = DEFAULT_LED_BLINK_INTERVAL_MS;
uint32_t filteredBatteryAdcQ8 = 0;
bool batteryAdcFilterInitialized = false;
bool batteryAdcPrimed = false;
bool usbStateKnown = false;
bool lastUsbConnected = false;
bool usbStateNotificationPending = false;
bool pendingUsbBatteryValid = false;
uint16_t pendingUsbBatteryMillivolts = 0;
uint8_t pendingUsbBatteryPercent = 0;
volatile bool usbInitialStatePending = false;

struct LightReading {
  uint16_t full;
  uint16_t ir;
  uint16_t visible;
  float lux;
  bool adcOverRange;
  bool saturated;
  bool valid;
  bool held;
  bool estimated;
  float rawLevel;
  float irRatio;
  bool spectralOverload;
  bool invalidChannels;
  bool gainSettled;
  uint8_t quality;
};

// One complete measurement fits in a single 20-byte ATT notification.
// All multi-byte fields use the nRF52840's little-endian byte order.
struct __attribute__((packed)) BleMeasurementPacket {
  char magic[2];
  uint8_t version;
  uint8_t quality;
  float lux;
  uint16_t visible;
  uint16_t ir;
  uint16_t full;
  uint8_t gain;
  uint16_t batteryMillivolts;
  uint8_t batteryPercent;
};

static_assert(sizeof(BleMeasurementPacket) == 18,
              "Unexpected BLE measurement packet size");

LightReading cachedReading;
bool hasCachedReading = false;

void logf(Logger& logger, LogLevel level, const char* format, ...) {
  char message[128];
  va_list args;
  va_start(args, format);
  vsnprintf(message, sizeof(message), format, args);
  va_end(args);

  switch (level) {
    case LOG_TRACE:
      logger.trace(message);
      break;
    case LOG_DEBUG:
      logger.debug(message);
      break;
    case LOG_NOTICE:
      logger.notice(message);
      break;
    case LOG_WARN:
      logger.warn(message);
      break;
    case LOG_ERROR:
      logger.error(message);
      break;
  }
}

void scanI2cBus() {
  uint8_t found = 0;
  bool tslFound = false;
  bootLog.notice("I2C scan start");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t status = Wire.endTransmission();
    if (status == 0) {
      logf(bootLog, LOG_NOTICE, "I2C device address=0x%02X", address);
      found++;
      tslFound = tslFound || address == 0x29;
    }
  }
  logf(bootLog, tslFound ? LOG_NOTICE : LOG_WARN,
       "I2C scan complete devices=%u tsl2591_0x29=%s", found,
       tslFound ? "present" : "missing");
}

void logReading(const LightReading& reading, const char* phase) {
  logf(sensorLog, LOG_TRACE,
       "%s reads=%lu full=%u ir=%u visible=%u lux=%.3f gain=%s raw=%.2f "
       "irRatio=%.3f",
       phase, (unsigned long)sensorReadCount, reading.full, reading.ir,
       reading.visible, reading.lux, GAIN_NAMES[gainIndex], reading.rawLevel,
       reading.irRatio);
  logf(sensorLog, LOG_TRACE,
       "flags valid=%u saturated=%u spectral=%u held=%u estimated=%u "
       "invalidChannels=%u gainSettled=%u quality=0x%02X",
       reading.valid, reading.saturated,
       reading.spectralOverload, reading.held, reading.estimated,
       reading.invalidChannels, reading.gainSettled, reading.quality);
}

void logBleHealth(unsigned long now) {
  if (now - lastBleHealthLogMs < 5000UL) {
    return;
  }
  lastBleHealthLogMs = now;

  if (!Bluefruit.connected()) {
    logf(bleLog, LOG_DEBUG, "advertising=%u uptimeMs=%lu cachedReading=%u",
         Bluefruit.Advertising.isRunning(), now, hasCachedReading);
    return;
  }

  BLEConnection* connection = Bluefruit.Connection(bleConnectionHandle);
  if (connection == nullptr) {
    bleLog.warn("connected state has no BLEConnection object");
    return;
  }

  logf(bleLog, LOG_DEBUG,
       "link handle=%u rssi=%d dBm mtu=%u interval=%.2f ms latency=%u "
       "sinceCommandMs=%lu notifications=%u",
       bleConnectionHandle, connection->getRssi(), connection->getMtu(),
       connection->getConnectionInterval() * 1.25F,
       connection->getSlaveLatency(), now - lastBleCommandMs,
       hasSentReading);
}

BatteryReading readBattery() {
  // Conversions occur only when a normal BLE publication is already due.
  // Average the same eight back-to-back samples used for calibration; this
  // adds neither a delay nor a periodic wake source.
  // The first conversion after boot can retain a stale sample-capacitor
  // value, so discard it once before seeding the filter.
  if (!batteryAdcPrimed) {
    analogRead(VOLTAGE_DIVIDER_PIN);
    batteryAdcPrimed = true;
  }
  uint32_t total = 0;
  for (uint8_t sample = 0; sample < BATTERY_ADC_SAMPLE_COUNT; sample++) {
    total += analogRead(VOLTAGE_DIVIDER_PIN);
  }
  uint32_t adcQ8 = (total * 256UL + BATTERY_ADC_SAMPLE_COUNT / 2) /
                   BATTERY_ADC_SAMPLE_COUNT;
  if (!batteryAdcFilterInitialized) {
    filteredBatteryAdcQ8 = adcQ8;
    batteryAdcFilterInitialized = true;
  } else {
    filteredBatteryAdcQ8 =
        (filteredBatteryAdcQ8 *
             (100U - BATTERY_ADC_FILTER_NEW_PERCENT) +
         adcQ8 * BATTERY_ADC_FILTER_NEW_PERCENT + 50U) /
        100U;
  }

  uint16_t adc = (uint16_t)((filteredBatteryAdcQ8 + 128U) / 256U);
  const uint64_t voltageNumerator =
      (uint64_t)adc * ADC_REFERENCE_MILLIVOLTS *
      BATTERY_DIVIDER_RATIO_PER_MILLE * BATTERY_CALIBRATION_PER_MILLE;
  const uint64_t voltageDenominator =
      (uint64_t)ADC_MAX_VALUE * 1000ULL * 1000ULL;
  uint16_t millivolts =
      (uint16_t)((voltageNumerator + voltageDenominator / 2) /
                 voltageDenominator);

  uint8_t percent = 0;
  if (millivolts >= BATTERY_FULL_MILLIVOLTS) {
    percent = 100;
  } else if (millivolts > BATTERY_EMPTY_MILLIVOLTS) {
    percent = (uint8_t)((millivolts - BATTERY_EMPTY_MILLIVOLTS + 5U) /
                        10U);
  }

  BatteryReading reading;
  reading.adc = adc;
  reading.millivolts = millivolts;
  reading.percent = percent;
  return reading;
}

bool putOnboardFlashInDeepPowerDown() {
  uint32_t idBefore = onboardFlash.getJEDECID();
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delay(10);
  uint32_t idAfter = onboardFlash.getJEDECID();
  return idBefore != 0 &&
         (idAfter == 0x00FFFFFFUL || idAfter == 0xFFFFFFFFUL);
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

// The TSL2591 INT output and push button wake the MCU from WFI. The button is
// wired between D10 and GND, therefore INPUT_PULLUP reads LOW when pressed.
// Keep the ISR minimal: WakeHandler carries the event to NrfWfiHandler.
void onTslInterrupt() {}

void onButtonInterrupt() {
  // Latch only the press edge. The main loop confirms it after debounce, so a
  // brief click remains available even if it is released before the CPU runs.
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonPressInterruptPending = true;
  }
  wakeHandler.notify(BUTTON_WAKE_MASK);
}

void configureExternalHardware() {
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);

  pinMode(VOLTAGE_DIVIDER_PIN, INPUT);
  pinMode(TSL2591_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TSL2591_INTERRUPT_PIN),
                  onTslInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),
                  onButtonInterrupt, CHANGE);
  buttonRawPressed = digitalRead(BUTTON_PIN) == LOW;
  buttonStablePressed = buttonRawPressed;
}

void setStatusLeds(bool redOn, bool greenOn) {
  analogWrite(RED_LED_PIN, redOn ? ledBrightness : 0);
  analogWrite(GREEN_LED_PIN, greenOn ? ledBrightness : 0);
}

void restartBleAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.start(0);
}

void forceBleReinitialization() {
  bleLog.warn("button double press: forcing BLE reinitialization");
  resetPublishState();
  if (Bluefruit.connected() &&
      bleConnectionHandle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(bleConnectionHandle);
    return;
  }
  restartBleAdvertising();
}

void beginStartupIndicator() {
  powerOnIndicatorActive = true;
  powerOnIndicatorTimer.start();
  setStatusLeds(false, true);
}

void beginDevicePowerOff() {
  if (!devicePowered || powerOffPending) {
    return;
  }
  powerOffPending = true;
  waitingForSecondPress = false;
  powerOffIndicatorTimer.start();
  setStatusLeds(true, false);
  bleLog.notice("button long press: power-off indicator started");
}

void turnDeviceOff() {
  if (!devicePowered) {
    return;
  }
  devicePowered = false;
  powerOffPending = false;
  powerOnIndicatorActive = false;
  waitingForSecondPress = false;
  setStatusLeds(false, false);
  if (Bluefruit.connected() &&
      bleConnectionHandle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(bleConnectionHandle);
  }
  Bluefruit.Advertising.stop();
  tsl.disable();
  bleLog.notice("button long press: device switched off");
}

void turnDeviceOn(unsigned long now) {
  if (devicePowered) {
    return;
  }
  devicePowered = true;
  resetLightState();
  resetPublishState();
  configureSensor();
  tsl.enable();
  lastReadMs = now;
  powerOffPending = false;
  restartBleAdvertising();
  beginStartupIndicator();
  bleLog.notice("button short press: device switched on; BLE advertising");
}

void updateStatusLed(unsigned long now, bool bleConnected) {
  (void)now;
  if (!devicePowered) {
    setStatusLeds(false, false);
    return;
  }

  if (powerOffPending) {
    setStatusLeds(true, false);
    return;
  }

  if (powerOnIndicatorActive) {
    setStatusLeds(false, true);
    return;
  }

  if (statusBlinkDurationTimer.isReached()) {
    setStatusLeds(false, false);
  }

  if (!statusBlinkIntervalTimer.isReached()) {
    return;
  }
  statusBlinkIntervalTimer.start();
  statusBlinkDurationTimer.start();
  setStatusLeds(!bleConnected, bleConnected);
}

void updatePowerIndicators() {
  if (powerOffPending && powerOffIndicatorTimer.isReached()) {
    turnDeviceOff();
    return;
  }

  if (powerOnIndicatorActive && powerOnIndicatorTimer.isReached()) {
    powerOnIndicatorActive = false;
    setStatusLeds(false, false);
    statusBlinkIntervalTimer.setDuration(ledBlinkIntervalMs);
    statusBlinkIntervalTimer.start();
    bleLog.notice("power-on indicator complete; status blinking enabled");
  }
}

void handleButton(unsigned long now) {
  bool rawPressed = digitalRead(BUTTON_PIN) == LOW;

  // A long press is only valid while the button is continuously held. Do not
  // reuse the first short click's timer when the user begins a second click.
  if (rawPressed && !buttonRawPressed) {
    buttonRawPressed = true;
    buttonLongPressTimer.start();
    buttonLongPressHandled = false;
    buttonLongPressTracking = true;
  } else if (!rawPressed && buttonRawPressed) {
    buttonRawPressed = false;
    buttonLongPressTracking = false;
  }

  bool pressPending;
  noInterrupts();
  pressPending = buttonPressInterruptPending;
  interrupts();

  if (pressPending && !buttonPressDebounceActive) {
    buttonDebounceTimer.start();
    buttonPressDebounceActive = true;
  }

  if (buttonPressDebounceActive && buttonDebounceTimer.isReached()) {
    noInterrupts();
    buttonPressInterruptPending = false;
    interrupts();
    buttonPressDebounceActive = false;
    buttonStablePressed = rawPressed;
    bleLog.debug("button press interrupt confirmed");

    if (!devicePowered) {
      turnDeviceOn(now);
      return;
    }
    if (waitingForSecondPress && !buttonDoublePressTimer.isReached()) {
      waitingForSecondPress = false;
      buttonLongPressHandled = true;
      buttonLongPressTracking = false;
      bleLog.notice("button double press detected; resetting BLE link");
      forceBleReinitialization();
      return;
    }
    waitingForSecondPress = true;
    buttonDoublePressTimer.start();
    bleLog.debug("button first short press; awaiting second press");
  }

  if (devicePowered && buttonLongPressTracking && !buttonLongPressHandled &&
      buttonLongPressTimer.isReached()) {
    buttonLongPressHandled = true;
    beginDevicePowerOff();
    return;
  }

  if (waitingForSecondPress && buttonDoublePressTimer.isReached()) {
    waitingForSecondPress = false;
    bleLog.debug("single short press while on: no action");
  }
}

void onBleConnected(uint16_t connectionHandle) {
  bleConnectionHandle = connectionHandle;
  pendingConnectionHandle = connectionHandle;
  bleConnectEventPending = true;
  lastBleCommandMs = firmwareNowMs();
  bleCommandSeen = false;
  lowPowerConnectionRequested = false;
  usbInitialStatePending = true;
  usbStateNotificationPending = false;
  resetPublishState();
}

void onBleDisconnected(uint16_t connectionHandle, uint8_t reason) {
  pendingConnectionHandle = connectionHandle;
  pendingDisconnectReason = reason;
  bleDisconnectEventPending = true;
  bleConnectionHandle = BLE_CONN_HANDLE_INVALID;
  lastBleCommandMs = 0;
  bleCommandSeen = false;
  lowPowerConnectionRequested = false;
  usbInitialStatePending = false;
  usbStateNotificationPending = false;
  bleRxLine = "";
  bleWasConnected = false;
}

void configureBle() {
  // Allow a complete JSON response to fit in one ATT notification when the
  // central negotiates the maximum MTU. This also increases the SoftDevice
  // notification queue from one entry to three; idle connection parameters
  // and radio duty cycle remain configured separately below.
  bootLog.notice("BLE stack setup start");
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);
  ble_gap_addr_t address = {};
  address.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
  memcpy(address.addr, BLE_STATIC_ADDRESS, sizeof(BLE_STATIC_ADDRESS));
  Bluefruit.setAddr(&address);
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(BLE_TX_POWER_DBM);
  Bluefruit.setName(BLE_DEVICE_NAME);

  bleUart.begin();

  Bluefruit.Periph.setConnInterval(
      FAST_CONNECTION_INTERVAL_MIN_UNITS,
      FAST_CONNECTION_INTERVAL_MAX_UNITS);
  Bluefruit.Periph.setConnSlaveLatency(0);
  Bluefruit.Periph.setConnSupervisionTimeout(
      CONNECTION_SUPERVISION_TIMEOUT_UNITS);
  Bluefruit.Periph.setConnectCallback(onBleConnected);
  Bluefruit.Periph.setDisconnectCallback(onBleDisconnected);

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.ScanResponse.clearData();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(160, 1600); // 100 ms, then 1 s
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
  logf(bootLog, LOG_NOTICE,
       "BLE advertising name=%s addr=%02X:%02X:%02X:%02X:%02X:%02X tx=%d "
       "fastIntervalMs=100 slowIntervalMs=1000",
       BLE_DEVICE_NAME, BLE_STATIC_ADDRESS[5], BLE_STATIC_ADDRESS[4],
       BLE_STATIC_ADDRESS[3], BLE_STATIC_ADDRESS[2], BLE_STATIC_ADDRESS[1],
       BLE_STATIC_ADDRESS[0], BLE_TX_POWER_DBM);
}

bool integrationTimeForMs(uint16_t ms, tsl2591IntegrationTime_t& timing) {
  switch (ms) {
    case 100:
      timing = TSL2591_INTEGRATIONTIME_100MS;
      return true;
    case 200:
      timing = TSL2591_INTEGRATIONTIME_200MS;
      return true;
    case 300:
      timing = TSL2591_INTEGRATIONTIME_300MS;
      return true;
    case 400:
      timing = TSL2591_INTEGRATIONTIME_400MS;
      return true;
    case 500:
      timing = TSL2591_INTEGRATIONTIME_500MS;
      return true;
    case 600:
      timing = TSL2591_INTEGRATIONTIME_600MS;
      return true;
  }
  return false;
}

void configureSensor() {
  tsl.setTiming(integrationTime);
  tsl.setGain(GAIN_STEPS[gainIndex]);
  logf(sensorLog, LOG_NOTICE, "configured integrationMs=%u gain=%s",
       integrationMs, GAIN_NAMES[gainIndex]);
}

void setGainIndex(uint8_t newGainIndex) {
  if (newGainIndex == gainIndex) {
    return;
  }

  gainIndex = newGainIndex;
  tsl.setGain(GAIN_STEPS[gainIndex]);
  logf(sensorLog, LOG_NOTICE, "autorange gain=%s index=%u",
       GAIN_NAMES[gainIndex], gainIndex);
}

void discardGainSettlingReading() {
  if (!discardAfterGainChange) {
    return;
  }
  sensorLog.debug("discarding settling sample after gain change");
  tsl.getFullLuminosity();
}

void updateReadingQuality(LightReading& reading) {
  uint8_t quality = 0;
  if (reading.saturated || reading.adcOverRange) {
    quality |= QUALITY_SATURATED;
  }
  if (reading.spectralOverload) {
    quality |= QUALITY_SPECTRAL;
  }
  if (reading.held) {
    quality |= QUALITY_HELD;
  }
  if (reading.estimated) {
    quality |= QUALITY_ESTIMATED;
  }
  if (reading.gainSettled) {
    quality |= QUALITY_GAIN_SETTLED;
  }
  if (reading.invalidChannels) {
    quality |= QUALITY_INVALID_CHANNELS;
  }
  reading.quality = quality;
}

void resetLightState() {
  lastValidLux = NAN;
  lastValidRawLevel = NAN;
  lastValidVisible = 0;
  lastPublishedLux = NAN;
  lastPublishedVisible = 0;
  spectralHoldLatched = false;
  hasCachedReading = false;
}

void setIntegrationMs(uint16_t value) {
  tsl2591IntegrationTime_t timing;
  if (!integrationTimeForMs(value, timing)) {
    return;
  }
  integrationMs = value;
  integrationTime = timing;
  tsl.setTiming(integrationTime);
  logf(sensorLog, LOG_NOTICE, "integration time changed integrationMs=%u",
       integrationMs);
}

LightReading readSensorOnce() {
  uint32_t luminosity = tsl.getFullLuminosity();
  sensorReadCount++;
  uint16_t ir = luminosity >> 16;
  uint16_t full = luminosity & 0xFFFF;
  uint16_t visible = full > ir ? full - ir : 0;
  // CH1 (IR) is a subset of CH0 (full spectrum). During sensor or power
  // settling, a split register read can transiently return CH1 > CH0. Passing
  // full=0 and ir>0 to Adafruit's formula produces +infinity, which used to
  // poison the stabilization cache across all subsequent readings.
  bool invalidChannels = ir > full;
  float lux = NAN;
  if (!invalidChannels) {
    // Avoid the 0/0 term in Adafruit's formula in genuine complete darkness.
    lux = full == 0 ? 0.0F : tsl.calculateLux(full, ir);
  }
  bool finiteLux = !isnan(lux) && !isinf(lux);
  bool hardSaturated = full >= SATURATION_COUNT || ir >= SATURATION_COUNT;
  bool adcOverRange = hardSaturated || full >= HIGH_COUNT || ir >= HIGH_COUNT;
  float irRatio = full > 0 ? (float)ir / (float)full : 0.0F;
  uint16_t peak = full > ir ? full : ir;
  float rawLevel = (float)peak / GAIN_FACTORS[gainIndex];
  bool rawLevelIsBackDown = !isnan(lastValidRawLevel) &&
                            rawLevel < lastValidRawLevel * RAW_FALLING_RELEASE_RATIO;
  bool spectralEnter = full >= SPECTRAL_OVERLOAD_COUNT &&
                       irRatio >= SPECTRAL_OVERLOAD_IR_RATIO;
  bool spectralRelease = full < SPECTRAL_RELEASE_COUNT ||
                         irRatio < SPECTRAL_RELEASE_IR_RATIO ||
                         rawLevelIsBackDown;

  if (spectralEnter) {
    spectralHoldLatched = true;
  } else if (spectralRelease) {
    spectralHoldLatched = false;
  }

  bool spectralOverload = spectralEnter || spectralHoldLatched;
  bool saturated = adcOverRange || spectralOverload;
  bool valid = !invalidChannels && !hardSaturated && finiteLux && lux >= 0;
  LightReading reading = {full, ir, visible, lux, adcOverRange, saturated,
                          valid, false, false, rawLevel, irRatio,
                          spectralOverload, invalidChannels, false, 0};
  updateReadingQuality(reading);
  if (!reading.valid || reading.saturated || reading.invalidChannels) {
    logReading(reading, "I2C luminosity read complete");
  }
  return reading;
}

LightReading readSensorAutoRange() {
  LightReading reading;
  bool overRangeDuringMeasurement = false;

  for (uint8_t attempt = 0; attempt < GAIN_STEP_COUNT; attempt++) {
    reading = readSensorOnce();
    uint16_t peak = reading.full > reading.ir ? reading.full : reading.ir;

    if ((reading.adcOverRange || peak >= HIGH_COUNT ||
         (reading.spectralOverload && gainIndex > 1)) &&
        gainIndex > 0) {
      overRangeDuringMeasurement = true;
      setGainIndex(gainIndex - 1);
      discardGainSettlingReading();
      continue;
    }

    if (reading.valid && peak <= LOW_COUNT && gainIndex + 1 < GAIN_STEP_COUNT) {
      setGainIndex(gainIndex + 1);
      discardGainSettlingReading();
      continue;
    }

    break;
  }

  if (reading.valid) {
    uint16_t peak = reading.full > reading.ir ? reading.full : reading.ir;
    if (!reading.spectralOverload && reading.irRatio <= TRUSTED_IR_RATIO &&
        reading.rawLevel > 0.0F) {
      float currentLuxPerRaw = reading.lux / reading.rawLevel;
      if (isnan(luxPerRawLevel)) {
        luxPerRawLevel = currentLuxPerRaw;
      } else {
        luxPerRawLevel = (luxPerRawLevel * 0.85F) + (currentLuxPerRaw * 0.15F);
      }
    }

    if (reading.spectralOverload && !isnan(luxPerRawLevel)) {
      float estimatedLux = reading.rawLevel * luxPerRawLevel;
      if (!isinf(estimatedLux) && estimatedLux > reading.lux) {
        reading.lux = estimatedLux;
        reading.estimated = true;
      }
    }

    bool rawLevelIsFalling = !isnan(lastValidRawLevel) &&
                             reading.rawLevel < lastValidRawLevel * RAW_FALLING_RELEASE_RATIO;
    bool rawLevelConfirmsRise = !isnan(lastValidRawLevel) &&
                                reading.rawLevel > lastValidRawLevel * RAW_RISING_CONFIRM_RATIO;
    bool publishedDrop = !isnan(lastPublishedLux) &&
                         reading.lux < lastPublishedLux * LUX_DROP_HOLD_RATIO;
    bool publishedRise = !isnan(lastPublishedLux) &&
                         lastPublishedLux > 0.0F &&
                         reading.lux > lastPublishedLux * LUX_RISE_HOLD_RATIO;
    bool incoherentDrop = publishedDrop && !rawLevelIsFalling;
    bool incoherentRise = publishedRise && !rawLevelConfirmsRise;
    bool settlingAfterGainChange = overRangeDuringMeasurement && discardAfterGainChange;

    if (settlingAfterGainChange && isnan(lastPublishedLux)) {
      reading.lux = NAN;
      reading.visible = 0;
      reading.valid = false;
      reading.held = true;
    } else if (settlingAfterGainChange || incoherentDrop || incoherentRise) {
      reading.lux = lastPublishedLux;
      reading.visible = lastPublishedVisible;
      reading.held = true;
    } else {
      lastValidLux = reading.lux;
      lastValidRawLevel = reading.rawLevel;
      lastValidVisible = reading.visible;
      lastPublishedLux = reading.lux;
      lastPublishedVisible = reading.visible;
    }
  } else if (!isnan(lastValidLux)) {
    reading.lux = lastPublishedLux;
    reading.visible = lastPublishedVisible;
    reading.held = true;
  }

  reading.gainSettled = overRangeDuringMeasurement && discardAfterGainChange;
  updateReadingQuality(reading);
  return reading;
}

bool shouldPublishReading(float lux, unsigned long now) {
  bool luxIsValid = !isnan(lux) && !isinf(lux) && lux >= 0.0F;

  if (!hasSentReading || luxIsValid != lastSentLuxWasValid) {
    return true;
  }

  if (now - lastSentMs >= publishMaxIntervalSeconds * 1000UL) {
    return true;
  }

  if (publishMode == PUBLISH_MODE_INTERVAL) {
    return false;
  }

  if (!luxIsValid) {
    return false;
  }

  if (lastSentLux <= 0.0F) {
    return lux != lastSentLux;
  }

  float changePercent = fabs((lux - lastSentLux) / lastSentLux) * 100.0F;
  return changePercent >= publishLuxChangePercent;
}

void rememberSentLux(float lux, unsigned long now) {
  hasSentReading = true;
  lastSentLuxWasValid = !isnan(lux) && !isinf(lux) && lux >= 0.0F;
  lastSentLux = lux;
  lastSentMs = now;
}

bool writeBleReliably(const uint8_t *data, size_t length) {
  size_t offset = 0;
  unsigned long deadline = millis() + 10000UL;
  size_t maxChunkLength = 20;
  BLEConnection *connection = Bluefruit.Connection(bleConnectionHandle);
  if (connection != nullptr && connection->getMtu() > 3) {
    maxChunkLength = connection->getMtu() - 3;
  }

  while (offset < length && Bluefruit.connected()) {
    size_t chunkLength = min(maxChunkLength, length - offset);
    size_t written = bleUart.write(data + offset, chunkLength);
    if (written > 0) {
      offset += written;
      continue;
    }

    if ((long)(millis() - deadline) >= 0) {
      break;
    }
    delay(5);
  }

  bool complete = offset == length;
  if (!complete) {
    logf(bleLog, LOG_WARN,
         "BLE write incomplete sent=%u expected=%u connected=%u",
         (unsigned)offset, (unsigned)length, Bluefruit.connected());
  }
  return complete;
}

void sendJsonLine(JsonDocument& document) {
  String line;
  line.reserve(320);
  serializeJson(document, line);
  logf(bleLog, LOG_DEBUG, "BLE JSON TX %s", line.c_str());
  const uint8_t *data =
      reinterpret_cast<const uint8_t *>(line.c_str());
  if (writeBleReliably(data, line.length())) {
    const uint8_t newline = '\n';
    writeBleReliably(&newline, 1);
  }
}

bool readUsbConnected() {
  uint32_t usbRegStatus = 0;
  if (sd_power_usbregstatus_get(&usbRegStatus) != NRF_SUCCESS ||
      (usbRegStatus & POWER_USBREGSTATUS_VBUSDETECT_Msk) == 0) {
    return false;
  }

  return !TinyUSBDevice.mounted() || !TinyUSBDevice.suspended();
}

void queueUsbStateNotification(bool connected) {
  usbStateNotificationPending = true;
  pendingUsbBatteryValid = false;
  if (!connected) {
    // USB-biased samples must not leak gradually into the battery display.
    // Re-seed the EMA from one fresh eight-sample average on this transition.
    batteryAdcFilterInitialized = false;
    BatteryReading battery = readBattery();
    pendingUsbBatteryMillivolts = battery.millivolts;
    pendingUsbBatteryPercent = battery.percent;
    pendingUsbBatteryValid = true;
  }
}

bool sendUsbState(bool connected) {
  char message[112];
  if (pendingUsbBatteryValid) {
    snprintf(message, sizeof(message),
             "{\"type\":\"usb\",\"connected\":false,"
             "\"batteryMillivolts\":%u,\"batteryPercent\":%u}\n",
             pendingUsbBatteryMillivolts, pendingUsbBatteryPercent);
  } else {
    snprintf(message, sizeof(message),
             "{\"type\":\"usb\",\"connected\":%s}\n",
             connected ? "true" : "false");
  }
  bool sent = writeBleReliably(
      reinterpret_cast<const uint8_t *>(message), strlen(message));
  if (sent) {
    logf(bleLog, LOG_NOTICE,
         "USB state TX connected=%u battery=%u%% (%umV valid=%u)",
         connected, pendingUsbBatteryPercent,
         pendingUsbBatteryMillivolts, pendingUsbBatteryValid);
  }
  return sent;
}

void serviceUsbState(bool bleConnected) {
  bool connected = readUsbConnected();
  if (!usbStateKnown || connected != lastUsbConnected) {
    usbStateKnown = true;
    lastUsbConnected = connected;
    if (bleConnected) {
      queueUsbStateNotification(connected);
    } else if (!connected) {
      batteryAdcFilterInitialized = false;
    }
  }

  // The connect callback occurs before Windows subscribes to NUS TX.
  // Wait for the CCCD notification bit so the initial state cannot be lost.
  if (usbInitialStatePending && bleUart.notifyEnabled()) {
    usbInitialStatePending = false;
    queueUsbStateNotification(lastUsbConnected);
  }

  if (bleConnected && usbStateNotificationPending &&
      bleUart.notifyEnabled() && sendUsbState(lastUsbConnected)) {
    usbStateNotificationPending = false;
    pendingUsbBatteryValid = false;
  }
}

void publishReading(const LightReading& reading, unsigned long now) {
  BatteryReading battery = readBattery();
  BleMeasurementPacket packet;
  packet.magic[0] = 'L';
  packet.magic[1] = 'T';
  packet.version = 2;
  packet.quality = reading.quality;
  packet.lux = (!isnan(reading.lux) && !isinf(reading.lux) &&
                reading.lux >= 0.0F)
                   ? reading.lux
                   : NAN;
  packet.visible = reading.visible;
  packet.ir = reading.ir;
  packet.full = reading.full;
  packet.gain = gainIndex;
  packet.batteryMillivolts = battery.millivolts;
  packet.batteryPercent = battery.percent;
  if (writeBleReliably(reinterpret_cast<const uint8_t *>(&packet),
                       sizeof(packet))) {
    rememberSentLux(reading.lux, now);
    logf(bleLog, LOG_DEBUG,
         "measurement notification TX bytes=%u quality=0x%02X gain=%s battery=%u%% (%umV adc=%u)",
         (unsigned)sizeof(packet), reading.quality, GAIN_NAMES[gainIndex],
         battery.percent, battery.millivolts, battery.adc);
  } else {
    sensorLog.warn("BLE measurement TX failed");
  }
}

void cacheReading(const LightReading& reading) {
  cachedReading = reading;
  hasCachedReading = true;
}

void sendCommandError(const char* cmd, const char* error) {
  logf(bleLog, LOG_WARN, "command error cmd=%s error=%s",
       cmd == nullptr ? "" : cmd, error == nullptr ? "" : error);
  JsonDocument response;
  response["type"] = "response";
  response["ok"] = false;
  if (cmd != nullptr && cmd[0] != '\0') {
    response["cmd"] = cmd;
  }
  response["error"] = error;
  sendJsonLine(response);
}

const char* publishModeName() {
  return publishMode == PUBLISH_MODE_INTERVAL ? "interval" : "auto";
}

void sendConfigResponse(const char* cmd) {
  JsonDocument response;
  response["type"] = "response";
  response["ok"] = true;
  response["cmd"] = cmd;
  JsonObject config = response["config"].to<JsonObject>();
  config["refreshMs"] = refreshMs;
  config["publishLuxChangePercent"] = publishLuxChangePercent;
  config["publishMaxIntervalSeconds"] = publishMaxIntervalSeconds;
  config["publishMode"] = publishModeName();
  config["integrationMs"] = integrationMs;
  config["discardAfterGainChange"] = discardAfterGainChange;
  config["ledBrightness"] = ledBrightness;
  config["ledBlinkIntervalMs"] = ledBlinkIntervalMs;
  sendJsonLine(response);
}

void resetPublishState() {
  hasSentReading = false;
  lastSentLux = NAN;
  lastSentMs = 0;
}

bool applyConfigCommand(JsonDocument& command) {
  if (!command["refreshMs"].isNull()) {
    if (!command["refreshMs"].is<unsigned long>()) {
      sendCommandError("config.set", "invalid_refresh_ms");
      return false;
    }
    unsigned long value = command["refreshMs"].as<unsigned long>();
    if (value < 50UL || value > 60000UL) {
      sendCommandError("config.set", "invalid_refresh_ms");
      return false;
    }
    refreshMs = value;
  }

  if (!command["publishLuxChangePercent"].isNull()) {
    if (!command["publishLuxChangePercent"].is<float>()) {
      sendCommandError("config.set", "invalid_publish_lux_change_percent");
      return false;
    }
    float value = command["publishLuxChangePercent"].as<float>();
    if (isnan(value) || value < 0.0F || value > 100.0F) {
      sendCommandError("config.set", "invalid_publish_lux_change_percent");
      return false;
    }
    publishLuxChangePercent = value;
  }

  if (!command["publishMaxIntervalSeconds"].isNull()) {
    if (!command["publishMaxIntervalSeconds"].is<unsigned long>()) {
      sendCommandError("config.set", "invalid_publish_max_interval_seconds");
      return false;
    }
    unsigned long value = command["publishMaxIntervalSeconds"].as<unsigned long>();
    if (value < 1UL || value > 86400UL) {
      sendCommandError("config.set", "invalid_publish_max_interval_seconds");
      return false;
    }
    publishMaxIntervalSeconds = value;
  }

  if (!command["publishMode"].isNull()) {
    const char* value = command["publishMode"] | "";
    if (strcmp(value, "auto") == 0) {
      publishMode = PUBLISH_MODE_AUTO;
    } else if (strcmp(value, "interval") == 0) {
      publishMode = PUBLISH_MODE_INTERVAL;
    } else {
      sendCommandError("config.set", "invalid_publish_mode");
      return false;
    }
  }

  if (!command["integrationMs"].isNull()) {
    if (!command["integrationMs"].is<unsigned long>()) {
      sendCommandError("config.set", "invalid_integration_ms");
      return false;
    }
    unsigned long value = command["integrationMs"].as<unsigned long>();
    if (value > 600UL) {
      sendCommandError("config.set", "invalid_integration_ms");
      return false;
    }
    tsl2591IntegrationTime_t timing;
    if (!integrationTimeForMs((uint16_t)value, timing)) {
      sendCommandError("config.set", "invalid_integration_ms");
      return false;
    }
    integrationMs = (uint16_t)value;
    integrationTime = timing;
    tsl.setTiming(integrationTime);
  }

  if (!command["discardAfterGainChange"].isNull()) {
    if (!command["discardAfterGainChange"].is<bool>()) {
      sendCommandError("config.set", "invalid_discard_after_gain_change");
      return false;
    }
    discardAfterGainChange = command["discardAfterGainChange"].as<bool>();
  }

  if (!command["ledBrightness"].isNull()) {
    if (!command["ledBrightness"].is<unsigned int>()) {
      sendCommandError("config.set", "invalid_led_brightness");
      return false;
    }
    unsigned int value = command["ledBrightness"].as<unsigned int>();
    if (value > 255U) {
      sendCommandError("config.set", "invalid_led_brightness");
      return false;
    }
    ledBrightness = (uint8_t)value;
    setStatusLeds(false, false);
    logf(bleLog, LOG_NOTICE, "LED brightness changed value=%u", ledBrightness);
  }

  if (!command["ledBlinkIntervalMs"].isNull()) {
    if (!command["ledBlinkIntervalMs"].is<unsigned long>()) {
      sendCommandError("config.set", "invalid_led_blink_interval_ms");
      return false;
    }
    unsigned long value = command["ledBlinkIntervalMs"].as<unsigned long>();
    if (value < MIN_LED_BLINK_INTERVAL_MS ||
        value > MAX_LED_BLINK_INTERVAL_MS) {
      sendCommandError("config.set", "invalid_led_blink_interval_ms");
      return false;
    }
    ledBlinkIntervalMs = value;
    if (!powerOnIndicatorActive && !powerOffPending) {
      statusBlinkIntervalTimer.setDuration(ledBlinkIntervalMs);
      statusBlinkIntervalTimer.start();
    }
    logf(bleLog, LOG_NOTICE, "LED blink interval changed value=%lu ms",
         ledBlinkIntervalMs);
  }

  resetPublishState();
  logf(bleLog, LOG_NOTICE,
       "config applied refreshMs=%lu changePct=%.2f maxIntervalSec=%lu mode=%s "
       "integrationMs=%u discardAfterGainChange=%u ledBrightness=%u ledBlinkIntervalMs=%lu",
       refreshMs, publishLuxChangePercent, publishMaxIntervalSeconds,
       publishModeName(), integrationMs, discardAfterGainChange, ledBrightness,
       ledBlinkIntervalMs);
  sendConfigResponse("config.set");
  return true;
}

void resetRuntimeConfig() {
  refreshMs = DEFAULT_REFRESH_MS;
  publishLuxChangePercent = DEFAULT_PUBLISH_LUX_CHANGE_PERCENT;
  publishMaxIntervalSeconds = DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS;
  publishMode = DEFAULT_PUBLISH_MODE;
  setIntegrationMs(DEFAULT_INTEGRATION_MS);
  discardAfterGainChange = DEFAULT_DISCARD_AFTER_GAIN_CHANGE;
  ledBrightness = DEFAULT_LED_BRIGHTNESS;
  ledBlinkIntervalMs = DEFAULT_LED_BLINK_INTERVAL_MS;
  statusBlinkIntervalTimer.setDuration(ledBlinkIntervalMs);
  statusBlinkIntervalTimer.start();
  setStatusLeds(false, false);
  resetPublishState();
  bleLog.notice("runtime configuration reset to defaults");
}

bool handleJsonCommand(const String& line, unsigned long now) {
  logf(bleLog, LOG_NOTICE, "BLE JSON RX %s", line.c_str());
  JsonDocument command;
  DeserializationError error = deserializeJson(command, line);

  if (error) {
    sendCommandError("", "invalid_json");
    return false;
  }

  const char* cmd = command["cmd"] | "";
  if (strcmp(cmd, "get") == 0 || strcmp(cmd, "read") == 0 ||
      strcmp(cmd, "current") == 0) {
    if (!hasCachedReading) {
      sendCommandError(cmd, "no_cached_reading");
      return false;
    }
    publishReading(cachedReading, now);
    return true;
  }

  if (strcmp(cmd, "config.get") == 0) {
    sendConfigResponse("config.get");
    return false;
  }

  if (strcmp(cmd, "config.set") == 0) {
    applyConfigCommand(command);
    return false;
  }

  if (strcmp(cmd, "config.reset") == 0) {
    resetRuntimeConfig();
    sendConfigResponse("config.reset");
    return false;
  }

  if (strcmp(cmd, "ping") == 0) {
    bleLog.debug("BLE ping received");
    return false;
  }

  sendCommandError(cmd, "unknown_command");
  return false;
}

bool processJsonCommands(unsigned long now) {
  bool published = false;

  while (bleUart.available() > 0) {
    char c = (char)bleUart.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      bleRxLine.trim();
      if (bleRxLine.length() > 0) {
        lastBleCommandMs = now;
        bleCommandSeen = true;
        if (handleJsonCommand(bleRxLine, now)) {
          published = true;
        }
      }
      bleRxLine = "";
      continue;
    }

    if (bleRxLine.length() >= COMMAND_MAX_LENGTH) {
      bleRxLine = "";
      sendCommandError("", "command_too_long");
      continue;
    }

    bleRxLine += c;
  }

  return published;
}

void sleepForMs(unsigned long sleepMs) {
  wakeHandler.consumeAll();
  wakeHandler.scheduleRtcWakeIn(sleepMs);
  nrfWfi.sleepUntil(wakeHandler);
  wakeHandler.consumeAll();
}

void sleepUntilNextSensorRead(unsigned long now) {
  unsigned long elapsed = now - lastReadMs;
  unsigned long sleepMs = elapsed < refreshMs ? refreshMs - elapsed : 1UL;
  // A latched press must be confirmed promptly, even if it was already
  // released before the next regular sensor wake.
  if (buttonPressDebounceActive && sleepMs > BUTTON_DEBOUNCE_MS) {
    sleepMs = BUTTON_DEBOUNCE_MS;
  }
  sleepForMs(sleepMs);
}

void setup() {
  // TinyUSB supplies the USB CDC interface used for automatic uploads.
  Serial.begin(115200);
  bootLog.begin(&Serial);
  bleLog.begin(&Serial);
  sensorLog.begin(&Serial);

  uint32_t resetReason = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = resetReason;
  bootLog.notice("========== LuxSensor diagnostic boot ==========");
  logf(bootLog, LOG_NOTICE, "firmware=%s resetReason=0x%08lX",
       FIRMWARE_VERSION, (unsigned long)resetReason);
  bootLog.notice("USB serial ready baud=115200");
  turnOffUserLeds();
  configureExternalHardware();
  bootLog.notice("external GPIO configured leds=D1,D2 button=D10 voltage=A0 tslInt=D6");
  onboardFlash.begin();
  logf(bootLog, LOG_NOTICE, "QSPI JEDEC ID before deep power-down=0x%06lX",
       (unsigned long)onboardFlash.getJEDECID());
  Wire.begin();
  analogReadResolution(10);
  analogWriteResolution(8);
  bootLog.notice("I2C Wire initialized");
  scanI2cBus();

  nrfClock.begin();
  buttonDebounceTimer.setNowProvider(firmwareNowMs);
  buttonLongPressTimer.setNowProvider(firmwareNowMs);
  buttonDoublePressTimer.setNowProvider(firmwareNowMs);
  statusBlinkIntervalTimer.setNowProvider(firmwareNowMs);
  statusBlinkDurationTimer.setNowProvider(firmwareNowMs);
  powerOffIndicatorTimer.setNowProvider(firmwareNowMs);
  powerOnIndicatorTimer.setNowProvider(firmwareNowMs);
  wakeHandler.begin(nrfClock);
  wakeHandler.beginRtcWake(RTC_WAKE_MASK);
  nrfWfi.setSuspendSysTickDuringSleep(true);
  nrfWfi.setMaskNonWakeInterruptsDuringSleep(false);
  nrfWfi.begin();

  configureBle();
  bool flashAsleep = putOnboardFlashInDeepPowerDown();
  logf(bootLog, flashAsleep ? LOG_NOTICE : LOG_WARN,
       "QSPI deep power-down=%s", flashAsleep ? "confirmed" : "not_confirmed");
  onboardFlash.end();

  sensorLog.notice("TSL2591 begin start address=0x29");
  if (!tsl.begin()) {
    sensorLog.error("TSL2591 begin FAILED; BLE advertising remains available");
    while (true) {
      sd_app_evt_wait();
    }
  }

  sensorLog.notice("TSL2591 begin succeeded");
  configureSensor();
  tsl.enable();
  sensorLog.notice("TSL2591 enabled; startup settling reads start");
  for (uint8_t i = 0; i < STARTUP_SETTLING_READS; i++) {
    readSensorAutoRange();
  }
  resetLightState();
  lastReadMs = firmwareNowMs();
  beginStartupIndicator();
  bootLog.notice("setup complete; entering main loop");
}

void loop() {
  unsigned long now = firmwareNowMs();
  handleButton(now);
  updatePowerIndicators();
  bool bleConnected = Bluefruit.connected();
  serviceUsbState(bleConnected);

  if (bleConnectEventPending) {
    bleConnectEventPending = false;
    logf(bleLog, LOG_NOTICE, "connected handle=%u mtu=%u",
         pendingConnectionHandle,
         Bluefruit.Connection(pendingConnectionHandle) == nullptr
             ? 0
             : Bluefruit.Connection(pendingConnectionHandle)->getMtu());
  }
  if (bleDisconnectEventPending) {
    bleDisconnectEventPending = false;
    logf(bleLog, LOG_WARN, "disconnected handle=%u reason=0x%02X; advertising restarts",
         pendingConnectionHandle, pendingDisconnectReason);
  }
  updateStatusLed(now, bleConnected);
  logBleHealth(now);

  if (!devicePowered) {
    sleepForMs(250UL);
    return;
  }

  if (bleConnected && !bleWasConnected) {
    hasSentReading = false;
  }
  bleWasConnected = bleConnected;

  if (bleConnected && processJsonCommands(now)) {
    return;
  }

  if (bleConnected && bleConnectionHandle != BLE_CONN_HANDLE_INVALID &&
      bleCommandSeen && !lowPowerConnectionRequested &&
      now - lastBleCommandMs >= BLE_LOW_POWER_SETTLE_MS) {
    lowPowerConnectionRequested = true;
    BLEConnection *connection =
        Bluefruit.Connection(bleConnectionHandle);
    if (connection != nullptr) {
      bleLog.notice("requesting low-power BLE connection parameters");
      connection->requestConnectionParameter(
          LOW_POWER_CONNECTION_INTERVAL_UNITS,
          LOW_POWER_CONNECTION_SLAVE_LATENCY,
          CONNECTION_SUPERVISION_TIMEOUT_UNITS);
    }
  }

  if (bleConnected && bleConnectionHandle != BLE_CONN_HANDLE_INVALID &&
      now - lastBleCommandMs >= BLE_COMMAND_WATCHDOG_MS) {
    bleLog.warn("BLE command watchdog expired; disconnecting client");
    Bluefruit.disconnect(bleConnectionHandle);
    return;
  }

  if (now - lastReadMs < refreshMs) {
    sleepUntilNextSensorRead(now);
    return;
  }
  lastReadMs = now;

  LightReading reading = readSensorAutoRange();

  if (reading.gainSettled && !hasCachedReading) {
    return;
  }

  cacheReading(reading);

  if (!bleConnected) {
    return;
  }

  if (!shouldPublishReading(reading.lux, now)) {
    return;
  }

  publishReading(reading, now);
}
