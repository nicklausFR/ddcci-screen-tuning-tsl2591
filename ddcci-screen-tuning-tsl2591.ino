#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash onboardFlash(&flashTransport);
BLEUart bleUart;

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

// BLE Nordic UART Service JSON line commands:
// - {"cmd":"get"} or {"cmd":"current"} immediately republishes the latest
//   cached reading without triggering a new sensor measurement.
// - {"cmd":"config.get"} returns the runtime configuration.
// - {"cmd":"config.set","refreshMs":250,"publishLuxChangePercent":2.0,
//    "publishMaxIntervalSeconds":60,"publishMode":"auto","integrationMs":200,
//    "discardAfterGainChange":true} updates runtime settings.
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

const char *FIRMWARE_VERSION = "tsl2591-ble-nus-2026-07-29-5";
const char *BLE_DEVICE_NAME = "LuxSensor";
const uint8_t BLE_STATIC_ADDRESS[6] = {0x7E, 0x42, 0x91, 0x29, 0x5A, 0xCE};
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
};

static_assert(sizeof(BleMeasurementPacket) == 15,
              "Unexpected BLE measurement packet size");

LightReading cachedReading;
bool hasCachedReading = false;

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

void onBleConnected(uint16_t connectionHandle) {
  bleConnectionHandle = connectionHandle;
  lastBleCommandMs = millis();
  bleCommandSeen = false;
  lowPowerConnectionRequested = false;
  resetPublishState();
}

void onBleDisconnected(uint16_t connectionHandle, uint8_t reason) {
  (void)connectionHandle;
  (void)reason;
  bleConnectionHandle = BLE_CONN_HANDLE_INVALID;
  lastBleCommandMs = 0;
  bleCommandSeen = false;
  lowPowerConnectionRequested = false;
  bleRxLine = "";
  bleWasConnected = false;
}

void configureBle() {
  // Allow a complete JSON response to fit in one ATT notification when the
  // central negotiates the maximum MTU. This also increases the SoftDevice
  // notification queue from one entry to three; idle connection parameters
  // and radio duty cycle remain configured separately below.
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
}

void setGainIndex(uint8_t newGainIndex) {
  if (newGainIndex == gainIndex) {
    return;
  }

  gainIndex = newGainIndex;
  tsl.setGain(GAIN_STEPS[gainIndex]);
}

void discardGainSettlingReading() {
  if (!discardAfterGainChange) {
    return;
  }
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
}

LightReading readSensorOnce() {
  uint32_t luminosity = tsl.getFullLuminosity();
  uint16_t ir = luminosity >> 16;
  uint16_t full = luminosity & 0xFFFF;
  uint16_t visible = full > ir ? full - ir : 0;
  float lux = tsl.calculateLux(full, ir);
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
  bool valid = !hardSaturated && !isnan(lux) && lux >= 0;
  LightReading reading = {full, ir, visible, lux, adcOverRange, saturated,
                          valid, false, false, rawLevel, irRatio,
                          spectralOverload, false, 0};
  updateReadingQuality(reading);
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
      if (estimatedLux > reading.lux) {
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
  bool luxIsValid = !isnan(lux) && lux >= 0.0F;

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
  lastSentLuxWasValid = !isnan(lux) && lux >= 0.0F;
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

  return offset == length;
}

void sendJsonLine(JsonDocument& document) {
  String line;
  line.reserve(320);
  serializeJson(document, line);
  const uint8_t *data =
      reinterpret_cast<const uint8_t *>(line.c_str());
  if (writeBleReliably(data, line.length())) {
    const uint8_t newline = '\n';
    writeBleReliably(&newline, 1);
  }
}

void publishReading(const LightReading& reading, unsigned long now) {
  BleMeasurementPacket packet;
  packet.magic[0] = 'L';
  packet.magic[1] = 'T';
  packet.version = 1;
  packet.quality = reading.quality;
  packet.lux = (!isnan(reading.lux) && reading.lux >= 0.0F)
                   ? reading.lux
                   : NAN;
  packet.visible = reading.visible;
  packet.ir = reading.ir;
  packet.full = reading.full;
  packet.gain = gainIndex;
  if (writeBleReliably(reinterpret_cast<const uint8_t *>(&packet),
                       sizeof(packet))) {
    rememberSentLux(reading.lux, now);
  }
}

void cacheReading(const LightReading& reading) {
  cachedReading = reading;
  hasCachedReading = true;
}

void sendCommandError(const char* cmd, const char* error) {
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

  resetPublishState();
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
  resetPublishState();
}

bool handleJsonCommand(const String& line, unsigned long now) {
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

void setup() {
  turnOffUserLeds();
  onboardFlash.begin();
  Wire.begin();
  configureBle();
  putOnboardFlashInDeepPowerDown();
  onboardFlash.end();

  if (!tsl.begin()) {
    while (true) {
      sd_app_evt_wait();
    }
  }

  configureSensor();
  tsl.enable();
  for (uint8_t i = 0; i < STARTUP_SETTLING_READS; i++) {
    readSensorAutoRange();
  }
  resetLightState();
  lastReadMs = millis();
}

void loop() {
  unsigned long now = millis();
  bool bleConnected = Bluefruit.connected();

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
      connection->requestConnectionParameter(
          LOW_POWER_CONNECTION_INTERVAL_UNITS,
          LOW_POWER_CONNECTION_SLAVE_LATENCY,
          CONNECTION_SUPERVISION_TIMEOUT_UNITS);
    }
  }

  if (bleConnected && bleConnectionHandle != BLE_CONN_HANDLE_INVALID &&
      now - lastBleCommandMs >= BLE_COMMAND_WATCHDOG_MS) {
    Bluefruit.disconnect(bleConnectionHandle);
    return;
  }

  if (now - lastReadMs < refreshMs) {
    delay(10);
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
