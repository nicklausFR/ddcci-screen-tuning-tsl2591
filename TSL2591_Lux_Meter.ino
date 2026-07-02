#include <Wire.h>
#include <Adafruit_TinyUSB.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include "Adafruit_TSL2591.h"

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// User-tunable defaults.
// refreshMs controls the internal sensor polling cadence. Keep it low enough
// for autorange and protection logic to react quickly.
const unsigned long DEFAULT_REFRESH_MS = 100;

// In auto mode, publish when lux changes by at least this percentage.
const float DEFAULT_PUBLISH_LUX_CHANGE_PERCENT = 1.0F;

// In auto mode this is the maximum time between publishes. In interval mode
// this is the fixed publish interval.
const unsigned long DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS = 30;

// Serial JSON line commands:
// - {"cmd":"get"} or {"cmd":"current"} immediately publishes the current reading.
// - {"cmd":"config.get"} returns the runtime configuration.
// - {"cmd":"config.set","refreshMs":250,"publishLuxChangePercent":2.0,
//    "publishMaxIntervalSeconds":60,"publishMode":"auto"} updates runtime settings.
// publishMode accepts "auto" or "interval". Runtime settings are not persisted
// across reset.

// Internal autorange, saturation, and spectral compensation constants. Change
// these only when recalibrating sensor behavior.
const uint16_t SATURATION_COUNT = 65535;
const uint16_t HIGH_COUNT = 60000;
const uint16_t HOLD_COUNT = 45000;
const uint16_t SPECTRAL_OVERLOAD_COUNT = 12000;
const uint16_t SPECTRAL_RELEASE_COUNT = 10000;
const float RAW_RISING_HOLD_LEVEL = 350.0F;
const float SPECTRAL_OVERLOAD_IR_RATIO = 0.28F;
const float SPECTRAL_RELEASE_IR_RATIO = 0.24F;
const float TRUSTED_IR_RATIO = 0.27F;
const uint16_t LOW_COUNT = 1000;
const uint16_t SERIAL_COMMAND_MAX_LENGTH = 256;
unsigned long lastReadMs = 0;

const char *FIRMWARE_VERSION = "tsl2591-autorange-2026-07-02-12";

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
bool serialWasConnected = false;
String serialRxLine = "";
unsigned long refreshMs = DEFAULT_REFRESH_MS;
float publishLuxChangePercent = DEFAULT_PUBLISH_LUX_CHANGE_PERCENT;
unsigned long publishMaxIntervalSeconds = DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS;
PublishMode publishMode = DEFAULT_PUBLISH_MODE;

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
};

void configureSensor() {
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  tsl.setGain(GAIN_STEPS[gainIndex]);
}

void setGainIndex(uint8_t newGainIndex) {
  if (newGainIndex == gainIndex) {
    return;
  }

  gainIndex = newGainIndex;
  tsl.setGain(GAIN_STEPS[gainIndex]);
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
  bool spectralEnter = full >= SPECTRAL_OVERLOAD_COUNT &&
                       irRatio >= SPECTRAL_OVERLOAD_IR_RATIO;
  bool spectralRelease = full < SPECTRAL_RELEASE_COUNT ||
                         irRatio < SPECTRAL_RELEASE_IR_RATIO;

  if (spectralEnter) {
    spectralHoldLatched = true;
  } else if (spectralRelease) {
    spectralHoldLatched = false;
  }

  bool spectralOverload = spectralEnter || spectralHoldLatched;
  bool saturated = adcOverRange || spectralOverload;
  bool valid = !hardSaturated && !isnan(lux) && lux >= 0;
  uint16_t peak = full > ir ? full : ir;
  float rawLevel = (float)peak / GAIN_FACTORS[gainIndex];

  return {full, ir, visible, lux, adcOverRange, saturated, valid, false, false,
          rawLevel, irRatio, spectralOverload};
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
      continue;
    }

    if (reading.valid && peak <= LOW_COUNT && gainIndex + 1 < GAIN_STEP_COUNT) {
      setGainIndex(gainIndex + 1);
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

    bool highRange = overRangeDuringMeasurement || reading.saturated ||
                     reading.spectralOverload || peak >= HOLD_COUNT;
    bool rawLevelIsRising = !isnan(lastValidRawLevel) &&
                            reading.rawLevel >= RAW_RISING_HOLD_LEVEL &&
                            reading.rawLevel >= lastValidRawLevel * 1.02F;
    bool publishedDrop = !isnan(lastPublishedLux) && reading.lux < lastPublishedLux;
    bool incoherentDrop = publishedDrop &&
                          (highRange || rawLevelIsRising);

    if (incoherentDrop) {
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

void publishReading(const LightReading& reading, unsigned long now) {
  Serial.print("{\"lux\":");
  if (isnan(reading.lux) || reading.lux < 0) {
    Serial.print("null");
  } else {
    Serial.print(reading.lux, 3);
  }
  Serial.print(",\"visible\":");
  Serial.print(reading.visible);
  Serial.print(",\"ir\":");
  Serial.print(reading.ir);
  Serial.print(",\"full\":");
  Serial.print(reading.full);
  Serial.print(",\"gain\":\"");
  Serial.print(GAIN_NAMES[gainIndex]);
  Serial.print("\"");
  Serial.print(",\"saturated\":");
  Serial.print(reading.saturated ? "true" : "false");
  Serial.print(",\"adcOverRange\":");
  Serial.print(reading.adcOverRange ? "true" : "false");
  Serial.print(",\"spectral\":");
  Serial.print(reading.spectralOverload ? "true" : "false");
  Serial.print(",\"held\":");
  Serial.print(reading.held ? "true" : "false");
  Serial.print(",\"estimated\":");
  Serial.print(reading.estimated ? "true" : "false");
  Serial.print(",\"raw\":");
  Serial.print(reading.rawLevel, 3);
  Serial.print(",\"irRatio\":");
  Serial.print(reading.irRatio, 3);
  Serial.print(",\"luxPerRaw\":");
  if (isnan(luxPerRawLevel)) {
    Serial.print("null");
  } else {
    Serial.print(luxPerRawLevel, 3);
  }
  Serial.print(",\"fw\":\"");
  Serial.print(FIRMWARE_VERSION);
  Serial.print("\"");
  Serial.println("}");

  rememberSentLux(reading.lux, now);
}

void sendCommandError(const char* cmd, const char* error) {
  JsonDocument response;
  response["type"] = "response";
  response["ok"] = false;
  if (cmd != nullptr && cmd[0] != '\0') {
    response["cmd"] = cmd;
  }
  response["error"] = error;
  serializeJson(response, Serial);
  Serial.println();
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
  serializeJson(response, Serial);
  Serial.println();
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

  resetPublishState();
  sendConfigResponse("config.set");
  return true;
}

void resetRuntimeConfig() {
  refreshMs = DEFAULT_REFRESH_MS;
  publishLuxChangePercent = DEFAULT_PUBLISH_LUX_CHANGE_PERCENT;
  publishMaxIntervalSeconds = DEFAULT_PUBLISH_MAX_INTERVAL_SECONDS;
  publishMode = DEFAULT_PUBLISH_MODE;
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
    LightReading reading = readSensorAutoRange();
    lastReadMs = now;
    publishReading(reading, now);
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

  sendCommandError(cmd, "unknown_command");
  return false;
}

bool processJsonCommands(unsigned long now) {
  bool published = false;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      serialRxLine.trim();
      if (serialRxLine.length() > 0 &&
          handleJsonCommand(serialRxLine, now)) {
        published = true;
      }
      serialRxLine = "";
      continue;
    }

    if (serialRxLine.length() >= SERIAL_COMMAND_MAX_LENGTH) {
      serialRxLine = "";
      sendCommandError("", "command_too_long");
      continue;
    }

    serialRxLine += c;
  }

  return published;
}

void setup() {
  Serial.begin(115200);
  unsigned long startMs = millis();
  while (!Serial && millis() - startMs < 3000) {
    delay(10);
  }

  Wire.begin();

  if (!tsl.begin()) {
    Serial.println("{\"error\":\"tsl2591_not_found\"}");
    while (true) {
      delay(1000);
    }
  }

  configureSensor();
  tsl.enable();
}

void loop() {
  unsigned long now = millis();
  bool serialConnected = Serial;

  if (serialConnected && !serialWasConnected) {
    hasSentReading = false;
  }
  serialWasConnected = serialConnected;

  if (serialConnected && processJsonCommands(now)) {
    return;
  }

  if (now - lastReadMs < refreshMs) {
    delay(10);
    return;
  }
  lastReadMs = now;

  LightReading reading = readSensorAutoRange();

  if (!serialConnected) {
    return;
  }

  if (!shouldPublishReading(reading.lux, now)) {
    return;
  }

  publishReading(reading, now);
}
