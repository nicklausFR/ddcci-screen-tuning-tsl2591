#include <Arduino.h>
#include <NrfWfiHandler.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
#include <nrf.h>
#endif

// Relier momentanement D1 a GND pour verifier le reveil GPIO.
constexpr uint8_t WAKE_PIN = D1;

Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash onboardFlash(&flashTransport);
NrfWfiHandler nrfWfi;
volatile bool gpioWake = false;

void onGpioWake() {
  gpioWake = true;
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

void disableUsbPeripheral() {
#if defined(NRF_USBD)
  NRF_USBD->ENABLE = 0;
#endif
}

void putOnboardFlashInDeepPowerDown() {
  onboardFlash.begin();
  flashTransport.begin();
  flashTransport.runCommand(0xB9);
  delay(10);
  onboardFlash.end();
}

void setup() {
  turnOffUserLeds();
  putOnboardFlashInDeepPowerDown();
  disableUsbPeripheral();

  pinMode(WAKE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WAKE_PIN), onGpioWake, CHANGE);

  nrfWfi.setSuspendSysTickDuringSleep(true);
  nrfWfi.setMaskNonWakeInterruptsDuringSleep(false);
  nrfWfi.begin();
}

void loop() {
  gpioWake = false;
  while (!gpioWake) {
    nrfWfi.sleepOnce();
  }
}
