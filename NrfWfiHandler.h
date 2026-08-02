#ifndef NRF_WFI_HANDLER_H
#define NRF_WFI_HANDLER_H

#include <Arduino.h>
#include <WakeHandler.h>

enum NrfWakeSense {
  NRF_WAKE_SENSE_DISABLED,
  NRF_WAKE_SENSE_HIGH,
  NRF_WAKE_SENSE_LOW
};

class NrfWfiHandler {
public:
  NrfWfiHandler();

  void begin();

  void configureAwakeLed(
    uint8_t pin,
    bool enabled,
    uint8_t onState,
    uint8_t offState
  );

  void setAwakeLedEnabled(bool enabled);
  bool awakeLedEnabled() const;
  void setSuspendSysTickDuringSleep(bool enabled);
  bool suspendSysTickDuringSleep() const;
  void setMaskNonWakeInterruptsDuringSleep(bool enabled);
  bool maskNonWakeInterruptsDuringSleep() const;
  bool keepInterruptDuringSleep(IRQn_Type irq);
  void clearKeptInterrupts();
  void awakeLedOn();
  void awakeLedOff();

  bool configureWakePin(uint8_t pin, NrfWakeSense sense);
  void disableWakePin(uint8_t pin);
  void clearWakePin(uint8_t pin);
  void clearWakeFlags();

  void sleepOnce();
  WakeMask sleepUntil(WakeHandler& wakeHandler);

private:
  static const uint8_t MAX_WAKE_PINS = 8;
  static const uint8_t MAX_KEPT_INTERRUPTS = 8;
  static const uint8_t INVALID_PIN = 255;

  uint8_t _wakePins[MAX_WAKE_PINS];
  NrfWakeSense _wakeSenses[MAX_WAKE_PINS];

  uint8_t _awakeLedPin;
  bool _awakeLedConfigured;
  bool _awakeLedEnabled;
  bool _suspendSysTickDuringSleep;
  bool _maskNonWakeInterruptsDuringSleep;
  uint8_t _awakeLedOnState;
  uint8_t _awakeLedOffState;
  IRQn_Type _keptInterrupts[MAX_KEPT_INTERRUPTS];
  uint8_t _keptInterruptCount;

  int8_t findWakePin(uint8_t pin) const;
  int8_t findFreeWakePin() const;
  void rememberWakePin(uint8_t pin, NrfWakeSense sense);
  bool hasLatchedWakePin() const;

  static uint8_t saveInterruptState();
  static void restoreInterruptState(uint8_t state);
  static uint32_t suspendSysTick();
  static void restoreSysTick(uint32_t ctrl);

  static const uint8_t MAX_NVIC_STATE_REGISTERS = 4;
  uint8_t maskNonWakeInterrupts(uint32_t* enabledState, uint8_t maxCount) const;
  static void restoreNonWakeInterrupts(const uint32_t* enabledState, uint8_t count);
};

#endif
