#include <NrfWfiHandler.h>

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
#include <nrf.h>
#endif

NrfWfiHandler::NrfWfiHandler()
  : _awakeLedPin(INVALID_PIN),
    _awakeLedConfigured(false),
    _awakeLedEnabled(false),
    _suspendSysTickDuringSleep(false),
    _maskNonWakeInterruptsDuringSleep(false),
    _awakeLedOnState(HIGH),
    _awakeLedOffState(LOW),
    _keptInterruptCount(0)
{
  for (uint8_t i = 0; i < MAX_WAKE_PINS; i++) {
    _wakePins[i] = INVALID_PIN;
    _wakeSenses[i] = NRF_WAKE_SENSE_DISABLED;
  }

  for (uint8_t i = 0; i < MAX_KEPT_INTERRUPTS; i++) {
    _keptInterrupts[i] = NonMaskableInt_IRQn;
  }

#if defined(GPIOTE_IRQn)
  keepInterruptDuringSleep(GPIOTE_IRQn);
#endif
}

void NrfWfiHandler::begin()
{
  if (_awakeLedConfigured) {
    pinMode(_awakeLedPin, OUTPUT);
    awakeLedOn();
  }
}

void NrfWfiHandler::configureAwakeLed(
  uint8_t pin,
  bool enabled,
  uint8_t onState,
  uint8_t offState
)
{
  _awakeLedPin = pin;
  _awakeLedEnabled = enabled;
  _awakeLedOnState = onState;
  _awakeLedOffState = offState;
  _awakeLedConfigured = true;

  pinMode(_awakeLedPin, OUTPUT);
  awakeLedOn();
}

void NrfWfiHandler::setAwakeLedEnabled(bool enabled)
{
  _awakeLedEnabled = enabled;

  if (_awakeLedConfigured) {
    if (_awakeLedEnabled) {
      awakeLedOn();
    } else {
      digitalWrite(_awakeLedPin, _awakeLedOffState);
    }
  }
}

bool NrfWfiHandler::awakeLedEnabled() const
{
  return _awakeLedEnabled;
}

void NrfWfiHandler::setSuspendSysTickDuringSleep(bool enabled)
{
  _suspendSysTickDuringSleep = enabled;
}

bool NrfWfiHandler::suspendSysTickDuringSleep() const
{
  return _suspendSysTickDuringSleep;
}

void NrfWfiHandler::setMaskNonWakeInterruptsDuringSleep(bool enabled)
{
  _maskNonWakeInterruptsDuringSleep = enabled;
}

bool NrfWfiHandler::maskNonWakeInterruptsDuringSleep() const
{
  return _maskNonWakeInterruptsDuringSleep;
}

bool NrfWfiHandler::keepInterruptDuringSleep(IRQn_Type irq)
{
  if ((int32_t)irq < 0) {
    return false;
  }

  for (uint8_t i = 0; i < _keptInterruptCount; i++) {
    if (_keptInterrupts[i] == irq) {
      return true;
    }
  }

  if (_keptInterruptCount >= MAX_KEPT_INTERRUPTS) {
    return false;
  }

  _keptInterrupts[_keptInterruptCount] = irq;
  _keptInterruptCount++;
  return true;
}

void NrfWfiHandler::clearKeptInterrupts()
{
  _keptInterruptCount = 0;
}

void NrfWfiHandler::awakeLedOn()
{
  if (_awakeLedConfigured && _awakeLedEnabled) {
    digitalWrite(_awakeLedPin, _awakeLedOnState);
  }
}

void NrfWfiHandler::awakeLedOff()
{
  if (_awakeLedConfigured && _awakeLedEnabled) {
    digitalWrite(_awakeLedPin, _awakeLedOffState);
  }
}

bool NrfWfiHandler::configureWakePin(uint8_t pin, NrfWakeSense sense)
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  if (pin >= 64) {
    return false;
  }

  pinMode(pin, INPUT);

  NRF_GPIO_Type* port = NRF_P0;
  uint8_t localPin = pin < 32 ? pin : pin - 32;

#if defined(NRF_P1)
  if (pin >= 32) {
    port = NRF_P1;
  }
#else
  if (pin >= 32) {
    return false;
  }
#endif

  port->PIN_CNF[localPin] &= ~(GPIO_PIN_CNF_SENSE_Msk);

  if (sense == NRF_WAKE_SENSE_HIGH) {
    port->PIN_CNF[localPin] |=
      (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  }
  else if (sense == NRF_WAKE_SENSE_LOW) {
    port->PIN_CNF[localPin] |=
      (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
  }

  if (sense != NRF_WAKE_SENSE_DISABLED) {
    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
  }

  rememberWakePin(pin, sense);
  return true;
#else
  (void)pin;
  (void)sense;
  return false;
#endif
}

void NrfWfiHandler::disableWakePin(uint8_t pin)
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  if (pin >= 64) {
    return;
  }

  NRF_GPIO_Type* port = NRF_P0;
  uint8_t localPin = pin < 32 ? pin : pin - 32;

#if defined(NRF_P1)
  if (pin >= 32) {
    port = NRF_P1;
  }
#else
  if (pin >= 32) {
    return;
  }
#endif

  port->PIN_CNF[localPin] &= ~(GPIO_PIN_CNF_SENSE_Msk);

  int8_t index = findWakePin(pin);
  if (index >= 0) {
    _wakePins[index] = INVALID_PIN;
    _wakeSenses[index] = NRF_WAKE_SENSE_DISABLED;
  }
#else
  (void)pin;
#endif
}

void NrfWfiHandler::clearWakePin(uint8_t pin)
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  if (pin >= 64) {
    return;
  }

  NRF_GPIO_Type* port = NRF_P0;
  uint8_t localPin = pin < 32 ? pin : pin - 32;

#if defined(NRF_P1)
  if (pin >= 32) {
    port = NRF_P1;
  }
#else
  if (pin >= 32) {
    return;
  }
#endif

  port->LATCH = (1UL << localPin);
#else
  (void)pin;
#endif
}

void NrfWfiHandler::clearWakeFlags()
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  NRF_GPIOTE->EVENTS_PORT = 0;

  for (uint8_t i = 0; i < MAX_WAKE_PINS; i++) {
    if (_wakePins[i] != INVALID_PIN) {
      clearWakePin(_wakePins[i]);
    }
  }
#endif
}

void NrfWfiHandler::sleepOnce()
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  uint32_t systickCtrl = 0;
  uint32_t nvicState[MAX_NVIC_STATE_REGISTERS];
  uint8_t nvicStateCount = 0;

  awakeLedOff();
  if (_suspendSysTickDuringSleep) {
    systickCtrl = suspendSysTick();
  }
  if (_maskNonWakeInterruptsDuringSleep) {
    nvicStateCount = maskNonWakeInterrupts(nvicState, MAX_NVIC_STATE_REGISTERS);
  }

  __DSB();
  __WFI();

  if (_maskNonWakeInterruptsDuringSleep) {
    restoreNonWakeInterrupts(nvicState, nvicStateCount);
  }
  if (_suspendSysTickDuringSleep) {
    restoreSysTick(systickCtrl);
  }

  awakeLedOn();
  __ISB();
#endif
}

WakeMask NrfWfiHandler::sleepUntil(WakeHandler& wakeHandler)
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  while (true) {
    uint8_t state = saveInterruptState();

    if (wakeHandler.hasAnyWakeFromCritical()) {
      restoreInterruptState(state);
      break;
    }

    uint32_t systickCtrl = 0;
    uint32_t nvicState[MAX_NVIC_STATE_REGISTERS];
    uint8_t nvicStateCount = 0;

    awakeLedOff();
    if (_suspendSysTickDuringSleep) {
      systickCtrl = suspendSysTick();
    }
    if (_maskNonWakeInterruptsDuringSleep) {
      nvicStateCount = maskNonWakeInterrupts(nvicState, MAX_NVIC_STATE_REGISTERS);
    }

    clearWakeFlags();
    restoreInterruptState(state);
    if (wakeHandler.hasAnyWake()) {
      if (_maskNonWakeInterruptsDuringSleep) {
        restoreNonWakeInterrupts(nvicState, nvicStateCount);
      }
      if (_suspendSysTickDuringSleep) {
        restoreSysTick(systickCtrl);
      }
      awakeLedOn();
      __ISB();
      break;
    }

    __DSB();
    __WFI();

    bool gpioWake = hasLatchedWakePin();
    state = saveInterruptState();
    if (_maskNonWakeInterruptsDuringSleep) {
      restoreNonWakeInterrupts(nvicState, nvicStateCount);
    }
    if (_suspendSysTickDuringSleep) {
      restoreSysTick(systickCtrl);
    }

    awakeLedOn();
    __ISB();

    restoreInterruptState(state);
    wakeHandler.syncClockFromRtc();
    if (gpioWake) {
      break;
    }
  }
#else
  while (!wakeHandler.hasAnyWake()) {
    delay(1);
  }
#endif

  return wakeHandler.pending();
}

int8_t NrfWfiHandler::findWakePin(uint8_t pin) const
{
  for (uint8_t i = 0; i < MAX_WAKE_PINS; i++) {
    if (_wakePins[i] == pin) {
      return (int8_t)i;
    }
  }

  return -1;
}

int8_t NrfWfiHandler::findFreeWakePin() const
{
  for (uint8_t i = 0; i < MAX_WAKE_PINS; i++) {
    if (_wakePins[i] == INVALID_PIN) {
      return (int8_t)i;
    }
  }

  return -1;
}

void NrfWfiHandler::rememberWakePin(uint8_t pin, NrfWakeSense sense)
{
  int8_t index = findWakePin(pin);

  if (index < 0) {
    index = findFreeWakePin();
  }

  if (index >= 0) {
    _wakePins[index] = pin;
    _wakeSenses[index] = sense;
  }
}

bool NrfWfiHandler::hasLatchedWakePin() const
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  for (uint8_t i = 0; i < MAX_WAKE_PINS; i++) {
    if (_wakePins[i] == INVALID_PIN || _wakeSenses[i] == NRF_WAKE_SENSE_DISABLED) {
      continue;
    }

    uint8_t pin = _wakePins[i];
    if (pin >= 64) {
      continue;
    }

    NRF_GPIO_Type* port = NRF_P0;
    uint8_t localPin = pin < 32 ? pin : pin - 32;

#if defined(NRF_P1)
    if (pin >= 32) {
      port = NRF_P1;
    }
#else
    if (pin >= 32) {
      continue;
    }
#endif

    if (port->LATCH & (1UL << localPin)) {
      return true;
    }
  }
#endif

  return false;
}

uint8_t NrfWfiHandler::saveInterruptState()
{
#if defined(__arm__) || defined(__ARM_ARCH)
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return (uint8_t)(primask & 0x01);
#else
  noInterrupts();
  return 0;
#endif
}

void NrfWfiHandler::restoreInterruptState(uint8_t state)
{
#if defined(__arm__) || defined(__ARM_ARCH)
  __set_PRIMASK(state);
#else
  interrupts();
#endif
}

uint32_t NrfWfiHandler::suspendSysTick()
{
#if defined(SysTick_CTRL_ENABLE_Msk)
  uint32_t ctrl = SysTick->CTRL;
  SysTick->CTRL = ctrl & ~SysTick_CTRL_ENABLE_Msk;
  return ctrl;
#else
  return 0;
#endif
}

void NrfWfiHandler::restoreSysTick(uint32_t ctrl)
{
#if defined(SysTick_CTRL_ENABLE_Msk)
  SysTick->CTRL = ctrl;
#else
  (void)ctrl;
#endif
}

uint8_t NrfWfiHandler::maskNonWakeInterrupts(uint32_t* enabledState, uint8_t maxCount) const
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  uint8_t count = 1;
  if (count > maxCount) {
    count = maxCount;
  }

  for (uint8_t i = 0; i < count; i++) {
    enabledState[i] = NVIC->ISER[i];
  }

  for (uint8_t i = 0; i < count; i++) {
    uint32_t keepEnabled = 0;

    for (uint8_t kept = 0; kept < _keptInterruptCount; kept++) {
      uint32_t irq = (uint32_t)_keptInterrupts[kept];

      if (irq / 32UL == i) {
        keepEnabled |= (1UL << (irq % 32UL));
      }
    }

    NVIC->ICER[i] = enabledState[i] & ~keepEnabled;
  }

  return count;
#else
  (void)enabledState;
  (void)maxCount;
  return 0;
#endif
}

void NrfWfiHandler::restoreNonWakeInterrupts(const uint32_t* enabledState, uint8_t count)
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  for (uint8_t i = 0; i < count; i++) {
    NVIC->ISER[i] = enabledState[i];
  }
#else
  (void)enabledState;
  (void)count;
#endif
}
