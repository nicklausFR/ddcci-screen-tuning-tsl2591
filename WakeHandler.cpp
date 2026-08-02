#include <WakeHandler.h>

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
#include <nrf.h>
#endif

WakeHandler* WakeHandler::activeRtcHandler = nullptr;

WakeHandler::WakeHandler()
  : _pending(0),
    _rtcWakeSourceMask(source(31)),
    _clock(nullptr),
    _scheduledRtcWakeMs(0),
    _rtcLastCounter(0),
    _rtcClockSyncReady(false)
{
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    _schedules[i].sourceMask = 0;
    _schedules[i].dueAtMs = 0;
    _schedules[i].intervalMs = 0;
    _schedules[i].enabled = false;
    _schedules[i].repeating = false;
  }
}

void WakeHandler::begin(NrfClock& clock)
{
  setClock(&clock);
}

void WakeHandler::setClock(NrfClock* clock)
{
  _clock = clock;
}

uint64_t WakeHandler::nowMs() const
{
  return _clock != nullptr ? _clock->monotonicMs() : (uint64_t)millis();
}

WakeMask WakeHandler::source(uint8_t index)
{
  if (index >= 32) {
    return 0;
  }

  return (WakeMask)1UL << index;
}

void WakeHandler::notify(WakeMask sourceMask)
{
  uint8_t state = saveInterruptState();
  _pending |= sourceMask;
  restoreInterruptState(state);
}

bool WakeHandler::hasWake(WakeMask sourceMask) const
{
  uint8_t state = saveInterruptState();
  bool result = (_pending & sourceMask) != 0;
  restoreInterruptState(state);

  return result;
}

bool WakeHandler::hasAnyWake() const
{
  uint8_t state = saveInterruptState();
  bool result = _pending != 0;
  restoreInterruptState(state);

  return result;
}

WakeMask WakeHandler::pending() const
{
  uint8_t state = saveInterruptState();
  WakeMask result = _pending;
  restoreInterruptState(state);

  return result;
}

bool WakeHandler::consume(WakeMask sourceMask)
{
  uint8_t state = saveInterruptState();
  bool result = (_pending & sourceMask) != 0;
  _pending &= ~sourceMask;
  restoreInterruptState(state);

  return result;
}

WakeMask WakeHandler::consumeAll()
{
  uint8_t state = saveInterruptState();
  WakeMask result = _pending;
  _pending = 0;
  restoreInterruptState(state);

  return result;
}

void WakeHandler::clear(WakeMask sourceMask)
{
  uint8_t state = saveInterruptState();
  _pending &= ~sourceMask;
  restoreInterruptState(state);
}

void WakeHandler::clearAll()
{
  uint8_t state = saveInterruptState();
  _pending = 0;
  restoreInterruptState(state);
}

bool WakeHandler::hasAnyWakeFromCritical() const
{
  return _pending != 0;
}

bool WakeHandler::scheduleInterval(
  uint8_t slot,
  WakeMask sourceMask,
  unsigned long intervalMs,
  bool dueNow
)
{
  if (slot >= MAX_SCHEDULES || sourceMask == 0 || intervalMs == 0) {
    return false;
  }

  _schedules[slot].sourceMask = sourceMask;
  _schedules[slot].intervalMs = intervalMs;
  _schedules[slot].dueAtMs = nowMs() + (dueNow ? 0 : intervalMs);
  _schedules[slot].enabled = true;
  _schedules[slot].repeating = true;
  return true;
}

bool WakeHandler::scheduleOnceAt(uint8_t slot, WakeMask sourceMask, uint64_t dueAtMs)
{
  if (slot >= MAX_SCHEDULES || sourceMask == 0) {
    return false;
  }

  _schedules[slot].sourceMask = sourceMask;
  _schedules[slot].intervalMs = 0;
  _schedules[slot].dueAtMs = dueAtMs;
  _schedules[slot].enabled = true;
  _schedules[slot].repeating = false;
  return true;
}

void WakeHandler::disableSchedule(uint8_t slot)
{
  if (slot >= MAX_SCHEDULES) {
    return;
  }

  _schedules[slot].enabled = false;
}

WakeMask WakeHandler::updateSchedules()
{
  return updateSchedules(nowMs());
}

WakeMask WakeHandler::updateSchedules(uint64_t nowMs)
{
  WakeMask dueMask = 0;

  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    WakeSchedule& schedule = _schedules[i];

    if (!schedule.enabled || !timeReached(nowMs, schedule.dueAtMs)) {
      continue;
    }

    dueMask |= schedule.sourceMask;

    if (schedule.repeating) {
      uint64_t minNextDelayMs = schedule.intervalMs / 2UL;
      uint64_t minNextDueAt = nowMs + minNextDelayMs;
      do {
        schedule.dueAtMs += schedule.intervalMs;
      } while (timeReached(minNextDueAt, schedule.dueAtMs));
    } else {
      schedule.enabled = false;
    }
  }

  return dueMask;
}

bool WakeHandler::hasSchedule() const
{
  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    if (_schedules[i].enabled) {
      return true;
    }
  }

  return false;
}

unsigned long WakeHandler::nextScheduleDelayMs() const
{
  return nextScheduleDelayMs(nowMs());
}

unsigned long WakeHandler::nextScheduleDelayMs(uint64_t nowMs) const
{
  bool found = false;
  uint64_t nextDueAt = 0;

  for (uint8_t i = 0; i < MAX_SCHEDULES; i++) {
    const WakeSchedule& schedule = _schedules[i];

    if (!schedule.enabled) {
      continue;
    }

    if (!found || schedule.dueAtMs < nextDueAt) {
      nextDueAt = schedule.dueAtMs;
      found = true;
    }
  }

  if (!found || timeReached(nowMs, nextDueAt)) {
    return 1;
  }

  uint64_t delayMs = nextDueAt - nowMs;
  return delayMs > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (unsigned long)delayMs;
}

void WakeHandler::beginRtcWake(WakeMask rtcWakeSourceMask)
{
  if (rtcWakeSourceMask != 0) {
    _rtcWakeSourceMask = rtcWakeSourceMask;
  }

  activeRtcHandler = this;

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;

  uint32_t waitCount = 0;
  while (!NRF_CLOCK->EVENTS_LFCLKSTARTED && waitCount < 1000000UL) {
    waitCount++;
  }

  NRF_RTC2->TASKS_STOP = 1;
  NRF_RTC2->TASKS_CLEAR = 1;
  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  NRF_RTC2->INTENCLR = RTC_INTENCLR_COMPARE0_Msk;

  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_SetPriority(RTC2_IRQn, 6);
  NVIC_EnableIRQ(RTC2_IRQn);

  NRF_RTC2->TASKS_START = 1;
  _rtcLastCounter = NRF_RTC2->COUNTER;
  _rtcClockSyncReady = true;
#endif
}

void WakeHandler::scheduleRtcWakeIn(unsigned long delayMs)
{
  _scheduledRtcWakeMs = delayMs == 0 ? 1 : delayMs;

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  const uint32_t maxTicks = 0x00FFFFFEUL;
  uint64_t ticks64 = ((uint64_t)(delayMs == 0 ? 1 : delayMs) * 32768ULL + 999ULL) / 1000ULL;

  if (ticks64 == 0) {
    ticks64 = 1;
  }

  if (ticks64 > maxTicks) {
    ticks64 = maxTicks;
  }

  NRF_RTC2->EVENTS_COMPARE[0] = 0;
  _rtcLastCounter = NRF_RTC2->COUNTER & 0x00FFFFFFUL;
  _rtcClockSyncReady = true;
  NRF_RTC2->CC[0] = (_rtcLastCounter + (uint32_t)ticks64) & 0x00FFFFFFUL;
  NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
#else
  (void)delayMs;
#endif
}

void WakeHandler::scheduleRtcWakeForNextSchedule()
{
  scheduleRtcWakeIn(nextScheduleDelayMs());
}

void WakeHandler::syncClockFromRtc()
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  if (_clock == nullptr || !_rtcClockSyncReady) {
    return;
  }

  uint32_t current = NRF_RTC2->COUNTER & 0x00FFFFFFUL;
  uint32_t elapsedTicks = (current - _rtcLastCounter) & 0x00FFFFFFUL;
  if (elapsedTicks == 0) {
    return;
  }

  uint64_t elapsedMs = ((uint64_t)elapsedTicks * 1000ULL + 16384ULL) / 32768ULL;
  if (elapsedMs > 0) {
    _clock->advanceSleepMs(elapsedMs > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (uint32_t)elapsedMs);
    _rtcLastCounter = current;
  }
#endif
}

bool WakeHandler::timeReached(uint64_t nowMs, uint64_t dueAtMs)
{
  return nowMs >= dueAtMs;
}

void WakeHandler::handleRtc2Interrupt()
{
#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
  if (NRF_RTC2->EVENTS_COMPARE[0]) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->INTENCLR = RTC_INTENCLR_COMPARE0_Msk;

    if (activeRtcHandler != nullptr) {
      activeRtcHandler->notify(activeRtcHandler->_rtcWakeSourceMask);
    }
  }
#endif
}

#if defined(NRF52_SERIES) || defined(ARDUINO_ARCH_NRF52)
extern "C" void __attribute__((weak)) RTC2_IRQHandler(void)
{
  WakeHandler::handleRtc2Interrupt();
}
#endif

uint8_t WakeHandler::saveInterruptState()
{
#if defined(__arm__) || defined(__ARM_ARCH)
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return (uint8_t)(primask & 0x01);
#elif defined(ARDUINO_ARCH_AVR)
  uint8_t sreg = SREG;
  noInterrupts();
  return sreg;
#else
  noInterrupts();
  return 0;
#endif
}

void WakeHandler::restoreInterruptState(uint8_t state)
{
#if defined(__arm__) || defined(__ARM_ARCH)
  __set_PRIMASK(state);
#elif defined(ARDUINO_ARCH_AVR)
  SREG = state;
#else
  interrupts();
#endif
}
