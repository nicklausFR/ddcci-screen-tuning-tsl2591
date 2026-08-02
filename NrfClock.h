#ifndef NRF_CLOCK_H
#define NRF_CLOCK_H

#include <Arduino.h>

// Monotonic clock that preserves elapsed time while SysTick is suspended.
class NrfClock {
public:
  void begin();
  uint64_t monotonicMs() const;
  void advanceSleepMs(uint32_t elapsedMs);

private:
  uint64_t sleepOffsetMs = 0;
};

#endif
