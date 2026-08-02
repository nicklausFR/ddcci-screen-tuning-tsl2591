#include "NrfClock.h"

void NrfClock::begin() {
  sleepOffsetMs = 0;
}

uint64_t NrfClock::monotonicMs() const {
  return sleepOffsetMs + (uint64_t) millis();
}

void NrfClock::advanceSleepMs(uint32_t elapsedMs) {
  sleepOffsetMs += elapsedMs;
}
