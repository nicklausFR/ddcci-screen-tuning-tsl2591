#ifndef WAKE_HANDLER_H
#define WAKE_HANDLER_H

#include <Arduino.h>
#include <NrfClock.h>

typedef uint32_t WakeMask;

class WakeHandler {
public:
  WakeHandler();

  static WakeMask source(uint8_t index);

  void begin(NrfClock& clock);
  void setClock(NrfClock* clock);
  uint64_t nowMs() const;

  void notify(WakeMask sourceMask);
  bool hasWake(WakeMask sourceMask) const;
  bool hasAnyWake() const;
  WakeMask pending() const;

  bool consume(WakeMask sourceMask);
  WakeMask consumeAll();
  void clear(WakeMask sourceMask);
  void clearAll();

  bool hasAnyWakeFromCritical() const;

  bool scheduleInterval(
    uint8_t slot,
    WakeMask sourceMask,
    unsigned long intervalMs,
    bool dueNow = false
  );
  bool scheduleOnceAt(uint8_t slot, WakeMask sourceMask, uint64_t dueAtMs);
  void disableSchedule(uint8_t slot);
  WakeMask updateSchedules();
  WakeMask updateSchedules(uint64_t nowMs);
  bool hasSchedule() const;
  unsigned long nextScheduleDelayMs() const;
  unsigned long nextScheduleDelayMs(uint64_t nowMs) const;

  void beginRtcWake(WakeMask rtcWakeSourceMask = 0);
  void scheduleRtcWakeIn(unsigned long delayMs);
  void scheduleRtcWakeForNextSchedule();
  void syncClockFromRtc();
  static void handleRtc2Interrupt();

private:
  struct WakeSchedule {
    WakeMask sourceMask;
    uint64_t dueAtMs;
    unsigned long intervalMs;
    bool enabled;
    bool repeating;
  };

  static const uint8_t MAX_SCHEDULES = 8;

  volatile WakeMask _pending;
  WakeSchedule _schedules[MAX_SCHEDULES];
  WakeMask _rtcWakeSourceMask;
  NrfClock* _clock;
  uint32_t _scheduledRtcWakeMs;
  uint32_t _rtcLastCounter;
  bool _rtcClockSyncReady;

  static uint8_t saveInterruptState();
  static void restoreInterruptState(uint8_t state);
  static bool timeReached(uint64_t nowMs, uint64_t dueAtMs);
  static WakeHandler* activeRtcHandler;
};

#endif
