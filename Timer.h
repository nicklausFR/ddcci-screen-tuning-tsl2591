#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>
#include <Logger.h>

class Timer {
public:
  typedef unsigned long (*NowProvider)();

private:
  unsigned long startTime;
  unsigned long duration;
  bool running;
  bool reached;
  Logger defaultLog;
  Logger* log;
  NowProvider nowProvider;

  void traceDuration(const char* event);

public:
  Timer(unsigned long delayMs = 0);
  Timer(Logger& logger, unsigned long delayMs = 0);
  Timer(const Timer& other);

  Timer& operator=(const Timer& other);

  void start();
  void setNowProvider(NowProvider provider);
  bool isRunning() const;
  bool isReached();

  void setDuration(unsigned long delayMs);
  unsigned long getDuration() const;
};

#endif
