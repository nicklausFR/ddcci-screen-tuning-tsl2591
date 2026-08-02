#include <Timer.h>

Timer::Timer(unsigned long delayMs)
  : startTime(0),
    duration(delayMs),
    running(false),
    reached(false),
    defaultLog("TIMER"),
    log(&defaultLog),
    nowProvider(millis)
{
  traceDuration("CREATED");
}

Timer::Timer(Logger& logger, unsigned long delayMs)
  : startTime(0),
    duration(delayMs),
    running(false),
    reached(false),
    defaultLog("TIMER"),
    log(&logger),
    nowProvider(millis)
{
  traceDuration("CREATED");
}

Timer::Timer(const Timer& other)
  : startTime(other.startTime),
    duration(other.duration),
    running(other.running),
    reached(other.reached),
    defaultLog("TIMER"),
    log(other.log == &other.defaultLog ? &defaultLog : other.log),
    nowProvider(other.nowProvider)
{
}

Timer& Timer::operator=(const Timer& other)
{
  if (this == &other) {
    return *this;
  }

  startTime = other.startTime;
  duration = other.duration;
  running = other.running;
  reached = other.reached;
  log = other.log == &other.defaultLog ? &defaultLog : other.log;
  nowProvider = other.nowProvider;

  return *this;
}

void Timer::traceDuration(const char* event) {
  char message[40];

  snprintf(
    message,
    sizeof(message),
    "%s duration=%lu",
    event,
    duration
  );

  log->trace(message);
}

void Timer::start() {
  startTime = nowProvider();
  running = true;
  reached = false;

  traceDuration("START");
}

void Timer::setNowProvider(NowProvider provider) {
  nowProvider = provider != nullptr ? provider : millis;
}

bool Timer::isRunning() const {
  return running;
}

bool Timer::isReached() {
  if (
    running &&
    !reached &&
    nowProvider() - startTime >= duration
  ) {
    reached = true;
    traceDuration("END");
    return true;
  }

  return false;
}

void Timer::setDuration(unsigned long delayMs) {
  duration = delayMs;
  running = false;
  reached = false;
  startTime = 0;

  traceDuration("SET");
}

unsigned long Timer::getDuration() const {
  return duration;
}
