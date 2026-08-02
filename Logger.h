#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <EventQueue.h>

enum LogLevel {
    LOG_TRACE,
    LOG_DEBUG,
    LOG_NOTICE,
    LOG_WARN,
    LOG_ERROR
};

struct LogMessage
{
    LogLevel type;
    const char* origin;
    char msg[128];
};

class LogHandler
{
public:
    LogHandler();

    void push(const LogMessage& msg);
    bool pop(LogMessage& msg);
    bool hasEvents() const;
    bool hasMessage();
    uint8_t droppedCount() const;

private:
    static const uint8_t FIFO_SIZE = 16;

    EventQueue<FIFO_SIZE, LogMessage> _queue;
    uint8_t _reportedDropped;

    void pushDirect(const LogMessage& msg);
    void pushOverflowWarning();
};

class Logger {
public:
    Logger();
    Logger(const char* source);
    Logger(const char* source, LogLevel level);
    Logger(const char* source, LogLevel level, LogHandler& handler);
    Logger(const char* source, Stream* output);

    void begin(Stream* output);
    void to(LogHandler& handler);
    void setLevel(LogLevel level);

    void trace(const char* message);
    void debug(const char* message);
    void notice(const char* message);
    void warn(const char* message);
    void error(const char* message);

    static const char* typeText(LogLevel level);

private:
    const char* _source;
    Stream* _output;
    LogHandler* _handler;
    LogLevel _level;

    void log(
        LogLevel level,
        const char* message
    );
};

#endif
