#include <Logger.h>

LogHandler::LogHandler()
    : _reportedDropped(0)
{
}

void LogHandler::pushDirect(const LogMessage& msg)
{
    _queue.push(msg);
}

void LogHandler::pushOverflowWarning()
{
    LogMessage warning;
    warning.type = LOG_WARN;
    warning.origin = "LOG";

    snprintf(
        warning.msg,
        sizeof(warning.msg),
        "Log FIFO full, %u message(s) dropped",
        (unsigned)(_queue.droppedCount() - _reportedDropped)
    );

    warning.msg[sizeof(warning.msg) - 1] = '\0';
    pushDirect(warning);
    _reportedDropped = _queue.droppedCount();
}

void LogHandler::push(const LogMessage& msg)
{
    _queue.push(msg);
}

bool LogHandler::hasEvents() const
{
    return _queue.hasEvents();
}

bool LogHandler::hasMessage()
{
    return hasEvents();
}

uint8_t LogHandler::droppedCount() const
{
    return _queue.droppedCount();
}

bool LogHandler::pop(LogMessage& msg)
{
    if (!_queue.pop(msg))
    {
        return false;
    }

    if (_queue.count() < FIFO_SIZE && _queue.droppedCount() > _reportedDropped)
    {
        pushOverflowWarning();
    }

    return true;
}

Logger::Logger()
    : _source("APP"),
      _output(nullptr),
      _handler(nullptr),
      _level(LOG_NOTICE)
{
}

Logger::Logger(const char* source)
    : _source(source != nullptr ? source : "APP"),
      _output(nullptr),
      _handler(nullptr),
      _level(LOG_NOTICE)
{
}

Logger::Logger(const char* source, LogLevel level)
    : _source(source != nullptr ? source : "APP"),
      _output(nullptr),
      _handler(nullptr),
      _level(level)
{
}

Logger::Logger(const char* source, LogLevel level, LogHandler& handler)
    : _source(source != nullptr ? source : "APP"),
      _output(nullptr),
      _handler(&handler),
      _level(level)
{
}

Logger::Logger(const char* source, Stream* output)
    : _source(source != nullptr ? source : "APP"),
      _output(output),
      _handler(nullptr),
      _level(LOG_NOTICE)
{
}

const char* Logger::typeText(LogLevel level)
{
    switch (level)
    {
        case LOG_TRACE:
            return "TRACE";

        case LOG_DEBUG:
            return "DEBUG";

        case LOG_NOTICE:
            return "NOTICE";

        case LOG_WARN:
            return "WARN";

        case LOG_ERROR:
            return "ERROR";

        default:
            return "UNKNOWN";
    }
}

void Logger::begin(Stream* output)
{
    _output = output;
    _handler = nullptr;
}

void Logger::to(LogHandler& handler)
{
    _output = nullptr;
    _handler = &handler;
}

void Logger::setLevel(LogLevel level)
{
    _level = level;
}

void Logger::trace(const char* message)
{
    log(LOG_TRACE, message);
}

void Logger::debug(const char* message)
{
    log(LOG_DEBUG, message);
}

void Logger::notice(const char* message)
{
    log(LOG_NOTICE, message);
}

void Logger::warn(const char* message)
{
    log(LOG_WARN, message);
}

void Logger::error(const char* message)
{
    log(LOG_ERROR, message);
}

void Logger::log(
    LogLevel level,
    const char* message
)
{
    const char* safeMessage = message != nullptr ? message : "";

    if (level < _level)
    {
        return;
    }

    LogMessage msg;
    msg.type = level;
    msg.origin = _source;

    strncpy(
        msg.msg,
        safeMessage,
        sizeof(msg.msg) - 1
    );

    msg.msg[sizeof(msg.msg) - 1] = '\0';

    if (_output != nullptr)
    {
        _output->print("[");
        _output->print(msg.origin);
        _output->print("][");
        _output->print(typeText(msg.type));
        _output->print("] ");
        _output->println(msg.msg);
        return;
    }

    if (_handler != nullptr)
    {
        _handler->push(msg);
    }
}
