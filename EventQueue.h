#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include <Arduino.h>

struct Event {
  uint16_t type = 0;
  uint16_t source = 0;
  int32_t code = 0;
  uint32_t value = 0;
  uint32_t timestamp = 0;
  uint16_t flags = 0;
};

template <uint8_t Capacity, typename EventT = Event>
class EventQueue {
public:
  static_assert(Capacity > 0, "EventQueue capacity must be greater than zero");

  bool push(const EventT& event) {
    bool dropped = false;

    if (_count >= Capacity) {
      _head = (_head + 1) % Capacity;
      _count--;
      if (_dropped < 255) {
        _dropped++;
      }
      dropped = true;
    }

    _events[_tail] = event;
    _tail = (_tail + 1) % Capacity;
    _count++;
    return !dropped;
  }

  bool push(uint16_t type,
            uint16_t source = 0,
            int32_t code = 0,
            uint32_t value = 0,
            uint16_t flags = 0,
            uint32_t timestamp = millis()) {
    Event event;
    event.type = type;
    event.source = source;
    event.code = code;
    event.value = value;
    event.flags = flags;
    event.timestamp = timestamp;
    return push(event);
  }

  bool pop(EventT& event) {
    if (_count == 0) {
      return false;
    }

    event = _events[_head];
    _head = (_head + 1) % Capacity;
    _count--;
    return true;
  }

  bool peek(EventT& event) const {
    if (_count == 0) {
      return false;
    }

    event = _events[_head];
    return true;
  }

  bool hasEvents() const {
    return _count > 0;
  }

  uint8_t count() const {
    return _count;
  }

  uint8_t capacity() const {
    return Capacity;
  }

  uint8_t droppedCount() const {
    return _dropped;
  }

  void resetDroppedCount() {
    _dropped = 0;
  }

  void clear() {
    _head = 0;
    _tail = 0;
    _count = 0;
  }

private:
  EventT _events[Capacity];
  uint8_t _head = 0;
  uint8_t _tail = 0;
  uint8_t _count = 0;
  uint8_t _dropped = 0;
};

#endif
