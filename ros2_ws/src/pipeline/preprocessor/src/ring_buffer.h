#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <vector>
#include <stdexcept>

#include "eeg_msgs/msg/sample.hpp"

#include <vector>
#include <stdexcept>

template<typename T>
class RingBuffer {
private:
  std::vector<T> buffer;
  size_t capacity;
  size_t current_size;
  size_t head;
  size_t tail;

public:
  RingBuffer() : capacity(0), current_size(0), head(0), tail(0) {}

  /* Reset buffer and set its size. Discards all current data. */
  void reset(size_t size) {
    buffer.resize(size);

    capacity = size;
    current_size = 0;
    head = 0;
    tail = 0;
  }

  /* Appends a message to the buffer, overwriting the oldest message if buffer is full. */
  void append(const T& msg) {
    buffer[tail] = msg;
    tail = (tail + 1) % capacity;

    if (current_size < capacity) {
      current_size++;
    } else {
      head = (head + 1) % capacity;
    }
  }

  /* Process each element in the buffer using a provided callback function. */
  void process_elements(std::function<void(const T&)> callback) const {
    for (size_t i = 0; i < current_size; ++i) {
      callback(buffer[(head + i) % capacity]);
    }
  }

  /* Queries if the buffer is full. */
  bool is_full() const {
    return current_size == capacity;
  }
};

#endif // RING_BUFFER_H
