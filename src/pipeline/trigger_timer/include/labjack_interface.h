//
// LabJack Interface - Common interface for LabJack and MockLabJack managers
// Created for neurosimo trigger_timer
//

#ifndef LABJACK_INTERFACE_H
#define LABJACK_INTERFACE_H

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"

class LabJackInterface {
public:
  using ConnectionStatusCallback = std::function<void(bool connected, int error_code, int64_t connection_time_ms)>;

  virtual ~LabJackInterface() = default;

  // Connection management
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void request_connection_attempt() = 0;
  virtual bool is_connected() const = 0;

  // Triggering operations
  virtual bool trigger_output(const char* output_name) = 0;

  // Callback for connection status changes
  virtual void set_connection_status_callback(ConnectionStatusCallback callback) = 0;
};

#endif // LABJACK_INTERFACE_H