//
// LabJack Manager - Handles all LabJack device communication
// Created for neurosimo trigger_timer
//

#ifndef LABJACK_MANAGER_H
#define LABJACK_MANAGER_H

#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"

class LabJackManager {
public:
  using ConnectionStatusCallback = std::function<void(bool connected, int error_code, int64_t connection_time_ms)>;

  explicit LabJackManager(rclcpp::Logger logger);
  ~LabJackManager();

  // Connection management
  void start();
  void stop();
  void request_connection_attempt();
  bool is_connected() const;

  // Triggering operations
  bool trigger_output(const char* output_name);

  // Callback for connection status changes
  void set_connection_status_callback(ConnectionStatusCallback callback);

private:
  rclcpp::Logger logger_;
  
  // LabJack handle and connection state
  std::atomic<int> labjack_handle_{-1};
  
  // Threading for non-blocking connection attempts
  std::thread connection_thread_;
  std::atomic<bool> thread_running_{false};
  std::atomic<bool> should_attempt_connection_{false};
  std::mutex connection_mutex_;
  std::condition_variable connection_cv_;
  
  // Connection status callback
  ConnectionStatusCallback status_callback_;
  std::mutex callback_mutex_;
  
  // Worker thread function
  void connection_worker();
  
  // Internal connection attempt (blocking)
  void attempt_connection();
  
  // Error handling
  bool check_error(int err, const char* action);
};

#endif // LABJACK_MANAGER_H
