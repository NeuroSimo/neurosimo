//
// Mock LabJack Manager - Sends socket triggers for simulation
// Created for neurosimo trigger_timer
//

#ifndef MOCK_LABJACK_MANAGER_H
#define MOCK_LABJACK_MANAGER_H

#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <functional>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "labjack_interface.h"

class MockLabJackManager : public LabJackInterface {
public:
  using ConnectionStatusCallback = std::function<void(bool connected, int error_code, int64_t connection_time_ms)>;

  explicit MockLabJackManager(rclcpp::Logger logger, const std::string& host = "127.0.0.1", int port = 60000);
  ~MockLabJackManager();

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

  // Socket connection details
  std::string host_;
  int port_;
  int socket_fd_{-1};

  // Connection state
  std::atomic<bool> connected_{false};

  // Threading for connection management
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

  // Socket operations
  bool connect_socket();
  void close_socket();
  bool send_trigger(const std::string& trigger_type);
};

#endif // MOCK_LABJACK_MANAGER_H