#include "mock_labjack_manager.h"

#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;

MockLabJackManager::MockLabJackManager(rclcpp::Logger logger, const std::string& host, int port)
  : logger_(logger), host_(host), port_(port) {
}

MockLabJackManager::~MockLabJackManager() {
  stop();
}

void MockLabJackManager::start() {
  if (thread_running_.load()) {
    return; // Already running
  }

  thread_running_.store(true);
  should_attempt_connection_.store(true);
  connection_thread_ = std::thread(&MockLabJackManager::connection_worker, this);

  RCLCPP_DEBUG(logger_, "MockLabJackManager started");
}

void MockLabJackManager::stop() {
  thread_running_.store(false);
  connection_cv_.notify_one();

  if (connection_thread_.joinable()) {
    connection_thread_.join();
  }

  close_socket();

  RCLCPP_DEBUG(logger_, "MockLabJackManager stopped");
}

void MockLabJackManager::request_connection_attempt() {
  if (!connected_.load()) {
    should_attempt_connection_.store(true);
    connection_cv_.notify_one();
  }
}

bool MockLabJackManager::is_connected() const {
  return connected_.load();
}

bool MockLabJackManager::trigger_output(const char* output_name) {
  if (!connected_.load()) {
    RCLCPP_WARN(logger_, "MockLabJack not connected, cannot send trigger");
    return false;
  }

  std::string trigger_type;

  // Map output names to trigger types
  if (strcmp(output_name, "FIO5") == 0) {
    trigger_type = "pulse_trigger";
  } else if (strcmp(output_name, "FIO4") == 0) {
    trigger_type = "latency_trigger";
  } else {
    RCLCPP_ERROR(logger_, "Unknown output name: %s", output_name);
    return false;
  }

  if (!send_trigger(trigger_type)) {
    RCLCPP_ERROR(logger_, "Failed to send %s trigger", trigger_type.c_str());
    return false;
  }

  RCLCPP_DEBUG(logger_, "Sent %s trigger", trigger_type.c_str());

  // Simulate the same 1ms delay as real hardware
  std::this_thread::sleep_for(1ms);

  return true;
}

void MockLabJackManager::set_connection_status_callback(ConnectionStatusCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  status_callback_ = callback;
}

void MockLabJackManager::connection_worker() {
  std::unique_lock<std::mutex> lock(connection_mutex_);

  while (thread_running_.load()) {
    // Wait for a connection attempt signal or timeout after 1 second
    connection_cv_.wait_for(lock, 1s, [this] {
      return should_attempt_connection_.load() || !thread_running_.load();
    });

    if (!thread_running_.load()) {
      break;
    }

    if (should_attempt_connection_.load() && !connected_.load()) {
      should_attempt_connection_.store(false);

      // Temporarily release the lock during the blocking operation
      lock.unlock();
      attempt_connection();
      lock.lock();
    }
  }
}

void MockLabJackManager::attempt_connection() {
  // Measure how long the connection attempt takes
  auto start_time = high_resolution_clock::now();

  bool socket_connected = connect_socket();

  auto end_time = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(end_time - start_time);

  // For mock purposes, always report as connected regardless of socket status
  connected_.store(true);

  if (socket_connected) {
    RCLCPP_INFO(logger_, "Successfully connected to neurone_simulator socket. Time taken: %ld ms", duration.count());
  } else {
    RCLCPP_INFO(logger_, "MockLabJack connected (socket connection failed but mocking enabled)");
  }

  // Notify callback if set - always report success for mock
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (status_callback_) {
      status_callback_(true, 0, duration.count());
    }
  }
}

bool MockLabJackManager::connect_socket() {
  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create socket");
    return false;
  }

  // Set up server address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr) <= 0) {
    RCLCPP_WARN(logger_, "Invalid address: %s (continuing with mock)", host_.c_str());
    close_socket();
    return false;
  }

  // Connect to server
  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(logger_, "Failed to connect to %s:%d", host_.c_str(), port_);
    close_socket();
    return false;
  }

  return true;
}

void MockLabJackManager::close_socket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_.store(false);
}

bool MockLabJackManager::send_trigger(const std::string& trigger_type) {
  if (!connected_.load()) {
    return false;
  }

  // Try to send via socket if connected, otherwise just mock the trigger
  if (socket_fd_ >= 0) {
    // Send the trigger type as a string followed by newline
    std::string message = trigger_type + "\n";
    ssize_t sent = send(socket_fd_, message.c_str(), message.length(), 0);

    if (sent < 0) {
      RCLCPP_WARN(logger_, "Failed to send trigger message via socket, continuing with mock");
      // Don't set connected to false - we're mocking
    } else {
      RCLCPP_DEBUG(logger_, "Sent %s trigger via socket", trigger_type.c_str());
    }
  } else {
    RCLCPP_DEBUG(logger_, "Mocking %s trigger (no socket connection)", trigger_type.c_str());
  }

  return true;
}