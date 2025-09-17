#include "labjack_manager.h"

#include <LabJackM.h>
#include "LJM_Utilities.h"

using namespace std::chrono;
using namespace std::chrono_literals;

LabJackManager::LabJackManager(rclcpp::Logger logger) : logger_(logger) {
}

LabJackManager::~LabJackManager() {
  stop();
}

void LabJackManager::start() {
  if (thread_running_.load()) {
    return; // Already running
  }
  
  thread_running_.store(true);
  should_attempt_connection_.store(true);
  connection_thread_ = std::thread(&LabJackManager::connection_worker, this);
  
  RCLCPP_INFO(logger_, "LabJackManager started");
}

void LabJackManager::stop() {
  thread_running_.store(false);
  connection_cv_.notify_one();
  
  if (connection_thread_.joinable()) {
    connection_thread_.join();
  }
  
  // Close LabJack connection if open
  int handle = labjack_handle_.load();
  if (handle != -1) {
    CloseOrDie(handle);
    labjack_handle_.store(-1);
  }
  
  RCLCPP_INFO(logger_, "LabJackManager stopped");
}

void LabJackManager::request_connection_attempt() {
  if (labjack_handle_.load() == -1) {
    should_attempt_connection_.store(true);
    connection_cv_.notify_one();
  }
}

bool LabJackManager::is_connected() const {
  return labjack_handle_.load() != -1;
}

bool LabJackManager::trigger_output(const char* output_name) {
  int handle = labjack_handle_.load();
  if (handle == -1) {
    return false;
  }

  // Set output port state to high
  int err = LJM_eWriteName(handle, output_name, 1);
  if (!check_error(err, "Setting digital output on LabJack")) {
    return false;
  }

  // Wait for one millisecond
  std::this_thread::sleep_for(1ms);

  // Set output port state to low
  err = LJM_eWriteName(handle, output_name, 0);
  if (!check_error(err, "Setting digital output on LabJack")) {
    return false;
  }

  return true;
}

void LabJackManager::set_connection_status_callback(ConnectionStatusCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  status_callback_ = callback;
}

void LabJackManager::connection_worker() {
  std::unique_lock<std::mutex> lock(connection_mutex_);
  
  while (thread_running_.load()) {
    // Wait for a connection attempt signal or timeout after 1 second
    connection_cv_.wait_for(lock, 1s, [this] {
      return should_attempt_connection_.load() || !thread_running_.load();
    });
    
    if (!thread_running_.load()) {
      break;
    }
    
    if (should_attempt_connection_.load() && labjack_handle_.load() == -1) {
      should_attempt_connection_.store(false);
      
      // Temporarily release the lock during the blocking operation
      lock.unlock();
      attempt_connection();
      lock.lock();
    }
  }
}

void LabJackManager::attempt_connection() {
  // Measure how long the connection attempt takes
  auto start_time = high_resolution_clock::now();
  
  int new_handle = -1;
  int err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &new_handle);
  
  auto end_time = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(end_time - start_time);
  
  bool success = (err == LJME_NOERROR);
  
  if (success) {
    labjack_handle_.store(new_handle);
    PrintDeviceInfoFromHandle(new_handle);
    RCLCPP_INFO(logger_, "Successfully connected to LabJack. Time taken: %ld ms", duration.count());
  } else {
    labjack_handle_.store(-1);
    RCLCPP_WARN(logger_, "Failed to connect to LabJack. Error code: %d, Time taken: %ld ms", err, duration.count());
  }
  
  // Notify callback if set
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (status_callback_) {
      status_callback_(success, err, duration.count());
    }
  }
}

bool LabJackManager::check_error(int err, const char* action) {
  if (err != LJME_NOERROR) {
    RCLCPP_ERROR(logger_, "%s failed with error code: %d", action, err);
    
    if (err == LJME_RECONNECT_FAILED) {
      // Mark as disconnected
      labjack_handle_.store(-1);
      RCLCPP_WARN(logger_, "LabJack connection lost. Will attempt to reconnect.");
      
      // Signal the connection thread to attempt reconnection
      should_attempt_connection_.store(true);
      connection_cv_.notify_one();
    }
    
    return false;
  }
  
  return true;
}
