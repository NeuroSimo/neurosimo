#ifndef EEG_BRIDGE_H
#define EEG_BRIDGE_H

#include <cstdlib>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "eeg_interfaces/msg/eeg_device_info.hpp"
#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/srv/start_streaming.hpp"
#include "eeg_interfaces/srv/stop_streaming.hpp"
#include "eeg_interfaces/srv/initialize_eeg_device_stream.hpp"

#include "system_interfaces/msg/component_health.hpp"
#include "system_interfaces/srv/abort_session.hpp"
#include "system_interfaces/msg/data_source_state.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "adapters/eeg_adapter.h"
#include "adapters/socket.h"

using namespace std::chrono_literals;

enum EegDeviceState {
  WAITING_FOR_EEG_DEVICE,
  EEG_DEVICE_STREAMING
};

/**
 * Supported EEG Devices
 *
 * List the options for the currently supported EEG devices. Used to define, which
 * adpter to use for the socket communnication between this software and the eeg device.
 */
enum EegDevice {
  NEURONE,
  TURBOLINK,
};

const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const uint64_t UNSET_PREVIOUS_SAMPLE_INDEX = std::numeric_limits<uint64_t>::max();

/**
 * Translate data from EEG adapter interface to ROS messages.
 *
 * Follows the adapter structure where class implementing EegAdapter interface
 * will be used to translate the raw socket data from the EEG device into common
 * format.
 *
 * Currently supported EEG devices are:
 *   - Bittium NeurOne
 */
class EegBridge : public rclcpp::Node {

public:
  EegBridge();

  void spin();

private:
  bool reset_state();

  void process_eeg_packet();

  eeg_interfaces::msg::Sample create_ros_sample(const AdapterSample& adapter_sample,
                                          const eeg_interfaces::msg::EegDeviceInfo& device_info);

  void handle_sample(eeg_interfaces::msg::Sample sample);
  bool check_for_dropped_samples(uint64_t device_sample_index);
  void check_for_sample_timeout();

  void create_publishers();

  void publish_heartbeat();
  void publish_health_status(uint8_t health_level, const std::string& message);
  void publish_device_info();
  void publish_data_source_state();

  void set_device_state(EegDeviceState new_state);

  void abort_session(const std::string& reason);

  void handle_start_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StartStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StartStreaming::Response> response);
  void handle_stop_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StopStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StopStreaming::Response> response);
  void handle_initialize(
      const std::shared_ptr<eeg_interfaces::srv::InitializeEegDeviceStream::Request> request,
      std::shared_ptr<eeg_interfaces::srv::InitializeEegDeviceStream::Response> response);

  /* Configuration */
  uint16_t port = 0;
  uint8_t num_of_tolerated_dropped_samples = 0;
  EegDevice eeg_device;

  std::shared_ptr<UdpSocket> socket_;
  std::shared_ptr<EegAdapter> eeg_adapter;
  uint8_t buffer[BUFFER_SIZE] = {0};

  /* State */
  EegDeviceState device_state = EegDeviceState::WAITING_FOR_EEG_DEVICE;
  std::chrono::steady_clock::time_point last_sample_time;

  /* Publishers */
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr eeg_sample_publisher;
  rclcpp::Publisher<eeg_interfaces::msg::EegDeviceInfo>::SharedPtr device_info_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;
  rclcpp::Publisher<system_interfaces::msg::DataSourceState>::SharedPtr data_source_state_publisher;

  rclcpp::TimerBase::SharedPtr heartbeat_publisher_timer;

  /* Services */
  rclcpp::Service<eeg_interfaces::srv::StartStreaming>::SharedPtr start_streaming_service;
  rclcpp::Service<eeg_interfaces::srv::StopStreaming>::SharedPtr stop_streaming_service;
  rclcpp::Service<eeg_interfaces::srv::InitializeEegDeviceStream>::SharedPtr initialize_service;

  /* Service client for session abort */
  rclcpp::Client<system_interfaces::srv::AbortSession>::SharedPtr abort_session_client;

  /* Data source state */
  system_interfaces::msg::DataSourceState::_state_type data_source_state = system_interfaces::msg::DataSourceState::READY;
  bool is_session_start = false;
  
  /* Device sample tracking for dropped sample detection */
  uint64_t previous_device_sample_index = UNSET_PREVIOUS_SAMPLE_INDEX;
  
  /* Streaming sample index (starts at 0 for each streaming run) */
  uint64_t session_sample_index = 0;

  double_t time_offset = UNSET_TIME;     // in seconds

};

#endif
