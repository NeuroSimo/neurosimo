#ifndef EEG_BRIDGE_H
#define EEG_BRIDGE_H

#include <cstdlib>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "eeg_interfaces/msg/eeg_info.hpp"
#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/latency_measurement_trigger.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"
#include "system_interfaces/msg/streamer_state.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "adapters/eeg_adapter.h"

using namespace std::chrono_literals;

enum EegDeviceState {
  WAITING_FOR_EEG_DEVICE,
  EEG_DEVICE_STREAMING
};

enum ErrorState {
  NO_ERROR,
  ERROR_SAMPLES_DROPPED
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
  void process_eeg_packet();
  void update_healthcheck(uint8_t status, std::string status_message,
                          std::string actionable_message);

  eeg_interfaces::msg::Sample create_ros_sample(const AdapterSample& adapter_sample,
                                          const eeg_interfaces::msg::EegInfo& eeg_info);

  void handle_sample(eeg_interfaces::msg::Sample sample);
  bool check_for_dropped_samples(uint64_t device_sample_index);

  void create_publishers();

  void publish_eeg_healthcheck();
  void publish_eeg_info();
  void publish_streamer_state();

  void set_eeg_device_state(EegDeviceState new_state);

  void handle_start_streaming(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_stop_streaming(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /* Configuration */
  uint16_t port = 0;
  uint8_t num_of_tolerated_dropped_samples = 0;
  EegDevice eeg_device;

  std::shared_ptr<EegAdapter> eeg_adapter;

  /* State */
  EegDeviceState eeg_device_state = EegDeviceState::WAITING_FOR_EEG_DEVICE;
  ErrorState error_state = ErrorState::NO_ERROR;

  /* Publishers */
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr eeg_sample_publisher;
  rclcpp::Publisher<eeg_interfaces::msg::EegInfo>::SharedPtr eeg_info_publisher;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;
  rclcpp::Publisher<system_interfaces::msg::StreamerState>::SharedPtr streamer_state_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::LatencyMeasurementTrigger>::SharedPtr latency_measurement_trigger_publisher;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;

  /* Services */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_streaming_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_streaming_service;

  /* Streaming state */
  system_interfaces::msg::StreamerState::_state_type streamer_state = system_interfaces::msg::StreamerState::READY;
  bool is_session_start = false;
  bool is_session_end = false;
  
  /* Device sample tracking for dropped sample detection */
  uint64_t previous_device_sample_index = UNSET_PREVIOUS_SAMPLE_INDEX;
  
  /* Streaming sample index (starts at 0 for each streaming run) */
  uint64_t session_sample_index = 0;

  double_t time_offset = UNSET_TIME;     // in seconds
  double_t session_start_time = UNSET_TIME;  // Unix timestamp when streaming started

  /* Healthcheck */
  uint8_t status = system_interfaces::msg::HealthcheckStatus::NOT_READY;
  std::string status_message;
  std::string actionable_message;
};

#endif
