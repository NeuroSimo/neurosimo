#ifndef MTMS_EEG_BRIDGE_H
#define MTMS_EEG_BRIDGE_H

#include <cstdlib>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "eeg_interfaces/msg/eeg_info.hpp"
#include "eeg_interfaces/msg/sample.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"
#include "system_interfaces/msg/timed_trigger.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "adapters/eeg_adapter.h"

using namespace std::chrono_literals;

enum EegDeviceState {
  WAITING_FOR_EEG_DEVICE,
  EEG_DEVICE_STREAMING
};

enum ErrorState {
  NO_ERROR,
  ERROR_OUT_OF_SYNC,
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
  void process_eeg_data_packet();
  void update_healthcheck(uint8_t status, std::string status_message,
                          std::string actionable_message);

  void handle_sync_trigger(double_t sync_time);
  void handle_sample(eeg_interfaces::msg::Sample sample);

  void create_publishers();
  void create_subscribers();

  void publish_eeg_healthcheck();
  void publish_eeg_info();

  void subscribe_to_session();
  void subscribe_to_mtms_device_healthcheck();

  void stop_session();
  void wait_for_session();

  /* Configuration */
  bool is_mtms_device_enabled = false;
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
  rclcpp::Publisher<system_interfaces::msg::TimedTrigger>::SharedPtr latency_measurement_trigger_publisher;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;

  /* Subscribers */
  rclcpp::Subscription<system_interfaces::msg::Healthcheck>::SharedPtr
      mtms_device_healthcheck_subscriber;
  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;

  /* Session management */
  bool first_sample_of_session = true;
  uint64_t previous_sample_index = UNSET_PREVIOUS_SAMPLE_INDEX;

  double_t time_correction = UNSET_TIME; // in seconds
  double_t time_offset = UNSET_TIME;     // in seconds

  bool session_been_stopped = true;
  bool session_received = false;
  system_interfaces::msg::SessionState session_state;

  /* Time synchronisation */
  uint16_t num_of_sync_triggers_received;
  double_t first_sync_trigger_timestamp = UNSET_TIME;

  uint32_t sample_packets_received_since_last_sync = 0;

  bool mtms_device_available = false;

  /* Session management */

  /* Initialize 'wait_for_session_to_stop' to true to avoid starting streaming when EEG bridge is restarted in the middle
     of a session, while EEG device is already streaming (in which case we should wait for the session to stop before
     starting to stream because there is no way the session and the EEG device can be in sync). */
  bool wait_for_session_to_stop = true;

  /* Healthcheck */
  uint8_t status = system_interfaces::msg::HealthcheckStatus::NOT_READY;
  std::string status_message;
  std::string actionable_message;
};

#endif
