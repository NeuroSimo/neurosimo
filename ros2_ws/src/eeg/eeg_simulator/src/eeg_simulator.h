#ifndef EEG_SIMULATOR_H
#define EEG_SIMULATOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/stream_info.hpp"
#include "eeg_interfaces/action/initialize_simulator_stream.hpp"
#include "eeg_interfaces/srv/start_streaming.hpp"
#include "eeg_interfaces/srv/stop_streaming.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

#include "project_interfaces/msg/dataset_info.hpp"

#include "dataset_manager.h"

#include "std_srvs/srv/trigger.hpp"

#include "system_interfaces/msg/component_health.hpp"
#include "system_interfaces/msg/data_source_state.hpp"

const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

class EegSimulator : public rclcpp::Node {
public:
  EegSimulator();

private:
  void publish_heartbeat();
  void publish_health_status(uint8_t health_level, const std::string& message);

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);

  rclcpp_action::GoalResponse handle_initialize_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const eeg_interfaces::action::InitializeSimulatorStream::Goal> goal);
  rclcpp_action::CancelResponse handle_initialize_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle);
  void handle_initialize_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle);
  void execute_initialize(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle);

  /* Publish a single sample at the given index with the specified session flags. */
  bool publish_single_sample(size_t sample_index, bool is_session_start, bool is_session_end);

  /* Publish samples from current_index until (but not including) the first sample that is after until_time.
     Returns true if the samples were published successfully. */
  bool publish_until(double_t until_time);

  void publish_data_source_state();
  void stream_timer_callback();
  void handle_start_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StartStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StartStreaming::Response> response);
  void handle_stop_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StopStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StopStreaming::Response> response);

  project_interfaces::msg::DatasetInfo dataset_info;

  system_interfaces::msg::DataSourceState::_state_type data_source_state = system_interfaces::msg::DataSourceState::READY;

  double_t play_dataset_from = 0.0;

  size_t current_index = 0;
  size_t current_pulse_index = 0;

  std::string error_message = UNSET_STRING;

  double_t sampling_period;

  double_t session_start_time = UNSET_TIME;  // Unix timestamp when streaming started
  uint64_t session_sample_index = 0;         // Monotonic sample index within a streaming run
  bool is_session_start = false;
  bool is_session_end = false;

  std::vector<std::vector<double_t>> dataset_buffer;
  size_t current_sample_index = 0;
  
  double_t latest_sample_time = 0.0;
  double_t time_offset = 0.0;

  /* Pulse times loaded from dataset JSON, used to inject pulse_trigger flags. */
  std::vector<double_t> pulse_times;

  /* Initialization state */
  bool is_initialized = false;
  std::string initialized_project_name;
  std::string initialized_dataset_filename;
  double_t initialized_start_time;

  rclcpp::CallbackGroup::SharedPtr callback_group;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;
  rclcpp::TimerBase::SharedPtr heartbeat_publisher_timer;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;

  std::unique_ptr<DatasetManager> dataset_manager_;

  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr eeg_publisher;

  rclcpp::Publisher<system_interfaces::msg::DataSourceState>::SharedPtr data_source_state_publisher;
  rclcpp::Service<eeg_interfaces::srv::StartStreaming>::SharedPtr start_streaming_service;
  rclcpp::Service<eeg_interfaces::srv::StopStreaming>::SharedPtr stop_streaming_service;
  rclcpp::TimerBase::SharedPtr stream_timer;

  /* Action server for initialization */
  rclcpp_action::Server<eeg_interfaces::action::InitializeSimulatorStream>::SharedPtr initialize_action_server;
};

#endif //EEG_SIMULATOR_H
