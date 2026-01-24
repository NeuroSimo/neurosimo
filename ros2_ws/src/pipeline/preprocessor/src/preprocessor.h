//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_PREPROCESSOR_H
#define EEG_PROCESSOR_PREPROCESSOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "inotify_utils/inotify_watcher.h"

#include "preprocessor_wrapper.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"
#include "pipeline_interfaces/action/initialize_preprocessor.hpp"
#include "pipeline_interfaces/srv/finalize_preprocessor.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

struct DeferredProcessingRequest {
  /* The time when processing should actually occur (after look-ahead samples have arrived). */
  double_t scheduled_time;
  
  /* The sample that triggered the processing request. */
  std::shared_ptr<eeg_interfaces::msg::Sample> triggering_sample;
  
  /* Comparison operator for priority queue (min-heap by scheduled_time). */
  bool operator>(const DeferredProcessingRequest& other) const {
    return scheduled_time > other.scheduled_time;
  }
};

struct SessionMetadataState {
  uint16_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;
  uint8_t num_eeg_channels = UNSET_NUM_OF_CHANNELS;
  uint8_t num_emg_channels = UNSET_NUM_OF_CHANNELS;
  double_t session_start_time = UNSET_TIME;
  double_t sampling_period = 0.0;

  void update(const eeg_interfaces::msg::SessionMetadata& msg) {
    sampling_frequency = msg.sampling_frequency;
    num_eeg_channels = msg.num_eeg_channels;
    num_emg_channels = msg.num_emg_channels;
    session_start_time = msg.start_time;
    sampling_period = sampling_frequency ? 1.0 / sampling_frequency : 0.0;
  }

  bool matches(const eeg_interfaces::msg::SessionMetadata& msg) const {
    return msg.sampling_frequency == sampling_frequency &&
           msg.num_eeg_channels == num_eeg_channels &&
           msg.num_emg_channels == num_emg_channels &&
           msg.start_time == session_start_time;
  }
};

class PreprocessorWrapper;

class EegPreprocessor : public rclcpp::Node {
public:
  EegPreprocessor();

private:
  void publish_healthcheck();

  void handle_session_start(const eeg_interfaces::msg::SessionMetadata& metadata);
  void handle_session_end();

  rclcpp_action::GoalResponse handle_initialize_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipeline_interfaces::action::InitializePreprocessor::Goal> goal);
  rclcpp_action::CancelResponse handle_initialize_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle);
  void handle_initialize_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle);
  void execute_initialize(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle);
  void handle_finalize_preprocessor(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Response> response);
  void publish_python_logs(double sample_time, bool is_initialization);
  void publish_sentinel_sample(double_t sample_time);

  void process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  bool is_sample_window_valid() const;
  void enqueue_deferred_request(const std::shared_ptr<eeg_interfaces::msg::Sample> msg, double_t sample_time);
  void process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time);
  void process_ready_deferred_requests(double_t current_sample_time);

  rclcpp::Logger logger;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_subscriber;
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr preprocessed_eeg_publisher;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  /* Action server for initialization */
  rclcpp_action::Server<pipeline_interfaces::action::InitializePreprocessor>::SharedPtr initialize_action_server;

  /* Service server for finalization */
  rclcpp::Service<pipeline_interfaces::srv::FinalizePreprocessor>::SharedPtr finalize_service_server;

  /* Initialization state */
  bool is_initialized = false;
  bool is_enabled = false;
  std::string initialized_project_name;
  std::string initialized_module_filename;
  std::filesystem::path initialized_working_directory;

  bool error_occurred = false;

  /* Pending session markers to be carried forward to the next published sample. */
  bool pending_session_start = false;
  bool pending_session_end = false;

  SessionMetadataState session_metadata;

  RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>> sample_buffer;
  eeg_interfaces::msg::Sample preprocessed_sample;

  std::unique_ptr<PreprocessorWrapper> preprocessor_wrapper;

  /* Deferred processing queue for handling look-ahead samples. */
  std::priority_queue<DeferredProcessingRequest,
                      std::vector<DeferredProcessingRequest>,
                      std::greater<DeferredProcessingRequest>> deferred_processing_queue;

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;

  /* When determining if a sample is ready to be processed, allow some tolerance to account
     for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_PREPROCESSOR_H
