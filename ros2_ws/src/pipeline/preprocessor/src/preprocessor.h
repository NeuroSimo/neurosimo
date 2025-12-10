//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_PREPROCESSOR_H
#define EEG_PROCESSOR_PREPROCESSOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "project_interfaces/msg/preprocessor_list.hpp"
#include "project_interfaces/srv/set_preprocessor_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_PREVIOUS_TIME = std::numeric_limits<double_t>::quiet_NaN();
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
  double_t session_start_time = UNSET_PREVIOUS_TIME;
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
  ~EegPreprocessor();

private:
  void publish_healthcheck();

  void handle_session_start(const eeg_interfaces::msg::SessionMetadata& metadata);
  void handle_session_end();

  void initialize_module();
  void publish_python_logs(double sample_time, bool is_initialization);

  void unset_preprocessor_module();

  bool set_preprocessor_enabled(bool enabled);
  void handle_set_preprocessor_enabled(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::string get_module_name_with_fallback(const std::string module_name);
  bool set_preprocessor_module(const std::string module_name);
  void handle_set_preprocessor_module(
      const std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Response> response);

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  void update_preprocessor_list();

  void check_dropped_samples(double_t sample_time);

  void process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  
  void process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time);
  void process_ready_deferred_requests(double_t current_sample_time);

  /* File-system related functions */
  bool change_working_directory(const std::string path);
  std::vector<std::string> list_python_modules_in_working_directory();

  void update_inotify_watch();
  void inotify_timer_callback();

  rclcpp::Logger logger;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_subscriber;
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr preprocessed_eeg_publisher;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Publisher<project_interfaces::msg::PreprocessorList>::SharedPtr preprocessor_list_publisher;

  rclcpp::Service<project_interfaces::srv::SetPreprocessorModule>::SharedPtr set_preprocessor_module_service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preprocessor_module_publisher;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_preprocessor_enabled_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr preprocessor_enabled_publisher;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  bool enabled = false;
  bool is_session_ongoing = false;

  std::string active_project = UNSET_STRING;

  std::string working_directory  = UNSET_STRING;
  bool is_working_directory_set = false;
  std::string module_name = UNSET_STRING;

  std::vector<std::string> modules;

  std::deque<std::pair<double_t, int>> dropped_samples_window;

  uint16_t total_dropped_samples = 0;
  uint16_t dropped_sample_threshold;

  bool error_occurred = false;

  SessionMetadataState session_metadata;

  double_t previous_time = UNSET_PREVIOUS_TIME;

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

  /* Inotify variables */
  rclcpp::TimerBase::SharedPtr inotify_timer;
  int inotify_descriptor;
  int watch_descriptor;
  char inotify_buffer[1024];

  /* When determining if samples have been dropped by comparing the timestamps of two consecutive
     samples, allow some tolerance to account for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_PREPROCESSOR_H
