//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_PREPROCESSOR_H
#define EEG_PROCESSOR_PREPROCESSOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/preprocessed_sample.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"

#include "project_interfaces/msg/preprocessor_list.hpp"
#include "project_interfaces/srv/set_preprocessor_module.hpp"
#include "project_interfaces/srv/set_preprocessor_enabled.hpp"

#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_PREVIOUS_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

enum class PreprocessorState {
  WAITING_FOR_ENABLED,
  READY,
  SAMPLES_DROPPED,
  DROPPED_SAMPLE_THRESHOLD_EXCEEDED,
  MODULE_ERROR
};

class PreprocessorWrapper;

class EegPreprocessor : public rclcpp::Node {
public:
  EegPreprocessor();
  ~EegPreprocessor();

private:
  void publish_healthcheck();

  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);

  void update_eeg_info(const eeg_msgs::msg::SampleMetadata& msg);
  void initialize_module();
  void publish_python_logs(double sample_time, bool is_initialization);

  void reset_preprocessor_state();

  void unset_preprocessor_module();

  bool set_preprocessor_enabled(bool enabled);
  void handle_set_preprocessor_enabled(
      const std::shared_ptr<project_interfaces::srv::SetPreprocessorEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPreprocessorEnabled::Response> response);

  std::string get_module_name_with_fallback(const std::string module_name);
  bool set_preprocessor_module(const std::string module_name);
  void handle_set_preprocessor_module(
      const std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Response> response);

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  void update_preprocessor_list();

  void check_dropped_samples(double_t sample_time);

  bool is_pulse_feedback_received(double_t sample_time);

  void process_sample(const std::shared_ptr<eeg_msgs::msg::Sample> msg);

  /* File-system related functions */
  bool change_working_directory(const std::string path);
  std::vector<std::string> list_python_modules_in_working_directory();

  void update_inotify_watch();
  void inotify_timer_callback();

  rclcpp::Logger logger;

  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;

  rclcpp::Subscription<eeg_msgs::msg::Sample>::SharedPtr raw_eeg_subscriber;
  rclcpp::Publisher<eeg_msgs::msg::PreprocessedSample>::SharedPtr preprocessed_eeg_publisher;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Publisher<project_interfaces::msg::PreprocessorList>::SharedPtr preprocessor_list_publisher;

  rclcpp::Service<project_interfaces::srv::SetPreprocessorModule>::SharedPtr set_preprocessor_module_service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preprocessor_module_publisher;

  rclcpp::Service<project_interfaces::srv::SetPreprocessorEnabled>::SharedPtr set_preprocessor_enabled_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr preprocessor_enabled_publisher;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  bool enabled = false;

  PreprocessorState preprocessor_state = PreprocessorState::WAITING_FOR_ENABLED;
  system_interfaces::msg::SessionState session_state;

  bool first_sample_ever = true;
  bool first_sample_of_session = false;

  bool reinitialize = true;

  std::string active_project = UNSET_STRING;

  std::string working_directory  = UNSET_STRING;
  bool is_working_directory_set = false;
  std::string module_name = UNSET_STRING;

  std::vector<std::string> modules;

  std::deque<std::pair<double_t, int>> dropped_samples_window;

  uint16_t total_dropped_samples = 0;
  uint16_t dropped_sample_threshold;

  uint16_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;
  uint8_t num_of_eeg_channels = UNSET_NUM_OF_CHANNELS;
  uint8_t num_of_emg_channels = UNSET_NUM_OF_CHANNELS;

  double_t sampling_period;

  double_t previous_time = UNSET_PREVIOUS_TIME;

  std::queue<double_t> pulse_execution_times;

  RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>> sample_buffer;
  eeg_msgs::msg::PreprocessedSample preprocessed_sample;

  std::unique_ptr<PreprocessorWrapper> preprocessor_wrapper;

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
