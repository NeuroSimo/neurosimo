//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_DECIDER_H
#define EEG_PROCESSOR_DECIDER_H

#include <deque>
#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/preprocessed_sample.hpp"

#include "pipeline_interfaces/srv/request_timed_trigger.hpp"

#include "mtms_trial_interfaces/msg/trial.hpp"
#include "mtms_trial_interfaces/action/perform_trial.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"

#include "pipeline_interfaces/msg/timing_error.hpp"
#include "pipeline_interfaces/msg/timing_latency.hpp"
#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/decision_info.hpp"

#include "project_interfaces/msg/decider_list.hpp"
#include "project_interfaces/srv/set_decider_module.hpp"
#include "project_interfaces/srv/set_decider_enabled.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_PREVIOUS_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

enum class DeciderState {
  WAITING_FOR_ENABLED,
  READY,
  DROPPED_SAMPLE_THRESHOLD_EXCEEDED,
  MODULE_ERROR
};

struct GoalMetadata {
  mtms_trial_interfaces::msg::Trial trial;

  /* The time of the sample based on which the decision to perform a trial was made. */
  double decision_time;
};


class DeciderWrapper;

class EegDecider : public rclcpp::Node {
public:
  EegDecider();
  ~EegDecider();

  void spin();

private:
  rclcpp_action::Client<mtms_trial_interfaces::action::PerformTrial>::SharedPtr perform_trial_client;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  void publish_healthcheck();

  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);
  void handle_timing_latency(const std::shared_ptr<pipeline_interfaces::msg::TimingLatency> msg);

  std::string goal_id_to_string(const rclcpp_action::GoalUUID &uuid);

  void empty_trial_queue();
  void precompute_trials();
  void update_dropped_sample_count();

  void perform_trial(const mtms_trial_interfaces::msg::Trial& trial, double decision_time);
  void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<mtms_trial_interfaces::action::PerformTrial>> goal_handle, const mtms_trial_interfaces::msg::Trial& trial, double decision_time);
  void trial_performed_callback(const rclcpp_action::ClientGoalHandle<mtms_trial_interfaces::action::PerformTrial>::WrappedResult &result);
  void request_timed_trigger(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request);
  void timed_trigger_callback(rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future);

  void update_eeg_info(const eeg_msgs::msg::PreprocessedSampleMetadata& msg);
  void initialize_module();

  void reset_decider_state();

  void unset_decider_module();

  bool set_decider_enabled(bool enabled);
  void handle_preprocessor_enabled(const std::shared_ptr<std_msgs::msg::Bool> msg);

  void handle_set_decider_enabled(
      const std::shared_ptr<project_interfaces::srv::SetDeciderEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDeciderEnabled::Response> response);

  std::string get_module_name_with_fallback(const std::string module_name);
  bool set_decider_module(const std::string module);
  void handle_set_decider_module(
      const std::shared_ptr<project_interfaces::srv::SetDeciderModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDeciderModule::Response> response);

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  void update_decider_list();

  void check_dropped_samples(double_t sample_time);

  void handle_trigger_from_eeg_device(const double_t trigger_time);

  void process_preprocessed_sample(const std::shared_ptr<eeg_msgs::msg::PreprocessedSample> msg);
  void process_raw_sample(const std::shared_ptr<eeg_msgs::msg::Sample> msg);

  void log_trial(const mtms_trial_interfaces::msg::Trial& trial, size_t num_of_remaining_trials);

  std::tuple<bool, double, uint16_t> consume_next_event(double_t current_time);
  void pop_event();

  /* File-system related functions */
  bool change_working_directory(const std::string path);
  std::vector<std::string> list_python_modules_in_working_directory();

  void update_inotify_watch();
  void inotify_timer_callback();

  rclcpp::Logger logger;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr preprocessor_enabled_subscriber;

  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;

  rclcpp::Subscription<eeg_msgs::msg::PreprocessedSample>::SharedPtr preprocessed_eeg_subscriber;
  rclcpp::Subscription<eeg_msgs::msg::Sample>::SharedPtr raw_eeg_subscriber;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Publisher<project_interfaces::msg::DeciderList>::SharedPtr decider_list_publisher;

  rclcpp::Service<project_interfaces::srv::SetDeciderModule>::SharedPtr set_decider_module_service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decider_module_publisher;

  rclcpp::Service<project_interfaces::srv::SetDeciderEnabled>::SharedPtr set_decider_enabled_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr decider_enabled_publisher;

  rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr timed_trigger_client;

  rclcpp::Publisher<pipeline_interfaces::msg::TimingError>::SharedPtr timing_error_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionInfo>::SharedPtr decision_info_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dropped_sample_count_publisher;

  rclcpp::Subscription<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_subscriber;

  bool enabled = false;

  DeciderState decider_state = DeciderState::WAITING_FOR_ENABLED;
  system_interfaces::msg::SessionState session_state;

  bool first_sample_ever = true;
  bool first_sample_of_session = false;

  uint16_t samples_since_last_processing = 0;

  /* Used for keeping track of the time of the previous trigger time to ensure that the minimum pulse
     interval is respected. */
  double_t previous_stimulation_time = UNSET_PREVIOUS_TIME;

  bool reinitialize = true;

  std::string active_project = UNSET_STRING;

  std::string working_directory  = UNSET_STRING;
  bool is_working_directory_set = false;
  std::string module_name = UNSET_STRING;

  std::vector<std::string> modules;

  std::deque<std::pair<double_t, int>> dropped_samples_window;

  uint16_t total_dropped_samples = 0;
  uint16_t dropped_sample_threshold;
  double_t timing_latency_threshold;

  /* Information about the EEG device configuration. */
  uint16_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;
  uint8_t num_of_eeg_channels = UNSET_NUM_OF_CHANNELS;
  uint8_t num_of_emg_channels = UNSET_NUM_OF_CHANNELS;
  double_t sampling_period;

  /* For checking if samples have been dropped, store the time of the previous sample received. */
  double_t previous_time = UNSET_PREVIOUS_TIME;

  RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>> sample_buffer;
  std::vector<pipeline_interfaces::msg::SensoryStimulus> sensory_stimuli;

  std::unique_ptr<DeciderWrapper> decider_wrapper;

  std::queue<std::pair<mtms_trial_interfaces::msg::Trial, double>> trial_queue;
  std::map<std::string, GoalMetadata> goal_to_metadata_map;

  bool performing_trial = false;
  bool processing_timed_trigger = false;

  bool preprocessor_enabled = false;

  double_t timing_latency = 0.0;

  /* ROS parameters */
  double_t minimum_intertrial_interval;

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;

  /* mTMS device */
  bool mtms_device_enabled = false;

  /* Inotify variables */
  rclcpp::TimerBase::SharedPtr inotify_timer;
  int inotify_descriptor;
  std::vector<int> watch_descriptors;
  char inotify_buffer[1024];

  /* Event queue for storing events from the Python module. */
  std::priority_queue<std::pair<double, uint16_t>,
                      std::vector<std::pair<double, uint16_t>>,
                      std::greater<std::pair<double, uint16_t>>> event_queue;

  /* When determining if samples have been dropped by comparing the timestamps of two consecutive
     samples, allow some tolerance to account for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_DECIDER_H
