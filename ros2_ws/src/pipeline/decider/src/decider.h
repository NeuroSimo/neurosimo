//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_DECIDER_H
#define EEG_PROCESSOR_DECIDER_H

#include <deque>
#include <cmath>
#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "inotify_utils/inotify_watcher.h"
#include "module_utils/module_manager.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/srv/request_timed_trigger.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "pipeline_interfaces/msg/coil_target.hpp"

#include "pipeline_interfaces/msg/timing_error.hpp"
#include "pipeline_interfaces/msg/timing_latency.hpp"
#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/decision_info.hpp"
#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"

#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

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


struct DeferredProcessingRequest {
  /* The time when processing should actually occur (after look-ahead samples have arrived). */
  double_t scheduled_time;
  
  /* The sample that triggered the processing request. */
  std::shared_ptr<eeg_interfaces::msg::Sample> triggering_sample;

  /* Whether this has an event. */
  bool has_event;
  
  /* Event type if has_event is true. */
  std::string event_type;
  
  /* Comparison operator for priority queue (min-heap by scheduled_time). */
  bool operator>(const DeferredProcessingRequest& other) const {
    return scheduled_time > other.scheduled_time;
  }
};


class DeciderWrapper;

class EegDecider : public rclcpp::Node {
public:
  EegDecider();

  void spin();

private:
  rclcpp::CallbackGroup::SharedPtr callback_group;

  void publish_healthcheck();

  void handle_session_start(const eeg_interfaces::msg::SessionMetadata& metadata);
  void handle_session_end();

  void handle_timing_latency(const std::shared_ptr<pipeline_interfaces::msg::TimingLatency> msg);
  void handle_is_coil_at_target(const std::shared_ptr<std_msgs::msg::Bool> msg);

  void request_timed_trigger(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request);
  void timed_trigger_callback(rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future);

  bool initialize_module();
  void log_section_header(const std::string& title);
  void publish_python_logs(double sample_time, bool is_initialization);

  void reset_decider_state();

  bool set_decider_enabled(bool enabled);
  void handle_preprocessor_enabled(const std::shared_ptr<std_msgs::msg::Bool> msg);

  void handle_pulse_delivered(const double_t pulse_delivered_time);

  void process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  std::tuple<bool, double, std::string> consume_next_event(double_t current_time);
  void pop_event();

  bool is_sample_window_valid() const;
  void enqueue_deferred_request(const std::shared_ptr<eeg_interfaces::msg::Sample> msg, double_t sample_time, bool has_event, const std::string& event_type);
  void process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time);
  void process_ready_deferred_requests(double_t current_sample_time);

  rclcpp::Logger logger;

  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr preprocessor_enabled_subscriber;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_subscriber;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_coil_at_target_subscriber;

  rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr timed_trigger_client;

  rclcpp::Publisher<pipeline_interfaces::msg::TimingError>::SharedPtr timing_error_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionInfo>::SharedPtr decision_info_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::CoilTarget>::SharedPtr coil_target_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  rclcpp::Subscription<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_subscriber;

  /* Module manager for handling module selection and project changes */
  std::unique_ptr<module_utils::ModuleManager> module_manager;

  bool is_session_ongoing = false;

  double_t next_periodic_processing_time = UNSET_TIME;

  /* Used for keeping track of the time of the previous trigger time to ensure that the minimum pulse
     interval is respected. */
  double_t previous_stimulation_time = UNSET_TIME;

  /* Used for pulse lockout: tracks when the lockout period ends (time when processing can resume). */
  double_t pulse_lockout_end_time = UNSET_TIME;

  double_t timing_latency_threshold;

  SessionMetadataState session_metadata;

  RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>> sample_buffer;
  std::vector<pipeline_interfaces::msg::SensoryStimulus> sensory_stimuli;

  std::unique_ptr<DeciderWrapper> decider_wrapper;

  bool is_processing_timed_trigger = false;

  bool is_preprocessor_enabled = false;

  double_t timing_latency = 0.0;

  /* State variables */
  bool error_occurred = false;

  /* Neuronavigation */
  bool is_coil_at_target = false;

  /* ROS parameters */
  double_t minimum_intertrial_interval;

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;

  /* Event queue for storing events from the Python module. */
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>,
                      std::greater<std::pair<double, std::string>>> event_queue;
  std::mutex event_queue_mutex;

  /* Deferred processing queue for handling look-ahead samples. */
  std::priority_queue<DeferredProcessingRequest,
                      std::vector<DeferredProcessingRequest>,
                      std::greater<DeferredProcessingRequest>> deferred_processing_queue;

  /* When determining if a sample is ready to be processed, allow some tolerance to account
     for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_DECIDER_H
