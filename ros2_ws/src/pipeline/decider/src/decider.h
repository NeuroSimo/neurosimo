//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_DECIDER_H
#define EEG_PROCESSOR_DECIDER_H

#include <deque>
#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"

#include "decider_wrapper.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/empty.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/stream_info.hpp"

#include "pipeline_interfaces/srv/request_timed_trigger.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "pipeline_interfaces/msg/coil_target.hpp"

#include "pipeline_interfaces/msg/timing_error.hpp"
#include "pipeline_interfaces/msg/pipeline_latency.hpp"
#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/decision_trace.hpp"
#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"
#include "pipeline_interfaces/srv/initialize_decider.hpp"
#include "pipeline_interfaces/srv/finalize_decider.hpp"

#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

using StreamInfo = eeg_interfaces::msg::StreamInfo;

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

  bool reset_state();

  void publish_healthcheck();

  void handle_pipeline_latency(const std::shared_ptr<pipeline_interfaces::msg::PipelineLatency> msg);
  void handle_is_coil_at_target(const std::shared_ptr<std_msgs::msg::Bool> msg);

  void request_timed_trigger(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request);
  void timed_trigger_callback(rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future);

  void log_section_header(const std::string& title);
  void publish_python_logs(double sample_time, bool is_initialization);

  void handle_initialize_decider(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeDecider::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeDecider::Response> response);

  void handle_finalize_decider(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizeDecider::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizeDecider::Response> response);

  void handle_pulse_trigger(const double_t pulse_trigger_time);

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

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_subscriber;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_coil_at_target_subscriber;

  rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr timed_trigger_client;

  rclcpp::Publisher<pipeline_interfaces::msg::TimingError>::SharedPtr timing_error_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::PipelineLatency>::SharedPtr pipeline_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::CoilTarget>::SharedPtr coil_target_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  rclcpp::Subscription<pipeline_interfaces::msg::PipelineLatency>::SharedPtr pipeline_latency_subscriber;

  /* Service server for initialization */
  rclcpp::Service<pipeline_interfaces::srv::InitializeDecider>::SharedPtr initialize_service_server;

  /* Service server for finalization */
  rclcpp::Service<pipeline_interfaces::srv::FinalizeDecider>::SharedPtr finalize_service_server;

  /* Initialization state */
  bool is_initialized = false;
  bool is_enabled = false;
  std::string initialized_project_name;
  std::string initialized_module_filename;
  std::filesystem::path initialized_working_directory;

  /* Session and decision tracking */
  std::array<uint8_t, 16> session_id = {};
  uint64_t decision_id = 0;

  double_t next_periodic_processing_time = UNSET_TIME;

  /* Used for keeping track of the time of the previous trigger time to ensure that the minimum pulse
     interval is respected. */
  double_t previous_stimulation_time = UNSET_TIME;

  /* Used for pulse lockout: tracks when the lockout period ends (time when processing can resume). */
  double_t pulse_lockout_end_time = UNSET_TIME;

  double_t pipeline_latency_threshold;

  StreamInfo stream_info;

  RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>> sample_buffer;
  std::vector<pipeline_interfaces::msg::SensoryStimulus> sensory_stimuli;

  std::unique_ptr<DeciderWrapper> decider_wrapper;

  bool is_processing_timed_trigger = false;

  double_t pipeline_latency = 0.0;

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

  /* Deferred processing queue for handling look-ahead samples. */
  std::priority_queue<DeferredProcessingRequest,
                      std::vector<DeferredProcessingRequest>,
                      std::greater<DeferredProcessingRequest>> deferred_processing_queue;

  /* When determining if a sample is ready to be processed, allow some tolerance to account
     for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_DECIDER_H
