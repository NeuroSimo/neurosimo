#include <chrono>
#include <filesystem>
#include <signal.h>
#include <execinfo.h>

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "decider_wrapper.h"
#include "decider.h"

#include "realtime_utils/utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string HEALTHCHECK_TOPIC = "/eeg/decider/healthcheck";
const std::string IS_COIL_AT_TARGET_TOPIC = "/neuronavigation/coil_at_target";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_DECIDER_NAME = "example";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

/* Signal handler for crash debugging */
void crash_handler(int sig) {
  void *array[10];
  size_t size;

  // Get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // Print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

/* XXX: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 20ms;


EegDecider::EegDecider() : Node("decider"), logger(rclcpp::get_logger("decider")) {
  /* Read ROS parameter: Minimum interval between consecutive pulses (in seconds). */
  auto minimum_intertrial_interval_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  minimum_intertrial_interval_descriptor.description = "The minimum interval between consecutive pulses (in seconds)";
  minimum_intertrial_interval_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  /* XXX: Have to provide 0.0 as a default value because the parameter server does not interpret NULL correctly
          when the parameter is a double. */
  this->declare_parameter("minimum-intertrial-interval", 0.0, minimum_intertrial_interval_descriptor);
  this->get_parameter("minimum-intertrial-interval", this->minimum_intertrial_interval);

  /* Read ROS parameter: dropped sample threshold */
  auto dropped_sample_threshold_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  dropped_sample_threshold_descriptor.description = "The threshold for the number of dropped samples before the decider enters an error state";
  dropped_sample_threshold_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("dropped-sample-threshold", 4, dropped_sample_threshold_descriptor);
  this->get_parameter("dropped-sample-threshold", this->dropped_sample_threshold);

  /* Read ROS parameter: timing latency threshold */
  auto timing_latency_threshold_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  timing_latency_threshold_descriptor.description = "The threshold for the timing latency (in seconds) before stimulation is prevented";
  this->declare_parameter("timing-latency-threshold", 0.005, timing_latency_threshold_descriptor);
  this->get_parameter("timing-latency-threshold", this->timing_latency_threshold);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Minimum pulse interval: %.1f (s)", this->minimum_intertrial_interval);
  RCLCPP_INFO(this->get_logger(), "  Dropped samples per second threshold: %d", this->dropped_sample_threshold);
  RCLCPP_INFO(this->get_logger(), "  Timing latency threshold: %.1f (ms)", 1000 * this->timing_latency_threshold);

  /* Validate the minimum pulse interval. */
  if (this->minimum_intertrial_interval <= 0) {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_ERROR(this->get_logger(), "Invalid minimum pulse interval: %.1f (s)", this->minimum_intertrial_interval);
    exit(1);
  }

  if (this->minimum_intertrial_interval < 0.5) {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_WARN(this->get_logger(), "Note: Minimum pulse interval is very low: %.1f (s)", this->minimum_intertrial_interval);
  }

  /* Publisher for healthcheck. */
  this->healthcheck_publisher = this->create_publisher<system_interfaces::msg::Healthcheck>(HEALTHCHECK_TOPIC, 10);

  /* Subscriber for session. */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.deadline_callback = [this]([[maybe_unused]] rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Session not received within deadline.");
  };

  this->session_subscriber = this->create_subscription<system_interfaces::msg::Session>(
    "/system/session",
    qos_session,
    std::bind(&EegDecider::handle_session, this, _1),
    subscription_options);

  /* EEG subscriber will be created by handle_preprocessor_enabled based on preprocessor state. */

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegDecider::handle_set_active_project, this, _1));

  /* Subscriber for is coil at target. */
  this->is_coil_at_target_subscriber = create_subscription<std_msgs::msg::Bool>(
    IS_COIL_AT_TARGET_TOPIC,
    qos_persist_latest,
    std::bind(&EegDecider::handle_is_coil_at_target, this, _1));

  /* Publisher for listing deciders. */
  this->decider_list_publisher = this->create_publisher<project_interfaces::msg::DeciderList>(
    "/pipeline/decider/list",
    qos_persist_latest);

  /* Subscriber for preprocessor enabled message. */
  this->preprocessor_enabled_subscriber = this->create_subscription<std_msgs::msg::Bool>(
    "/pipeline/preprocessor/enabled",
    qos_persist_latest,
    std::bind(&EegDecider::handle_preprocessor_enabled, this, _1));

  /* Service for changing decider module. */
  this->set_decider_module_service = this->create_service<project_interfaces::srv::SetDeciderModule>(
    "/pipeline/decider/module/set",
    std::bind(&EegDecider::handle_set_decider_module, this, _1, _2));

  /* Publisher for decider module. */
  this->decider_module_publisher = this->create_publisher<std_msgs::msg::String>(
    "/pipeline/decider/module",
    qos_persist_latest);

  /* Service for enabling and disabling decider. */
  this->set_decider_enabled_service = this->create_service<std_srvs::srv::SetBool>(
    "/pipeline/decider/enabled/set",
    std::bind(&EegDecider::handle_set_decider_enabled, this, _1, _2));

  /* Publisher for decider enabled message. */
  this->decider_enabled_publisher = this->create_publisher<std_msgs::msg::Bool>(
    "/pipeline/decider/enabled",
    qos_persist_latest);

  /* Publisher for timing error. */
  this->timing_error_publisher = this->create_publisher<pipeline_interfaces::msg::TimingError>(
    "/pipeline/timing/error",
    10);

  /* Publisher for decision info. */
  this->decision_info_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionInfo>(
    "/pipeline/decision_info",
    10);

  /* Publisher for dropped sample count. */
  this->dropped_sample_count_publisher = this->create_publisher<std_msgs::msg::Int32>(
    "/pipeline/dropped_samples",
    10);

  /* Subscriber for timing latency. */
  this->timing_latency_subscriber = this->create_subscription<pipeline_interfaces::msg::TimingLatency>(
    "/pipeline/timing/latency",
    10,
    std::bind(&EegDecider::handle_timing_latency, this, _1));

  /* Publisher for timing latency. */
  this->timing_latency_publisher = this->create_publisher<pipeline_interfaces::msg::TimingLatency>(
    "/pipeline/timing/latency",
    10);

  /* Publisher for sensory stimulus. */

  // Messages can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all = rclcpp::QoS(rclcpp::KeepAll())
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->sensory_stimulus_publisher = this->create_publisher<pipeline_interfaces::msg::SensoryStimulus>(
    "/pipeline/sensory_stimulus",
    qos_keep_all);

  /* Publisher for coil target. */
  this->coil_target_publisher = this->create_publisher<pipeline_interfaces::msg::CoilTarget>(
    "/neuronavigation/coil_target",
    10);

  /* Publisher for Python logs from decider. */
  // Logs can be sent in bursts so keep all messages in the queue.
  // Use TRANSIENT_LOCAL durability so late-joining subscribers get buffered messages.
  auto qos_keep_all_logs = rclcpp::QoS(rclcpp::KeepAll())
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->python_log_publisher = this->create_publisher<pipeline_interfaces::msg::LogMessages>(
    "/pipeline/decider/log",
    qos_keep_all_logs);

  /* Publisher for pulse events. */
  this->pulse_event_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/pipeline/pulse_events",
    100);

  /* Service client for timed trigger. */
  this->timed_trigger_client = this->create_client<pipeline_interfaces::srv::RequestTimedTrigger>("/pipeline/timed_trigger");

  while (!timed_trigger_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /pipeline/timed_trigger not available, waiting...");
  }

  /* Initialize variables. */
  this->decider_wrapper = std::make_unique<DeciderWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>();

  /* Initialize inotify. */
  this->inotify_descriptor = inotify_init();
  if (this->inotify_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error initializing inotify");
      exit(1);
  }

  /* Set the inotify descriptor to non-blocking. */
  int flags = fcntl(inotify_descriptor, F_GETFL, 0);
  fcntl(inotify_descriptor, F_SETFL, flags | O_NONBLOCK);

  /* Create a timer callback to poll inotify. */
  this->inotify_timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&EegDecider::inotify_timer_callback, this));

  this->healthcheck_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EegDecider::publish_healthcheck, this));
}

EegDecider::~EegDecider() {
  for (int wd : watch_descriptors) {
    inotify_rm_watch(inotify_descriptor, wd);
  }
  close(inotify_descriptor);
}

void EegDecider::publish_healthcheck() {
  auto healthcheck = system_interfaces::msg::Healthcheck();

  switch (this->decider_state) {
    case DeciderState::WAITING_FOR_ENABLED:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
      healthcheck.status_message = "Decider not enabled";
      healthcheck.actionable_message = "Please enable the decider.";
      break;

    case DeciderState::READY:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
      healthcheck.status_message = "Ready";
      healthcheck.actionable_message = "Ready";
      break;

    case DeciderState::DROPPED_SAMPLE_THRESHOLD_EXCEEDED:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Dropped sample threshold exceeded";
      healthcheck.actionable_message = "Dropped sample threshold (" + std::to_string(this->dropped_sample_threshold) + " per second) exceeded.";
      break;

    case DeciderState::MODULE_ERROR:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Module error";
      healthcheck.actionable_message = "Decider has encountered an error.";
      break;
  }
  this->healthcheck_publisher->publish(healthcheck);
}

void EegDecider::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  bool state_changed = this->session_state.value != msg->state.value;
  this->session_state = msg->state;

  if (state_changed && this->session_state.value == system_interfaces::msg::SessionState::STOPPED) {
    this->first_sample_of_session = true;
    this->total_dropped_samples = 0;
    update_dropped_sample_count();

    this->reinitialize = true;
    this->previous_stimulation_time = UNSET_PREVIOUS_TIME;
    this->pulse_lockout_end_time = UNSET_PREVIOUS_TIME;

    /* Clear deferred processing queue when session stops. */
    while (!this->deferred_processing_queue.empty()) {
      this->deferred_processing_queue.pop();
    }

    /* Reset the decider state when the session is stopped. */
    reset_decider_state();
  }
}

void EegDecider::handle_timing_latency(const std::shared_ptr<pipeline_interfaces::msg::TimingLatency> msg) {
  this->timing_latency = msg->latency;
}

void EegDecider::update_eeg_info(const eeg_interfaces::msg::SampleMetadata& msg) {
  this->sampling_frequency = msg.sampling_frequency;
  this->num_of_eeg_channels = msg.num_of_eeg_channels;
  this->num_of_emg_channels = msg.num_of_emg_channels;

  this->sampling_period = 1.0 / this->sampling_frequency;
}

void EegDecider::log_section_header(const std::string& title) {
  std::wstring underline_str(title.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), title.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");
}

void EegDecider::publish_python_logs(double sample_time, bool is_initialization) {
  auto logs = this->decider_wrapper->get_and_clear_logs();
  
  if (logs.empty()) {
    return;
  }
  
  // Create a single batched message containing all logs
  auto batch_msg = pipeline_interfaces::msg::LogMessages();
  
  for (const auto& log_entry : logs) {
    // Create individual log message
    auto log_msg = pipeline_interfaces::msg::LogMessage();
    log_msg.message = log_entry.message;
    log_msg.sample_time = sample_time;
    log_msg.level = static_cast<uint8_t>(log_entry.level);
    log_msg.is_initialization = is_initialization;
    
    batch_msg.messages.push_back(log_msg);
    
    // Log to console with appropriate level
    if (log_entry.level == LogLevel::ERROR) {
      RCLCPP_ERROR(this->get_logger(), "[Python Error]: %s", log_entry.message.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "[Python]: %s", log_entry.message.c_str());
    }
  }
  
  // Publish all logs in a single batched message
  this->python_log_publisher->publish(batch_msg);
}

void EegDecider::initialize_module() {
  if (this->working_directory == UNSET_STRING ||
      this->module_name == UNSET_STRING) {

    RCLCPP_INFO(this->get_logger(), "Not initializing, decider module unset.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "");

  log_section_header("Loading decider: " + this->module_name);

  /* Clear the event queue before initializing. */
  while (!this->event_queue.empty()) {
    this->event_queue.pop();
  }

  this->decider_wrapper->initialize_module(
    PROJECTS_DIRECTORY,
    this->working_directory,
    this->module_name,
    this->num_of_eeg_channels,
    this->num_of_emg_channels,
    this->sampling_frequency,
    this->sensory_stimuli,
    this->event_queue,
    this->event_queue_mutex);

  /* Publish initialization logs from Python constructor */
  publish_python_logs(0.0, true);

  if (this->decider_wrapper->get_state() != WrapperState::READY) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load.");
    return;
  }

  size_t buffer_size = this->decider_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %s%d%s Hz", bold_on.c_str(), this->sampling_frequency, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %s%d%s", bold_on.c_str(), this->num_of_eeg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %s%d%s", bold_on.c_str(), this->num_of_emg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  /* Perform warm-up if requested by the Python module */
  this->decider_wrapper->warm_up();

  /* Send the initial sensory stimuli to the presenter. */
  for (auto& sensory_stimulus : this->sensory_stimuli) {
    this->sensory_stimulus_publisher->publish(sensory_stimulus);
  }
  this->sensory_stimuli.clear();
}

std::tuple<bool, double, std::string> EegDecider::consume_next_event(double_t current_time) {
  std::lock_guard<std::mutex> lock(this->event_queue_mutex);

  if (this->event_queue.empty()) {
    return std::make_tuple(false, 0.0, "");
  }

  double_t event_time;
  std::string event_type;

  /* Pop events until the event time is within the tolerance. */
  while (!this->event_queue.empty()) {
    auto event = this->event_queue.top();
    event_time = event.first;
    event_type = event.second;

    if (event_time - this->TOLERANCE_S >= current_time - this->sampling_period) {
      break;
    }
    this->event_queue.pop();
  }

  /* Check if we popped all events */
  if (this->event_queue.empty()) {
    return std::make_tuple(false, 0.0, "");
  }

  /* If the event time is too far in the future, return false. */
  if (event_time > current_time + this->TOLERANCE_S) {
    return std::make_tuple(false, 0.0, "");
  }
  return std::make_tuple(true, event_time, event_type);
}

void EegDecider::pop_event() {
  std::lock_guard<std::mutex> lock(this->event_queue_mutex);
  if (!this->event_queue.empty()) {
    this->event_queue.pop();
  }
}

void EegDecider::process_ready_deferred_requests(double_t current_sample_time) {
  /* Process any deferred requests that are now ready (have enough look-ahead samples). */
  while (!this->deferred_processing_queue.empty()) {
    const auto& next_request = this->deferred_processing_queue.top();

    /* Check if our current sample time is within the tolerance of the processing time of the next request. */
    if (current_sample_time >= next_request.scheduled_time - this->TOLERANCE_S) {
      /* Process this deferred request. */
      process_deferred_request(next_request, current_sample_time);
      this->deferred_processing_queue.pop();
    } else {
      /* The next request is not ready yet. */
      break;
    }
  }
}

void EegDecider::process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time) {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  double_t sample_time = request.triggering_sample->time;
  bool pulse_delivered = request.pulse_delivered;
  bool has_event = request.has_event;
  std::string event_type = request.event_type;

  /* Process the sample. */
  auto [success, timed_trigger, coil_target] = this->decider_wrapper->process(
    this->sensory_stimuli,
    this->sample_buffer,
    sample_time,
    pulse_delivered,
    has_event,
    event_type,
    this->event_queue,
    this->event_queue_mutex,
    this->is_coil_at_target);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(sample_time, false);

  /* Log and return early if the Python call failed. */
  if (!success) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(),
                          *this->get_clock(),
                          1000,
                          "Python call failed, not processing EEG sample at time %.3f (s).",
                          sample_time);
    return;
  }

  /* Calculate the total latency of the decider. */
  auto end_time = std::chrono::high_resolution_clock::now();
  double_t decider_processing_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Check if the decision is positive. */
  bool is_decision_positive = (timed_trigger != nullptr);

  /* If the decision is negative, publish the decision info and return. */
  if (!is_decision_positive) {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time sample_time_rcl(request.triggering_sample->metadata.system_time);
    double_t total_latency = now.seconds() - sample_time_rcl.seconds();

    auto decision_info = pipeline_interfaces::msg::DecisionInfo();
    decision_info.stimulate = false;
    decision_info.feasible = false;
    decision_info.decision_time = sample_time;
    decision_info.decider_latency = decider_processing_duration;
    decision_info.preprocessor_latency = request.triggering_sample->preprocessing_duration;
    decision_info.total_latency = total_latency;

    this->decision_info_publisher->publish(decision_info);
    return;
  }

  /* Check that timing latency is within threshold. */
  if (this->timing_latency > this->timing_latency_threshold) {
    RCLCPP_ERROR(this->get_logger(), "Timing latency (%.1f ms) exceeds threshold (%.1f ms), ignoring stimulation request.", this->timing_latency * 1000, this->timing_latency_threshold * 1000);
    return;
  }

  /* Check that the minimum intertrial interval is respected. */
  auto time_since_previous_trial = timed_trigger->time - this->previous_stimulation_time;
  auto has_minimum_intertrial_interval_passed = std::isnan(this->previous_stimulation_time) ||
                                                time_since_previous_trial >= this->minimum_intertrial_interval;

  if (!has_minimum_intertrial_interval_passed) {
    RCLCPP_ERROR(this->get_logger(), "Stimulation requested but minimum intertrial interval (%.1f s) not respected (time since previous stimulation: %.3f s), ignoring request.",
                  this->minimum_intertrial_interval,
                  time_since_previous_trial);
    return;
  }

  /* Send timed trigger if requested. */
  double_t trigger_time = timed_trigger->time;

  auto request_msg = std::make_shared<pipeline_interfaces::srv::RequestTimedTrigger::Request>();
  request_msg->timed_trigger = *timed_trigger;
  request_msg->decision_time = sample_time;
  request_msg->system_time_for_sample = request.triggering_sample->metadata.system_time;
  request_msg->preprocessor_latency = request.triggering_sample->preprocessing_duration;
  request_msg->decider_latency = decider_processing_duration;

  this->request_timed_trigger(request_msg);

  RCLCPP_INFO(this->get_logger(), "Timing trigger at time %.3f (s).", timed_trigger->time);

  /* Update the previous stimulation time. */
  this->previous_stimulation_time = timed_trigger->time;
  
  /* Set the pulse lockout end time. */
  this->pulse_lockout_end_time = timed_trigger->time + this->decider_wrapper->get_pulse_lockout_duration();

  /* Publish sensory stimuli if the vector is not empty. */
  if (!this->sensory_stimuli.empty()) {
    auto sensory_stimulus_count = this->sensory_stimuli.size();
    RCLCPP_INFO(this->get_logger(), "Requesting %zu sensory stimuli at time %.3f (s).", sensory_stimulus_count, sample_time);

    for (const auto& sensory_stimulus : this->sensory_stimuli) {
      this->sensory_stimulus_publisher->publish(sensory_stimulus);
    }
    this->sensory_stimuli.clear();
  }
  
  /* Publish coil target if it is set. */
  if (!coil_target.empty()) {
    auto coil_target_msg = pipeline_interfaces::msg::CoilTarget();
    coil_target_msg.target_name = coil_target;
    RCLCPP_INFO(this->get_logger(), "Sending coil target %s to neuronavigation at time %.3f (s).", coil_target_msg.target_name.c_str(), sample_time);
    this->coil_target_publisher->publish(coil_target_msg);
  }
}

/* Service clients */

void EegDecider::request_timed_trigger(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request) {
  using ServiceResponseFuture = rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    this->timed_trigger_callback(future);
  };

  auto future_result = this->timed_trigger_client->async_send_request(request, response_received_callback);
}

void EegDecider::timed_trigger_callback(rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future) {
  auto result = future.get().second;
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send timed trigger.");
  }
}

/* Initialization and reset functions */

void EegDecider::reset_decider_state() {
  this->decider_state = this->enabled ? DeciderState::READY : DeciderState::WAITING_FOR_ENABLED;
  this->next_periodic_processing_time = this->decider_wrapper->get_first_periodic_processing_at();
}


void EegDecider::handle_preprocessor_enabled(const std::shared_ptr<std_msgs::msg::Bool> msg) {
  this->is_preprocessor_enabled = msg->data;

  /* Destroy existing subscriber. */
  this->eeg_subscriber.reset();

  /* Create the subscriber based on preprocessor state. */
  std::string topic = this->is_preprocessor_enabled ? EEG_PREPROCESSED_TOPIC : EEG_RAW_TOPIC;
  this->eeg_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    topic,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegDecider::process_sample, this, _1));

  RCLCPP_DEBUG(this->get_logger(), "Reading %s EEG data.", this->is_preprocessor_enabled ? "preprocessed" : "raw");
}

/* Listing and setting EEG deciders. */
bool EegDecider::set_decider_enabled(bool enabled) {

  /* Only allow enabling the decider if a module is set. */
  if (enabled && this->module_name == UNSET_STRING) {
    RCLCPP_WARN(this->get_logger(), "Cannot enable decider, no module set.");

    return false;
  }

  /* Update global state variable. */
  this->enabled = enabled;

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = enabled;

  this->decider_enabled_publisher->publish(msg);

  /* Re-initialize the module each time the decider is enabled. */
  if (enabled) {
    this->reinitialize = true;
  }

  /* Reset decider state. */
  reset_decider_state();

  return true;
}

void EegDecider::handle_set_decider_enabled(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  response->success = set_decider_enabled(request->data);
  response->message = "";
}

void EegDecider::unset_decider_module() {
  this->module_name = UNSET_STRING;

  RCLCPP_INFO(this->get_logger(), "Decider module unset.");

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->decider_module_publisher->publish(msg);

  /* Reset the Python module state. */
  this->decider_wrapper->reset_module_state();

  /* Disable the decider. */
  set_decider_enabled(false);
}

std::string EegDecider::get_module_name_with_fallback(const std::string module_name) {
  if (std::find(this->modules.begin(), this->modules.end(), module_name) != this->modules.end()) {
    return module_name;
  }
  if (std::find(this->modules.begin(), this->modules.end(), DEFAULT_DECIDER_NAME) != this->modules.end()) {
    RCLCPP_WARN(this->get_logger(), "Module %s not found, setting to default", module_name.c_str());
    return DEFAULT_DECIDER_NAME;
  }
  if (!this->modules.empty()) {
    RCLCPP_WARN(this->get_logger(), "Module %s not found, setting to first module on the list: %s", module_name.c_str(), this->modules[0].c_str());
    return this->modules[0];
  }
  RCLCPP_WARN(this->get_logger(), "No deciders found in project: %s%s%s.", bold_on.c_str(), this->active_project.c_str(), bold_off.c_str());
  return UNSET_STRING;
}

bool EegDecider::set_decider_module(const std::string module_name) {
  this->module_name = get_module_name_with_fallback(module_name);

  if (this->module_name == UNSET_STRING) {
    RCLCPP_ERROR(this->get_logger(), "No decider module set.");
    this->unset_decider_module();

    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Decider set to: %s%s%s.", bold_on.c_str(), this->module_name.c_str(), bold_off.c_str());

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->decider_module_publisher->publish(msg);

  /* Re-initialize the module each time the module is reset. */
  this->reinitialize = true;

  return true;
}

void EegDecider::handle_set_decider_module(
      const std::shared_ptr<project_interfaces::srv::SetDeciderModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDeciderModule::Response> response) {

  response->success = set_decider_module(request->module);
}

void EegDecider::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;
  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(), "Project set to: %s%s%s.", bold_on.c_str(), this->active_project.c_str(), bold_off.c_str());

  this->is_working_directory_set = change_working_directory(PROJECTS_DIRECTORY + "/" + this->active_project + "/decider");
  update_decider_list();

  update_inotify_watch();
}

void EegDecider::handle_is_coil_at_target(const std::shared_ptr<std_msgs::msg::Bool> msg) {
  this->is_coil_at_target = msg->data;
}

/* File-system related functions */

bool EegDecider::change_working_directory(const std::string path) {
  this->working_directory = path;

  /* Check that the directory exists and follow symlinks. */
  std::error_code ec;
  if (!std::filesystem::exists(this->working_directory, ec) ||
      !std::filesystem::is_directory(this->working_directory, ec)) {
    RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s.", path.c_str());
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", ec.message().c_str());
    }
    return false;
  }

  /* If it's a symlink, resolve it and check the target. */
  if (std::filesystem::is_symlink(this->working_directory, ec)) {
    auto resolved_path = std::filesystem::canonical(this->working_directory, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to resolve symlink %s: %s", path.c_str(), ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Resolved symlink %s to %s", path.c_str(), resolved_path.c_str());

    if (!std::filesystem::is_directory(resolved_path, ec)) {
      RCLCPP_ERROR(this->get_logger(), "Symlink target is not a directory: %s -> %s", path.c_str(), resolved_path.c_str());
      return false;
    }
  }

  /* Change the working directory to the project directory. */
  if (chdir(this->working_directory.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s.", this->working_directory.c_str());
    return false;
  }

  return true;
}

std::vector<std::string> EegDecider::list_python_modules_in_working_directory() {
  std::vector<std::string> modules;

  /* List all .py files in the working directory. */
  std::error_code ec;
  try {
    for (const auto &entry : std::filesystem::directory_iterator(this->working_directory, ec)) {
      if (ec) {
        RCLCPP_WARN(this->get_logger(), "Error accessing directory %s: %s", this->working_directory.c_str(), ec.message().c_str());
        return modules;  // Return empty vector
      }
      
      std::error_code entry_ec;
      if (entry.is_regular_file(entry_ec) && !entry_ec && entry.path().extension() == ".py") {
        modules.push_back(entry.path().stem().string());
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(this->get_logger(), "Filesystem error while listing modules in %s: %s", this->working_directory.c_str(), e.what());
    return modules;  // Return whatever we managed to collect
  }

  /* Sort modules */
  std::sort(modules.begin(), modules.end());

  return modules;
}

void EegDecider::update_inotify_watch() {
  /* Remove all old watches. */
  for (int wd : watch_descriptors) {
    inotify_rm_watch(inotify_descriptor, wd);
  }
  watch_descriptors.clear();

  /* Collect working directory and its subdirectories. */
  std::vector<std::filesystem::path> directories;

  /* Check if working directory exists and is accessible. */
  std::error_code ec;
  if (!std::filesystem::exists(this->working_directory, ec) ||
      !std::filesystem::is_directory(this->working_directory, ec)) {
    RCLCPP_ERROR(this->get_logger(), "Working directory does not exist or is not a directory: %s", this->working_directory.c_str());
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", ec.message().c_str());
    }
    return;
  }

  directories.push_back(std::filesystem::path(this->working_directory));

  /* Use error code version to handle symlinks and permission issues gracefully. */
  try {
    for (const auto& entry : std::filesystem::recursive_directory_iterator(this->working_directory, ec)) {
      if (ec) {
        RCLCPP_WARN(this->get_logger(), "Error iterating directory %s: %s", this->working_directory.c_str(), ec.message().c_str());
        break;
      }

      std::error_code entry_ec;
      if (std::filesystem::is_directory(entry, entry_ec) && !entry_ec) {
        directories.push_back(entry.path());
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(this->get_logger(), "Filesystem error while iterating %s: %s", this->working_directory.c_str(), e.what());
  }

  /* Add watches for all collected directories. */
  for (const auto& dir : directories) {
    int wd = inotify_add_watch(inotify_descriptor, dir.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE | IN_MOVE);
    if (wd == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error adding watch for: %s", dir.c_str());
    } else {
      watch_descriptors.push_back(wd);
      RCLCPP_DEBUG(this->get_logger(), "Added watch for: %s", dir.c_str());
    }
  }
}

void EegDecider::inotify_timer_callback() {
  int length = read(inotify_descriptor, inotify_buffer, 1024);

  if (length < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      /* No events, return early. */
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error reading inotify");
      return;
    }
  }

  int i = 0;
  while (i < length) {
    struct inotify_event *event = (struct inotify_event *)&inotify_buffer[i];
    if (event->len) {
      std::string event_name = event->name;

      std::vector<std::string> internal_imports = this->decider_wrapper->get_internal_imports();
      bool is_internal_import = std::find(internal_imports.begin(), internal_imports.end(), event_name) != internal_imports.end();

      /* Check if the file ends with .py. */
      bool is_python_file = false;
      if (event_name.length() >= 3) {
        is_python_file = (event_name.substr(event_name.length() - 3) == ".py");
      }

      if ((event->mask & IN_MODIFY) && (is_internal_import || event_name == this->module_name + ".py")) {
        RCLCPP_INFO(this->get_logger(), "Module %s was modified, re-loading.", event_name.c_str());
        this->reinitialize = true;
      }
      if ((event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) && is_python_file) {
        RCLCPP_INFO(this->get_logger(), "File %s created, deleted, or moved, updating decider list.", event_name.c_str());
        this->update_decider_list();
      }
    }
    i += sizeof(struct inotify_event) + event->len;
  }
}

void EegDecider::update_decider_list() {
  if (is_working_directory_set) {
    this->modules = this->list_python_modules_in_working_directory();
  } else {
    this->modules.clear();
  }
  auto msg = project_interfaces::msg::DeciderList();
  msg.scripts = this->modules;

  this->decider_list_publisher->publish(msg);
}

/* EEG functions */
void EegDecider::update_dropped_sample_count() {
  auto msg = std_msgs::msg::Int32();
  msg.data = this->total_dropped_samples;

  this->dropped_sample_count_publisher->publish(msg);
}

void EegDecider::check_dropped_samples(double_t sample_time) {
  if (this->sampling_frequency == UNSET_SAMPLING_FREQUENCY) {
    RCLCPP_WARN(this->get_logger(), "Sampling frequency not received, cannot check for dropped samples.");
    return;
  }

  if (this->previous_time) {
    auto time_diff = sample_time - this->previous_time;
    auto threshold = this->sampling_period + this->TOLERANCE_S;

    /* Calculate number of dropped samples. */
    int dropped_samples = std::max(0, static_cast<int>(std::round((time_diff - this->sampling_period) * this->sampling_frequency)));

    if (time_diff > threshold) {
      /* Accumulate dropped samples. */
      this->total_dropped_samples += dropped_samples;

      /* Sliding window: add dropped samples and clean up entries older than 1 second. */
      this->dropped_samples_window.push_back({sample_time, dropped_samples});
      while (!this->dropped_samples_window.empty() &&
             (sample_time - this->dropped_samples_window.front().first > 1.0)) {
        this->dropped_samples_window.pop_front();
      }

      /* Sum of dropped samples in the last second. */
      int recent_dropped_samples = 0;
      for (const auto& entry : this->dropped_samples_window) {
        recent_dropped_samples += entry.second;
      }

      if (recent_dropped_samples > this->dropped_sample_threshold) {
        this->decider_state = DeciderState::DROPPED_SAMPLE_THRESHOLD_EXCEEDED;
        RCLCPP_ERROR(this->get_logger(),
            "Dropped samples exceeded threshold! Recent dropped samples: %d, Threshold: %d",
            recent_dropped_samples, this->dropped_sample_threshold);
      } else {
        RCLCPP_WARN(this->get_logger(),
            "Dropped samples detected. Time difference: %.5f, Dropped samples: %d",
            time_diff, dropped_samples);
      }
      this->update_dropped_sample_count();

    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "Time difference between consecutive samples: %.5f", time_diff);
    }
  }

  /* Update the previous time. */
  this->previous_time = sample_time;
}

/* Note: This method is only relevant in the non-mTMS context, where triggers are sent to the TMS device to deliver pulses. */
void EegDecider::handle_trigger_from_eeg_device(const double_t actual_trigger_time) {
  if (this->previous_stimulation_time == UNSET_PREVIOUS_TIME) {
    return;
  }

  /* Calculate the time difference between the incoming EEG trigger and the trigger time. */
  double_t timing_error = actual_trigger_time - previous_stimulation_time;

  RCLCPP_INFO(this->get_logger(), "Actual trigger from EEG device at: %.4f (s), excepted trigger at: %.4f (s), timing error: %.1f (ms)", actual_trigger_time, previous_stimulation_time, 1000 * timing_error);

  /* Publish timing error ROS message. */
  auto msg = pipeline_interfaces::msg::TimingError();
  msg.error = timing_error;

  this->timing_error_publisher->publish(msg);
}

void EegDecider::process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  auto start_time = std::chrono::high_resolution_clock::now();

  double_t sample_time = msg->time;

  /* Check that session has started. */
  if (this->session_state.value != system_interfaces::msg::SessionState::STARTED) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Session not started, not processing EEG sample at time %.3f (s).",
                         sample_time);
    return;
  }

  /* Update EEG info with every new session OR if this is the first EEG sample received. */
  if (this->first_sample_of_session || this->first_sample_ever) {
    update_eeg_info(msg->metadata);

    /* Avoid checking for dropped samples on the first sample. */
    this->previous_time = UNSET_PREVIOUS_TIME;

    this->first_sample_ever = false;
  }

  /* Check that the decider is enabled. */
  if (!this->enabled) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Decider not enabled");
    return;
  }

  /* Assert that module name is set - we shouldn't otherwise allow to enable the decider. */
  assert(this->module_name != UNSET_STRING);

  if (this->reinitialize ||
      this->decider_wrapper->get_state() == WrapperState::UNINITIALIZED ||
      this->first_sample_of_session) {

    initialize_module();
    reset_decider_state();

    this->reinitialize = false;
  }
  this->first_sample_of_session = false;

  /* Check that the decider module has not encountered an error. */
  if (this->decider_wrapper->get_state() == WrapperState::ERROR) {
    this->decider_state = DeciderState::MODULE_ERROR;

    if (this->decider_state != DeciderState::MODULE_ERROR) {
      RCLCPP_INFO(this->get_logger(), "An error occurred in decider module.");
    }
    return;
  }

  check_dropped_samples(sample_time);

  /* If the sample includes a trigger, handle it acoordingly. */
  if (msg->pulse_delivered) {
    handle_trigger_from_eeg_device(sample_time);
  }

  /* Append the sample to the buffer. */
  this->sample_buffer.append(msg);

  /* Check if periodic processing should trigger based on time comparison. */
  bool periodic_processing_triggered = false;
  if (this->decider_wrapper->is_processing_interval_enabled()) {
    if (sample_time >= this->next_periodic_processing_time - this->TOLERANCE_S) {
      /* Move to next processing time and mark that periodic processing should occur (if buffer is full and not in lockout). */
      this->next_periodic_processing_time += this->decider_wrapper->get_periodic_processing_interval();
      periodic_processing_triggered = true;
    }
  }

  /* Only proceed with actual processing if the buffer is full. This removes the inconvenience of handling partial buffers on the Python side. */
  if (!this->sample_buffer.is_full()) {
    return;
  }

  /* Process any deferred requests that are now ready (have enough look-ahead samples). */
  process_ready_deferred_requests(sample_time);

  /* Check if any decider-defined events occur at the current sample. */
  auto [has_event, event_time, event_type] = consume_next_event(sample_time);
  if (has_event) {
    RCLCPP_INFO(this->get_logger(), "Received decider-defined event at time %.4f (s), event type: %s", sample_time, event_type.c_str());
  }

  /* Check if we're in the pulse lockout period. */
  bool in_lockout_period = false;
  if (!std::isnan(this->pulse_lockout_end_time) && sample_time < this->pulse_lockout_end_time) {
    in_lockout_period = true;
  }

  /* Check if the sample should trigger a Python call. */
  bool should_trigger_python_call = false;

  /* If the sample includes an EEG trigger, always trigger a Python call
     (irrespective of the lockout period). The Python-side handler can return None if it
     doesn't care about the trigger. */
  bool pulse_delivered = msg->pulse_delivered;
  if (pulse_delivered) {
    should_trigger_python_call = true;
  }

  /* If periodic processing was triggered and we're not in lockout, schedule processing. */
  if (periodic_processing_triggered && !in_lockout_period) {
    should_trigger_python_call = true;
  }

  /* Always trigger processing if the sample includes an event (irrespective of the lockout period). */
  if (has_event) {
    should_trigger_python_call = true;
  }

  /* Return early if no Python call should be triggered. */
  if (!should_trigger_python_call) {
    return;
  }

  /* Defer processing based on the number of look-ahead samples.
  
     Calculate when processing should actually occur based on the look-ahead window. 
     For a sample window like [-10, 5], we need 5 samples of look-ahead after the
     triggering sample before we can process it.
      
     The look-ahead depends on what's being processed:
       - Events with custom sample windows: use event-specific look-ahead
       - EEG triggers and periodic processing: use default look-ahead */
  int look_ahead_samples;
  if (has_event) {
    look_ahead_samples = this->decider_wrapper->get_look_ahead_samples_for_event(event_type);
  } else {
    look_ahead_samples = this->decider_wrapper->get_look_ahead_samples();
  }
  
  /* Create a deferred processing request. */
  DeferredProcessingRequest request;
  request.triggering_sample = msg;
  request.pulse_delivered = pulse_delivered;
  request.has_event = has_event;
  request.event_type = event_type;
  
  /* Calculate the time when we'll have enough look-ahead samples.
      If look-ahead is 5 samples, we need to wait for 5 more samples after this one.
      Each sample takes sampling_period time. */
  if (look_ahead_samples > 0) {
    request.scheduled_time = sample_time + (look_ahead_samples * this->sampling_period);
    RCLCPP_DEBUG(this->get_logger(), 
                  "Deferring processing for sample at %.4f (s) until %.4f (s) (look-ahead: %d samples)",
                  sample_time, request.scheduled_time, look_ahead_samples);
  } else {
    /* If look-ahead is 0 or negative, no look-ahead is needed, process at this sample time. */
    request.scheduled_time = sample_time;
  }
  
  /* Add to deferred processing queue. */
  this->deferred_processing_queue.push(request);
  
  /* Check if the request we just added can be processed immediately (e.g., if look_ahead_samples == 0). */
  process_ready_deferred_requests(sample_time);
}

void EegDecider::spin() {
  auto base_interface = this->get_node_base_interface();

  while (rclcpp::ok()) {
    rclcpp::spin_some(base_interface);
  }
}


int main(int argc, char *argv[]) {
  // Install signal handlers for crash debugging
  signal(SIGSEGV, crash_handler);
  signal(SIGABRT, crash_handler);
  signal(SIGFPE, crash_handler);

  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("decider");

  realtime_utils::MemoryConfig mem_config;
  mem_config.enable_memory_optimization = true;
  mem_config.preallocate_size = 10 * 1024 * 1024; // 10 MB

  realtime_utils::SchedulingConfig sched_config;
  sched_config.enable_scheduling_optimization = true;
  sched_config.scheduling_policy = SCHED_RR;
  sched_config.priority_level = realtime_utils::PriorityLevel::HIGHEST_REALTIME;

  try {
    realtime_utils::initialize_scheduling(sched_config, logger);
    realtime_utils::initialize_memory(mem_config, logger);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Initialization failed: %s", e.what());
    return -1;
  }

  auto node = std::make_shared<EegDecider>();

  node->spin();
  rclcpp::shutdown();

  return 0;
}
