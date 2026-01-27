#include <chrono>
#include <filesystem>
#include <signal.h>
#include <execinfo.h>

#include "decider_wrapper.h"
#include "decider.h"

#include "realtime_utils/utils.h"
#include "filesystem_utils/filesystem_utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string HEALTHCHECK_TOPIC = "/eeg/decider/healthcheck";
const std::string IS_COIL_AT_TARGET_TOPIC = "/neuronavigation/coil_at_target";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_MODULE_NAME = "example";

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

EegDecider::EegDecider() : Node("decider"), logger(rclcpp::get_logger("decider")) {
  /* Read ROS parameter: Minimum interval between consecutive pulses (in seconds). */
  auto minimum_intertrial_interval_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  minimum_intertrial_interval_descriptor.description = "The minimum interval between consecutive pulses (in seconds)";
  minimum_intertrial_interval_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  /* XXX: Have to provide 0.0 as a default value because the parameter server does not interpret NULL correctly
          when the parameter is a double. */
  this->declare_parameter("minimum-intertrial-interval", 0.0, minimum_intertrial_interval_descriptor);
  this->get_parameter("minimum-intertrial-interval", this->minimum_intertrial_interval);

  /* Read ROS parameter: timing latency threshold */
  auto pipeline_latency_threshold_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  pipeline_latency_threshold_descriptor.description = "The threshold for the timing latency (in seconds) before stimulation is prevented";
  this->declare_parameter("timing-latency-threshold", 0.005, pipeline_latency_threshold_descriptor);
  this->get_parameter("timing-latency-threshold", this->pipeline_latency_threshold);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Minimum pulse interval: %.1f (s)", this->minimum_intertrial_interval);
  RCLCPP_INFO(this->get_logger(), "  Timing latency threshold: %.1f (ms)", 1000 * this->pipeline_latency_threshold);

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

  /* Note: The EEG subscriber will be during initialization based on whether preprocessor is enabled. */

  /* Set up QoS profile for persistent state topics */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  /* Subscriber for is coil at target. */
  this->is_coil_at_target_subscriber = create_subscription<std_msgs::msg::Bool>(
    IS_COIL_AT_TARGET_TOPIC,
    qos_persist_latest,
    std::bind(&EegDecider::handle_is_coil_at_target, this, _1));

  /* Publisher for timing error. */
  this->timing_error_publisher = this->create_publisher<pipeline_interfaces::msg::TimingError>(
    "/pipeline/timing/error",
    10);

  /* Publisher for decision trace. */
  this->decision_trace_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionTrace>(
    "/pipeline/decision_trace",
    10);

  /* Subscriber for timing latency. */
  this->pipeline_latency_subscriber = this->create_subscription<pipeline_interfaces::msg::PipelineLatency>(
    "/pipeline/latency",
    10,
    std::bind(&EegDecider::handle_pipeline_latency, this, _1));

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

  /* Service client for timed trigger. */
  this->timed_trigger_client = this->create_client<pipeline_interfaces::srv::RequestTimedTrigger>("/pipeline/timed_trigger");

  while (!timed_trigger_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /pipeline/timed_trigger not available, waiting...");
  }

  /* Initialize variables. */
  this->decider_wrapper = std::make_unique<DeciderWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>();

  /* Initialize service server for component initialization */
  this->initialize_service_server = this->create_service<pipeline_interfaces::srv::InitializeDecider>(
    "/pipeline/decider/initialize",
    std::bind(&EegDecider::handle_initialize_decider, this, std::placeholders::_1, std::placeholders::_2));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<pipeline_interfaces::srv::FinalizeDecider>(
    "/pipeline/decider/finalize",
    std::bind(&EegDecider::handle_finalize_decider, this, std::placeholders::_1, std::placeholders::_2));

  this->healthcheck_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EegDecider::publish_healthcheck, this));
}

void EegDecider::handle_initialize_decider(
  const std::shared_ptr<pipeline_interfaces::srv::InitializeDecider::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::InitializeDecider::Response> response) {

  // Reset state
  if (!this->reset_state()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reset decider state while initializing");
    response->success = false;
    return;
  }

  // Set enabled state
  this->is_enabled = request->enabled;

  // If not enabled, just mark as disabled and return early
  if (!request->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Decider marked as disabled: project=%s, module=%s",
                request->project_name.c_str(), request->module_filename.c_str());
    response->success = true;
    return;
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / request->project_name;
  std::filesystem::path decider_path = project_path / "decider";
  std::filesystem::path module_path = decider_path / request->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    response->success = false;
    return;
  }

  // Store initialization state
  this->initialized_project_name = request->project_name;
  this->initialized_module_filename = request->module_filename;
  this->initialized_working_directory = decider_path;

  // Store stream info
  this->stream_info = request->stream_info;

  // Store session ID and reset decision ID
  this->session_id = request->session_id;
  this->decision_id = 0;

  // Change working directory to the module directory
  if (!filesystem_utils::change_working_directory(decider_path.string(), this->get_logger())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s", decider_path.string().c_str());
    response->success = false;
    return;
  }

  // Extract module name from filename (remove .py extension)
  std::string module_name = request->module_filename;
  if (module_name.size() > 3 && module_name.substr(module_name.size() - 3) == ".py") {
    module_name = module_name.substr(0, module_name.size() - 3);
  }

  /* Create the EEG subscriber based on whether preprocessor is enabled. */
  this->eeg_subscriber.reset();

  std::string topic = request->preprocessor_enabled ? EEG_PREPROCESSED_TOPIC : EEG_ENRICHED_TOPIC;
  this->eeg_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    topic,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegDecider::process_sample, this, _1));
  
  // Print section header
  log_section_header("Loading decider: " + module_name);

  // Initialize the decider wrapper
  bool success = this->decider_wrapper->initialize_module(
    PROJECTS_DIRECTORY,
    this->initialized_working_directory.string(),
    module_name,
    request->subject_id,
    request->stream_info.num_eeg_channels,
    request->stream_info.num_emg_channels,
    request->stream_info.sampling_frequency,
    this->sensory_stimuli,
    this->event_queue);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize decider module");
    response->success = false;
    return;
  }

  // Publish initialization logs from Python constructor
  publish_python_logs(0.0, true);

  // Get buffer size and set up sample buffer
  size_t buffer_size = this->decider_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %s%d%s Hz", bold_on.c_str(), request->stream_info.sampling_frequency, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %s%d%s", bold_on.c_str(), request->stream_info.num_eeg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %s%d%s", bold_on.c_str(), request->stream_info.num_emg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  /* Perform warm-up if requested by the Python module */
  bool was_warmup_successful = this->decider_wrapper->warm_up();
  if (!was_warmup_successful) {
    RCLCPP_ERROR(this->get_logger(), "Failed to warm up decider module.");
    response->success = false;
    return;
  }

  // Send the initial sensory stimuli to the presenter
  for (auto& sensory_stimulus : this->sensory_stimuli) {
    this->sensory_stimulus_publisher->publish(sensory_stimulus);
  }
  this->sensory_stimuli.clear();

  // Mark as initialized
  this->is_initialized = true;

  RCLCPP_INFO(this->get_logger(), "Decider initialized successfully: project=%s, module=%s",
              request->project_name.c_str(), request->module_filename.c_str());

  response->success = true;
}

void EegDecider::handle_finalize_decider(
  const std::shared_ptr<pipeline_interfaces::srv::FinalizeDecider::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::FinalizeDecider::Response> response) {

  response->success = this->reset_state();

  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reset decider state");
  }
}

bool EegDecider::reset_state() {
  bool success = true;

  this->initialized_project_name = UNSET_STRING;
  this->initialized_module_filename = UNSET_STRING;
  this->initialized_working_directory = "";

  // Reset session and decision tracking
  this->session_id = {};
  this->decision_id = 0;

  this->is_initialized = false;
  this->is_enabled = false;
  this->error_occurred = false;
  this->previous_stimulation_time = UNSET_TIME;
  this->pulse_lockout_end_time = UNSET_TIME;
  this->next_periodic_processing_time = UNSET_TIME;
  this->pipeline_latency = 0.0;

  /* Clear the event queue. */
  {
    decltype(this->event_queue) empty;
    this->event_queue.swap(empty);
  }

  /* Clear deferred processing queue. */
  {
    decltype(this->deferred_processing_queue) empty;
    this->deferred_processing_queue.swap(empty);
  }

  /* Reset decider wrapper. */
  if (this->decider_wrapper) {
    success &= this->decider_wrapper->reset_module_state();
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reset decider module state");
    }
  }

  /* Reset sample buffer. */
  this->sample_buffer.reset(0);

  /* Clear sensory stimuli. */
  this->sensory_stimuli.clear();

  return success;
}

void EegDecider::publish_healthcheck() {
  auto healthcheck = system_interfaces::msg::Healthcheck();

  switch (this->error_occurred) {
    case true:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
      healthcheck.status_message = "Error occurred";
      healthcheck.actionable_message = "An error occurred in decider.";
      break;

    case false:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
      healthcheck.status_message = "Ready";
      healthcheck.actionable_message = "No error occurred in decider.";
      break; 
  }
  this->healthcheck_publisher->publish(healthcheck);
}

void EegDecider::handle_pipeline_latency(const std::shared_ptr<pipeline_interfaces::msg::PipelineLatency> msg) {
  this->pipeline_latency = msg->latency;
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

std::tuple<bool, double, std::string> EegDecider::consume_next_event(double_t current_time) {
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

    if (event_time - this->TOLERANCE_S >= current_time - 1.0 / this->stream_info.sampling_frequency) {
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

bool EegDecider::is_sample_window_valid() const {
  /* Check that all samples in the current buffer window are valid for processing.
     A window is invalid if:
     1. The buffer is not yet full (not enough samples)
     2. Any sample in the window is paused
     3. Any sample in the window is in a rest period
     4. Any sample in the window is marked as invalid by the preprocessor */

  if (!this->sample_buffer.is_full()) {
    return false;
  }

  bool has_invalid_sample = false;
  this->sample_buffer.process_elements([&has_invalid_sample](const std::shared_ptr<eeg_interfaces::msg::Sample>& sample) {
    if (sample->paused || sample->in_rest || !sample->valid) {
      has_invalid_sample = true;
    }
  });

  return !has_invalid_sample;
}

void EegDecider::process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time) {
  /* Suppress unused parameter warning */
  (void)current_sample_time;

  /* Validate that the current sample window is suitable for processing. */
  if (!is_sample_window_valid()) {
    return;
  }

  /* Capture timing for decision trace */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_decider_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t sample_time = request.triggering_sample->time;

  /* Process the sample. */
  auto [success, timed_trigger, coil_target] = this->decider_wrapper->process(
    this->sensory_stimuli,
    this->sample_buffer,
    sample_time,
    request.triggering_sample->pulse_trigger,
    request.has_event,
    request.event_type,
    this->event_queue,
    this->is_coil_at_target);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(sample_time, false);

  /* Log and return early if the Python call failed. */
  if (!success) {
    RCLCPP_ERROR(this->get_logger(),
                 "Python call failed, not processing EEG sample at time %.3f (s).",
                 sample_time);
    this->error_occurred = true;
    return;
  }

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

  /* Check if the decision is positive. */
  bool stimulate = (timed_trigger != nullptr);

  /* Calculate the total latency of the decider. */
  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_decider_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    end_time.time_since_epoch()).count();

  double_t decider_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Publish decision trace. */
  auto decision_trace = pipeline_interfaces::msg::DecisionTrace();

  // Metadata (filled by Decider)
  decision_trace.session_id = this->session_id;
  decision_trace.decision_id = ++this->decision_id;  // Increment decision ID

  // Status (filled by each stage) - initially set by Decider
  decision_trace.status = stimulate ? pipeline_interfaces::msg::DecisionTrace::STATUS_DECIDED_YES
                                   : pipeline_interfaces::msg::DecisionTrace::STATUS_DECIDED_NO;

  // Decision info
  decision_trace.reference_sample_time = sample_time;
  decision_trace.reference_sample_index = request.triggering_sample->sample_index;
  decision_trace.stimulate = stimulate;
  decision_trace.requested_stimulation_time = timed_trigger ? timed_trigger->time : 0.0;

  // Decider / preprocessing timing
  decision_trace.decider_duration = decider_duration;
  decision_trace.preprocessor_duration = request.triggering_sample->preprocessor_duration;

  // System timing
  decision_trace.system_time_decider_received = system_time_decider_received;
  decision_trace.system_time_decider_finished = system_time_decider_finished;

  this->decision_trace_publisher->publish(decision_trace);

  /* If the decision is negative, return early. */
  if (!stimulate) {
    return;
  }

  /* Check that timing latency is within threshold. */
  if (this->pipeline_latency > this->pipeline_latency_threshold) {
    RCLCPP_ERROR(this->get_logger(), "Timing latency (%.1f ms) exceeds threshold (%.1f ms), ignoring stimulation request.", this->pipeline_latency * 1000, this->pipeline_latency_threshold * 1000);
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
  auto request_msg = std::make_shared<pipeline_interfaces::srv::RequestTimedTrigger::Request>();

  request_msg->timed_trigger = *timed_trigger;
  request_msg->session_id = this->session_id;
  request_msg->decision_id = this->decision_id;
  this->request_timed_trigger(request_msg);

  RCLCPP_INFO(this->get_logger(), "Timing trigger at time %.3f (s).", timed_trigger->time);

  /* Update the previous stimulation time. */
  this->previous_stimulation_time = timed_trigger->time;
  
  /* Set the pulse lockout end time. */
  this->pulse_lockout_end_time = timed_trigger->time + this->decider_wrapper->get_pulse_lockout_duration();
}

void EegDecider::enqueue_deferred_request(const std::shared_ptr<eeg_interfaces::msg::Sample> msg, double_t sample_time, bool has_event, const std::string& event_type) {
  /* Create a deferred processing request. */
  DeferredProcessingRequest request;
  request.triggering_sample = msg;
  request.has_event = has_event;
  request.event_type = event_type;

  /* Calculate the number of look-ahead samples needed.
     The look-ahead depends on what's being processed:
       - Events with custom sample windows: use event-specific look-ahead
       - EEG triggers and periodic processing: use default look-ahead */
  int look_ahead_samples;
  if (has_event) {
    look_ahead_samples = this->decider_wrapper->get_look_ahead_samples_for_event(event_type);
  } else {
    look_ahead_samples = this->decider_wrapper->get_look_ahead_samples();
  }

  /* Calculate the time when we'll have enough look-ahead samples.
     If look-ahead is 5 samples, we need to wait for 5 more samples after this one.
     Each sample takes sampling_period time. */
  request.scheduled_time = sample_time + (look_ahead_samples * 1.0 / this->stream_info.sampling_frequency);

  /* Add to deferred processing queue. */
  this->deferred_processing_queue.push(request);
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

void EegDecider::handle_is_coil_at_target(const std::shared_ptr<std_msgs::msg::Bool> msg) {
  this->is_coil_at_target = msg->data;
}

void EegDecider::handle_pulse_trigger(const double_t pulse_trigger_time) {
  if (this->previous_stimulation_time == UNSET_TIME) {
    return;
  }

  /* Calculate the time difference between the incoming pulse and the expected pulse time. */
  double_t timing_error = pulse_trigger_time - previous_stimulation_time;

  RCLCPP_INFO(this->get_logger(), "Pulse delivered at: %.4f (s), expected pulse at: %.4f (s), timing error: %.1f (ms)", pulse_trigger_time, previous_stimulation_time, 1000 * timing_error);

  /* Publish timing error ROS message. */
  auto msg = pipeline_interfaces::msg::TimingError();
  msg.error = timing_error;

  this->timing_error_publisher->publish(msg);
}

void EegDecider::process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Return early if decider is not enabled or initialized. */
  if (!this->is_enabled || !this->is_initialized) {
    return;
  }
  double_t sample_time = msg->time;

  /* Check that no error has occurred. */
  if (this->error_occurred) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in decider module, not processing EEG sample at time %.3f (s).",
                         sample_time);
    return;
  }

  /* If the sample includes a trigger, handle it accordingly. */
  if (msg->pulse_trigger) {
    handle_pulse_trigger(sample_time);
  }

  /* Append the sample to the buffer. */
  this->sample_buffer.append(msg);

  /* Check if periodic processing should trigger based on time comparison. */
  bool periodic_processing_triggered = false;
  if (this->decider_wrapper->is_processing_interval_enabled()) {
    // Initialize next periodic processing time if not already set.
    if (std::isnan(this->next_periodic_processing_time)) {
      this->next_periodic_processing_time = this->decider_wrapper->get_first_periodic_processing_at();
    }

    // Check if it's time to trigger periodic processing.
    if (sample_time >= this->next_periodic_processing_time - this->TOLERANCE_S) {
      /* Move to next processing time and mark that periodic processing should occur. */
      this->next_periodic_processing_time += this->decider_wrapper->get_periodic_processing_interval();
      periodic_processing_triggered = true;
    }
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

  /* Check if the sample should trigger a Python call. One of the following must be true:

     1. Periodic processing was triggered AND we're not in the pulse lockout period
     2. The sample includes a pulse delivered
     3. The sample includes an event. */
  bool should_trigger_python_call = (periodic_processing_triggered && !in_lockout_period) || msg->pulse_trigger || has_event;

  /* Return early if no Python call should be triggered. */
  if (!should_trigger_python_call) {
    return;
  }

  /* Enqueue a deferred processing request. */
  enqueue_deferred_request(msg, sample_time, has_event, event_type);

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
