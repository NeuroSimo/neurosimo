#include <chrono>
#include <algorithm>
#include <filesystem>
#include <signal.h>
#include <execinfo.h>
#include <limits>

#define XXH_INLINE_ALL
#include <xxhash.h>

#include "decider_wrapper.h"
#include "decider.h"

#include "realtime_utils/utils.h"
#include "filesystem_utils/filesystem_utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string EEG_PREPROCESSED_TOPIC = "/neurosimo/eeg/preprocessed";
const std::string EEG_ENRICHED_TOPIC = "/neurosimo/eeg/enriched";
const std::string HEARTBEAT_TOPIC = "/neurosimo/decider/heartbeat";
const std::string IS_COIL_AT_TARGET_TOPIC = "/neuronavigation/coil_at_target";
const std::string TARGETED_PULSES_TOPIC = "/mtms/targeted_pulses";

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
  /* Publisher for heartbeat. */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(HEARTBEAT_TOPIC, 10);

  /* Note: The EEG subscriber will be during initialization based on whether preprocessor is enabled. */

  /* Set up QoS profile for persistent state topics */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  /* Publisher for health. */
  this->health_publisher = this->create_publisher<neurosimo_system_interfaces::msg::ComponentHealth>(
    "/neurosimo/decider/health", qos_persist_latest);

  /* Subscriber for is coil at target. */
  this->is_coil_at_target_subscriber = create_subscription<std_msgs::msg::Bool>(
    IS_COIL_AT_TARGET_TOPIC,
    qos_persist_latest,
    std::bind(&EegDecider::handle_is_coil_at_target, this, _1));

  this->loopback_latency_subscriber = create_subscription<neurosimo_pipeline_interfaces::msg::LoopbackLatency>(
    "/neurosimo/pipeline/latency/loopback",
    10,
    std::bind(&EegDecider::handle_loopback_latency, this, _1));

  /* Publisher for decision trace. */
  this->decision_trace_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::DecisionTrace>(
    "/neurosimo/pipeline/decision_trace",
    10);

  /* Publisher for attempt trace. */
  this->attempt_trace_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>(
    "/neurosimo/pipeline/attempt_trace",
    10);

  /* Publisher for sensory stimulus. */

  // Messages can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all = rclcpp::QoS(rclcpp::KeepAll())
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->sensory_stimulus_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::SensoryStimulus>(
    "/neurosimo/pipeline/sensory_stimulus",
    qos_keep_all);

  /* Publisher for targeted pulses from decider output. */
  this->targeted_pulses_publisher = this->create_publisher<shared_stimulation_interfaces::msg::TargetedPulses>(
    TARGETED_PULSES_TOPIC,
    qos_keep_all);

  /* Publisher for coil target. */
  this->coil_target_publisher = this->create_publisher<shared_stimulation_interfaces::msg::CoilTarget>(
    "/neuronavigation/coil_target",
    10);

  /* Publisher for Python logs from decider. */
  // Logs can be sent in bursts so keep all messages in the queue.
  // Use TRANSIENT_LOCAL durability so late-joining subscribers get buffered messages.
  auto qos_keep_all_logs = rclcpp::QoS(rclcpp::KeepAll())
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->python_log_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::LogMessages>(
    "/neurosimo/pipeline/decider/log",
    qos_keep_all_logs);

  /* Publisher for pulse processing latency. */
  this->pulse_processing_latency_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::Latency>(
    "/neurosimo/pipeline/latency/pulse_processing",
    10);

  /* Publisher for event processing latency. */
  this->event_processing_latency_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::Latency>(
    "/neurosimo/pipeline/latency/event_processing",
    10);

  /* Publisher for task finished. */
  this->task_finished_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::TaskFinished>(
    "/neurosimo/pipeline/task_finished",
    10);

  /* Subscriber for task start from experiment coordinator. */
  this->task_start_subscriber = create_subscription<neurosimo_pipeline_interfaces::msg::TaskStart>(
    "/neurosimo/pipeline/task_start",
    10,
    std::bind(&EegDecider::handle_task_start, this, _1));

  /* Service client for timed trigger.

     The service server is created at session start, hence do not wait for it here. */
  this->timed_trigger_client = this->create_client<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>(
    "/neurosimo/pipeline/timed_trigger", rclcpp::QoS(rclcpp::ServicesQoS()));

  /* Service client for session abort. */
  this->abort_session_client = this->create_client<neurosimo_system_interfaces::srv::AbortSession>(
    "/neurosimo/session/abort", rclcpp::QoS(rclcpp::ServicesQoS()));

  while (!abort_session_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /neurosimo/session/abort not available, waiting...");
  }

  /* Initialize variables. */
  this->decider_wrapper = std::make_unique<DeciderWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>();

  /* Initialize service server for component initialization */
  this->initialize_service_server = this->create_service<neurosimo_pipeline_interfaces::srv::InitializeDecider>(
    "/neurosimo/pipeline/decider/initialize",
    std::bind(&EegDecider::handle_initialize_decider, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::QoS(rclcpp::ServicesQoS()));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<neurosimo_pipeline_interfaces::srv::FinalizeDecider>(
    "/neurosimo/pipeline/decider/finalize",
    std::bind(&EegDecider::handle_finalize_decider, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::QoS(rclcpp::ServicesQoS()));

  /* Initialize thread control flags */
  this->heartbeat_thread_running = false;
  this->log_thread_running = false;

  /* Start heartbeat thread */
  this->heartbeat_thread_running = true;
  this->heartbeat_thread = std::thread([this]() {
    while (this->heartbeat_thread_running) {
      this->publish_heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  /* Start log drain thread */
  this->log_thread_running = true;
  this->log_thread = std::thread([this]() {
    while (this->log_thread_running) {
      this->check_and_publish_logs();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  /* Publish initial health status. */
  this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::READY, "");

  RCLCPP_INFO(this->get_logger(), "Decider node initialized successfully");
}

void EegDecider::handle_initialize_decider(
  const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeDecider::Request> request,
  std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeDecider::Response> response) {

  this->is_enabled = request->enabled;

  // If not enabled, just mark as disabled and return early
  if (!request->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Decider marked as disabled: project=%s, module=%s",
                request->project_name.c_str(), request->module_filename.c_str());
    response->success = true;

    // Do not shutdown the node if it is not enabled, only in case of error.
    return;
  }

  // Set safety configuration from request
  this->minimum_trial_interval = request->minimum_trial_interval;

  // Validate the minimum trial interval
  if (this->minimum_trial_interval <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid minimum trial interval: %.1f (s)", this->minimum_trial_interval);
    response->success = false;

    // Shutdown the node to get a clean state after the error.
    this->shutdown_requested = true;
    return;
  }

  if (this->minimum_trial_interval < 0.5) {
    RCLCPP_WARN(this->get_logger(), "Note: Minimum trial interval is very low: %.1f (s)", this->minimum_trial_interval);
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / request->project_name;
  std::filesystem::path decider_path = project_path / "decider";
  std::filesystem::path module_path = decider_path / request->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    response->success = false;

    // Shutdown the node to get a clean state after the error.
    this->shutdown_requested = true;
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
  this->targeted_pulses_sequence_number = 0;

  // Change working directory to the module directory
  if (!filesystem_utils::change_working_directory(decider_path.string(), this->get_logger())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s", decider_path.string().c_str());
    response->success = false;

    // Shutdown the node to get a clean state after the error.
    this->shutdown_requested = true;
    return;
  }

  // Extract module name from filename (remove .py extension)
  std::string module_name = request->module_filename;
  if (module_name.size() > 3 && module_name.substr(module_name.size() - 3) == ".py") {
    module_name = module_name.substr(0, module_name.size() - 3);
  }

  /* Store preprocessor enabled state for backpressure detection. */
  this->preprocessor_enabled = request->preprocessor_enabled;

  /* Create the EEG subscriber based on whether preprocessor is enabled. */
  std::string topic = this->preprocessor_enabled ? EEG_PREPROCESSED_TOPIC : EEG_ENRICHED_TOPIC;
  this->eeg_subscriber = create_subscription<neurosimo_eeg_interfaces::msg::Sample>(
    topic,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegDecider::process_sample, this, _1));

  /* Subscribe to attempt commit messages from experiment coordinator. */
  this->attempt_commit_subscriber = create_subscription<neurosimo_pipeline_interfaces::msg::AttemptCommit>(
    "/neurosimo/pipeline/attempt_commit",
    10,
    std::bind(&EegDecider::handle_attempt_commit, this, _1));
  
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

  // Publish initialization logs from Python constructor
  this->decider_wrapper->drain_logs();
  publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_INITIALIZATION, 0.0);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize decider module");
    response->success = false;

    // Shutdown the node to get a clean state after the error.
    this->shutdown_requested = true;
    return;
  }

  // Get buffer size and set up sample buffer
  size_t buffer_size = this->decider_wrapper->get_envelope_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %s%d%s Hz", bold_on.c_str(), request->stream_info.sampling_frequency, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %s%d%s", bold_on.c_str(), request->stream_info.num_eeg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %s%d%s", bold_on.c_str(), request->stream_info.num_emg_channels, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Safety configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Minimum trial interval: %s%.1f%s (s)", bold_on.c_str(), this->minimum_trial_interval, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  /* Perform warm-up if requested by the Python module */
  bool was_warmup_successful = this->decider_wrapper->warm_up();
  if (!was_warmup_successful) {
    RCLCPP_ERROR(this->get_logger(), "Failed to warm up decider module.");

    this->decider_wrapper->drain_logs();
    publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_INITIALIZATION, 0.0);

    response->success = false;

    // Shutdown the node to get a clean state after the error.
    this->shutdown_requested = true;
    return;
  }

  // Send the initial sensory stimuli to the presenter
  for (auto& sensory_stimulus : this->sensory_stimuli) {
    this->sensory_stimulus_publisher->publish(sensory_stimulus);
  }
  this->sensory_stimuli.clear();

  // Mark as initialized
  this->is_initialized = true;
  this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::READY, "");

  response->success = true;
}

EegDecider::~EegDecider() {
  /* Stop and join threads */
  this->heartbeat_thread_running = false;
  this->log_thread_running = false;

  if (this->heartbeat_thread.joinable()) {
    this->heartbeat_thread.join();
  }

  if (this->log_thread.joinable()) {
    this->log_thread.join();
  }
}

void EegDecider::handle_finalize_decider(
  [[maybe_unused]] const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeDecider::Request> request,
  std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeDecider::Response> response) {

  /* Publish logs from the previous sample at the beginning of the finalization. */
  if (!std::isnan(this->previous_sample_time)) {
    this->decider_wrapper->drain_logs();
    publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_RUNTIME, this->previous_sample_time);
  }

  /* Destroy the Python instance first so its __del__ runs before log draining. */
  this->decider_wrapper->destroy_instance();

  /* Drain and publish output from __del__. */
  this->decider_wrapper->drain_logs();
  publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_FINALIZATION, 0.0);

  /* Store the final fingerprint before resetting state */
  response->decision_fingerprint = this->decision_fingerprint;
  RCLCPP_INFO(this->get_logger(), "Session decision fingerprint: 0x%016lx", response->decision_fingerprint);

  RCLCPP_INFO(this->get_logger(), "Decider finalized successfully");
  response->success = true;

  /* Request shutdown */
  this->shutdown_requested = true;
}

void EegDecider::publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void EegDecider::check_and_publish_logs() {
  /* Publish any pending Python logs if handle_sample hasn't checked them since the last timer call. */
  if (!this->logs_checked_since_last_timer && !std::isnan(this->previous_sample_time)) {
    publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_RUNTIME, this->previous_sample_time);
  }

  /* Reset the flag for the next timer interval. */
  this->logs_checked_since_last_timer = false;
}

void EegDecider::publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = neurosimo_system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void EegDecider::log_section_header(const std::string& title) {
  std::wstring underline_str(title.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), title.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");
}

void EegDecider::publish_python_logs(uint8_t phase, double sample_time) {
  auto logs = this->decider_wrapper->get_and_clear_logs();
  
  if (logs.empty()) {
    return;
  }
  
  // Create a single batched message containing all logs
  auto batch_msg = neurosimo_pipeline_interfaces::msg::LogMessages();
  
  for (const auto& log_entry : logs) {
    // Create individual log message
    auto log_msg = neurosimo_pipeline_interfaces::msg::LogMessage();
    log_msg.message = log_entry.message;
    log_msg.sample_time = sample_time;
    log_msg.level = static_cast<uint8_t>(log_entry.level);
    log_msg.phase = phase;
    log_msg.processing_path = log_entry.processing_path;

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

std::tuple<bool, double> EegDecider::consume_next_event(double_t current_time) {
  if (this->event_queue.empty()) {
    return std::make_tuple(false, 0.0);
  }

  double_t event_time;

  /* Pop events until the event time is within the tolerance. */
  while (!this->event_queue.empty()) {
    event_time = this->event_queue.top();

    if (event_time - this->TOLERANCE >= current_time - 1.0 / this->stream_info.sampling_frequency) {
      break;
    }
    this->event_queue.pop();
  }

  /* Check if we popped all events */
  if (this->event_queue.empty()) {
    return std::make_tuple(false, 0.0);
  }

  /* If the event time is too far in the future, return false. */
  if (event_time > current_time + this->TOLERANCE) {
    return std::make_tuple(false, 0.0);
  }
  return std::make_tuple(true, event_time);
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
    if (current_sample_time >= next_request.scheduled_time - this->TOLERANCE) {
      /* Dispatch to the appropriate handler. */
      switch (next_request.processing_reason) {
        case ProcessingReason::Periodic:
          process_periodic_request(next_request);
          break;
        case ProcessingReason::Pulse:
          process_pulse_request(next_request);
          break;
        case ProcessingReason::Event:
          process_event_request(next_request);
          break;
      }
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
     4. Any sample in the window is in a task period
     5. Any sample in the window is marked as invalid by the preprocessor
     6. Any sample in the window failed preprocessing */

  if (!this->sample_buffer.is_full()) {
    return false;
  }

  bool has_invalid_sample = false;
  this->sample_buffer.process_elements([&has_invalid_sample](const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>& sample) {
    if (sample->paused || sample->in_rest || sample->in_task || !sample->valid || sample->preprocessing_failed) {
      has_invalid_sample = true;
    }
  });

  return !has_invalid_sample;
}

void EegDecider::handle_stimulation_request(
    const ProcessResult& result,
    double_t reference_time,
    double_t reference_eeg_device_timestamp,
    uint8_t attempt_timing,
    const std::string& attempt_type,
    uint64_t decision_id) {

  bool request_timed_trigger = result.trigger_offset != nullptr;
  bool request_targeted_pulses = !result.targeted_pulses.empty();

  double_t earliest_pulse_time = std::numeric_limits<double_t>::quiet_NaN();
  double_t latest_pulse_time = std::numeric_limits<double_t>::quiet_NaN();

  if (request_timed_trigger) {
    earliest_pulse_time = reference_time + *result.trigger_offset;
    latest_pulse_time = earliest_pulse_time;
  }

  if (request_targeted_pulses) {
    earliest_pulse_time = reference_time + result.targeted_pulses.front().time_offset;
    latest_pulse_time = earliest_pulse_time;
    for (const auto& pulse : result.targeted_pulses) {
      earliest_pulse_time = std::min(earliest_pulse_time, reference_time + pulse.time_offset);
      latest_pulse_time = std::max(latest_pulse_time, reference_time + pulse.time_offset);
    }
  }

  /* Publish attempt trace. */
  auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
  attempt_trace.session_id = this->session_id;
  attempt_trace.attempt_in_session = this->current_attempt_in_session;
  attempt_trace.requested_stimulation_time = earliest_pulse_time;
  attempt_trace.reference_time = reference_time;
  attempt_trace.attempt_timing = attempt_timing;
  attempt_trace.attempt_type = attempt_type;
  attempt_trace.decision_id = decision_id;
  this->attempt_trace_publisher->publish(attempt_trace);

  /* Store for use when computing timing_offset on pulse observed. */
  this->last_requested_stimulation_time = earliest_pulse_time;

  /* Check that the minimum trial interval has passed. */
  auto time_since_previous_trial = earliest_pulse_time - this->previous_stimulation_time;
  auto has_minimum_trial_interval_passed = std::isnan(this->previous_stimulation_time) ||
                                           time_since_previous_trial >= this->minimum_trial_interval;

  if (!has_minimum_trial_interval_passed) {
    RCLCPP_ERROR(this->get_logger(), "Stimulation requested but minimum trial interval (%.1f s) has not passed (time since previous stimulation: %.3f s).",
      this->minimum_trial_interval,
      time_since_previous_trial);

    this->error_occurred = true;
    this->abort_session("Minimum trial interval not passed");
    return;
  }

  /* If timed triggers are requested, send them. */
  if (request_timed_trigger) {
    RCLCPP_INFO(this->get_logger(), "Timing trigger at time %.3f (s).", earliest_pulse_time);

    auto request_msg = std::make_shared<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request>();
    request_msg->trigger_offset = *result.trigger_offset;
    request_msg->session_id = this->session_id;
    request_msg->attempt_in_session = this->current_attempt_in_session;
    request_msg->reference_sample_time = reference_time;
    this->request_timed_trigger(request_msg);
  }

  /* If targeted pulses are requested, send them. */
  if (request_targeted_pulses) {
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing %zu targeted pulse(s) at time %.3f (s).",
      result.targeted_pulses.size(),
      reference_time);

    auto targeted_pulses_msg = shared_stimulation_interfaces::msg::TargetedPulses();
    targeted_pulses_msg.session_id = this->session_id;
    targeted_pulses_msg.reference_eeg_device_timestamp = reference_eeg_device_timestamp;
    targeted_pulses_msg.sequence_number = ++this->targeted_pulses_sequence_number;
    targeted_pulses_msg.pulses = result.targeted_pulses;

    this->targeted_pulses_publisher->publish(targeted_pulses_msg);
  }

  this->previous_stimulation_time = latest_pulse_time;
  this->stimulation_requested = true;
}

void EegDecider::process_pulse_request(const DeferredProcessingRequest& request) {
  if (!is_sample_window_valid()) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  double_t sample_time = request.triggering_sample->time;
  std::string stage_name = request.triggering_sample->stage_name;

  auto result = this->decider_wrapper->process_pulse(
    this->sensory_stimuli, this->sample_buffer, sample_time,
    this->event_queue, this->is_coil_at_target, stage_name,
    request.triggering_sample->trial_in_stage);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Python call failed during pulse processing at time %.3f (s).", sample_time);
    this->error_occurred = true;
    this->abort_session("Decider Python error");
    return;
  }

  /* Publish sensory stimuli if the vector is not empty. */
  if (!this->sensory_stimuli.empty()) {
    RCLCPP_INFO(this->get_logger(), "Requesting %zu sensory stimuli at time %.3f (s).", this->sensory_stimuli.size(), sample_time);
    for (const auto& sensory_stimulus : this->sensory_stimuli) {
      this->sensory_stimulus_publisher->publish(sensory_stimulus);
    }
    this->sensory_stimuli.clear();
  }

  /* Publish coil target if it is set. */
  if (!result.coil_target.empty()) {
    auto coil_target_msg = shared_stimulation_interfaces::msg::CoilTarget();
    coil_target_msg.target_name = result.coil_target;
    RCLCPP_INFO(this->get_logger(), "Sending coil target %s to neuronavigation at time %.3f (s).", coil_target_msg.target_name.c_str(), sample_time);
    this->coil_target_publisher->publish(coil_target_msg);
  }

  /* Publish the processing duration. */
  auto end_time = std::chrono::high_resolution_clock::now();
  double_t decider_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  auto latency_msg = neurosimo_pipeline_interfaces::msg::Latency();
  latency_msg.latency = decider_duration;
  this->pulse_processing_latency_publisher->publish(latency_msg);

  /* Publish attempt trace with STATUS_PULSE_PROCESSED. */
  auto pulse_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
  pulse_trace.session_id = this->session_id;
  pulse_trace.attempt_in_session = this->current_attempt_in_session;
  pulse_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_PULSE_PROCESSED;
  pulse_trace.actual_stimulation_time = sample_time;
  pulse_trace.actual_stimulation_sample_index = request.triggering_sample->sample_index;
  pulse_trace.timing_offset = sample_time - this->last_requested_stimulation_time;
  pulse_trace.invalid_trial = result.invalid_trial;
  this->attempt_trace_publisher->publish(pulse_trace);
}

void EegDecider::process_event_request(const DeferredProcessingRequest& request) {
  if (!is_sample_window_valid()) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  double_t sample_time = request.triggering_sample->time;
  std::string stage_name = request.triggering_sample->stage_name;

  auto result = this->decider_wrapper->process_event(
    this->sensory_stimuli, this->sample_buffer, sample_time,
    this->event_queue, this->is_coil_at_target, stage_name,
    request.triggering_sample->trial_in_stage);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Python call failed during event processing at time %.3f (s).", sample_time);
    this->error_occurred = true;
    this->abort_session("Decider Python error");
    return;
  }

  /* Publish sensory stimuli if the vector is not empty. */
  if (!this->sensory_stimuli.empty()) {
    RCLCPP_INFO(this->get_logger(), "Requesting %zu sensory stimuli at time %.3f (s).", this->sensory_stimuli.size(), sample_time);
    for (const auto& sensory_stimulus : this->sensory_stimuli) {
      this->sensory_stimulus_publisher->publish(sensory_stimulus);
    }
    this->sensory_stimuli.clear();
  }

  /* Publish coil target if it is set. */
  if (!result.coil_target.empty()) {
    auto coil_target_msg = shared_stimulation_interfaces::msg::CoilTarget();
    coil_target_msg.target_name = result.coil_target;
    this->coil_target_publisher->publish(coil_target_msg);
  }

  /* Publish the processing duration. */
  auto end_time = std::chrono::high_resolution_clock::now();
  double_t decider_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  auto latency_msg = neurosimo_pipeline_interfaces::msg::Latency();
  latency_msg.latency = decider_duration;
  this->event_processing_latency_publisher->publish(latency_msg);
}

void EegDecider::process_periodic_request(const DeferredProcessingRequest& request) {
  if (!is_sample_window_valid()) {
    return;
  }

  /* Capture timing for decision trace */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_decider_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t reference_time = request.triggering_sample->time;
  double_t reference_eeg_device_timestamp = request.triggering_sample->eeg_device_timestamp;
  std::string stage_name = request.triggering_sample->stage_name;

  auto result = this->decider_wrapper->process_periodic(
    this->sensory_stimuli, this->sample_buffer, reference_time,
    this->event_queue, this->is_coil_at_target, stage_name,
    request.triggering_sample->trial_in_stage);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Python call failed during periodic processing at time %.3f (s).", reference_time);
    this->error_occurred = true;
    this->abort_session("Decider Python error");
    return;
  }

  /* Publish sensory stimuli if the vector is not empty. */
  if (!this->sensory_stimuli.empty()) {
    RCLCPP_INFO(this->get_logger(), "Requesting %zu sensory stimuli at time %.3f (s).", this->sensory_stimuli.size(), reference_time);
    for (const auto& sensory_stimulus : this->sensory_stimuli) {
      this->sensory_stimulus_publisher->publish(sensory_stimulus);
    }
    this->sensory_stimuli.clear();
  }

  /* Publish coil target if it is set. */
  if (!result.coil_target.empty()) {
    RCLCPP_INFO(this->get_logger(), "Sending coil target %s to neuronavigation at time %.3f (s).", result.coil_target.c_str(), reference_time);
    auto coil_target_msg = shared_stimulation_interfaces::msg::CoilTarget();
    coil_target_msg.target_name = result.coil_target;
    this->coil_target_publisher->publish(coil_target_msg);
  }

  /* Calculate the total latency of the decider. */
  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_decider_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    end_time.time_since_epoch()).count();

  double_t decider_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Check if timed triggers are requested. */
  bool request_timed_trigger = result.trigger_offset != nullptr;
  bool request_targeted_pulses = !result.targeted_pulses.empty();

  double_t requested_stimulation_time = std::numeric_limits<double_t>::quiet_NaN();
  if (request_timed_trigger) {
    requested_stimulation_time = reference_time + *result.trigger_offset;
  }
  if (request_targeted_pulses) {
    requested_stimulation_time = reference_time + result.targeted_pulses.front().time_offset;
  }

  /* Update decision fingerprint with evaluation info. */
  this->decision_fingerprint = XXH64(
    &reference_time,
    sizeof(reference_time),
    this->decision_fingerprint);

  this->decision_fingerprint = XXH64(
    &requested_stimulation_time,
    sizeof(requested_stimulation_time),
    this->decision_fingerprint);

  /* Calculate the latency of the decision path. */
  auto decision_path_latency = (system_time_decider_finished - request.triggering_sample->system_time_data_source_published) / 1e9;  // Convert nanoseconds to seconds

  /* Publish decision trace (on every decision, including negative). */
  auto decision_trace = neurosimo_pipeline_interfaces::msg::DecisionTrace();

  decision_trace.decision_id = ++this->decision_id;

  bool stimulation_requested = request_timed_trigger || request_targeted_pulses;
  decision_trace.status = stimulation_requested ? neurosimo_pipeline_interfaces::msg::DecisionTrace::STATUS_DECIDED_YES
                                                : neurosimo_pipeline_interfaces::msg::DecisionTrace::STATUS_DECIDED_NO;

  decision_trace.reference_sample_time = reference_time;
  decision_trace.reference_sample_index = request.triggering_sample->sample_index;
  decision_trace.stimulate = stimulation_requested;

  decision_trace.eeg_device_processing_duration = this->latest_eeg_device_processing_duration;
  decision_trace.decider_duration = decider_duration;
  decision_trace.preprocessor_duration = request.triggering_sample->preprocessor_duration;
  decision_trace.overhead_duration = decision_path_latency - request.triggering_sample->preprocessor_duration - decider_duration;
  decision_trace.total_duration = decision_trace.preprocessor_duration + decision_trace.decider_duration + decision_trace.overhead_duration;

  decision_trace.system_time_decider_received = system_time_decider_received;
  decision_trace.system_time_decider_finished = system_time_decider_finished;

  this->decision_trace_publisher->publish(decision_trace);

  /* Handle stimulation requests. */
  if (stimulation_requested) {
    handle_stimulation_request(
      result, reference_time, reference_eeg_device_timestamp,
      neurosimo_pipeline_interfaces::msg::AttemptTrace::ATTEMPT_TIMING_PERIODIC,
      "",
      this->decision_id);
  }
}

void EegDecider::enqueue_deferred_request(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg, double_t sample_time, ProcessingReason processing_reason) {
  /* Create a deferred processing request. */
  DeferredProcessingRequest request;
  request.triggering_sample = msg;
  request.processing_reason = processing_reason;

  /* Calculate the number of look-ahead samples needed based on the processing type. */
  int look_ahead_samples;
  switch (processing_reason) {
    case ProcessingReason::Pulse:
      look_ahead_samples = this->decider_wrapper->get_pulse_look_ahead_samples();
      break;
    case ProcessingReason::Event:
      look_ahead_samples = this->decider_wrapper->get_event_look_ahead_samples();
      break;
    case ProcessingReason::Periodic:
      look_ahead_samples = this->decider_wrapper->get_periodic_look_ahead_samples();
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid processing type: %d", static_cast<int>(processing_reason));
      return;
  }

  /* Negative look-ahead is treated as no deferral. */
  if (look_ahead_samples < 0) {
    look_ahead_samples = 0;
  }

  /* Calculate the time when we'll have enough look-ahead samples.
     If look-ahead is 5 samples, we need to wait for 5 more samples after this one.
     Each sample takes sampling_period time. */
  request.scheduled_time = sample_time + (look_ahead_samples * 1.0 / this->stream_info.sampling_frequency);

  /* Add to deferred processing queue. */
  this->deferred_processing_queue.push(request);
}

/* Service clients */

void EegDecider::request_timed_trigger(std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request> request) {
  using ServiceResponseFuture = rclcpp::Client<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    this->timed_trigger_callback(future);
  };

  auto future_result = this->timed_trigger_client->async_send_request(request, response_received_callback);
}

void EegDecider::timed_trigger_callback(rclcpp::Client<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future) {
  auto result = future.get().second;
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send timed trigger.");
  }
}

void EegDecider::abort_session(const std::string& reason) {
  auto request = std::make_shared<neurosimo_system_interfaces::srv::AbortSession::Request>();
  request->source = "decider";
  request->reason = reason;

  auto result = this->abort_session_client->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Requested session abort: %s", reason.c_str());
}

/* Initialization and reset functions */

void EegDecider::handle_is_coil_at_target(const std::shared_ptr<std_msgs::msg::Bool> msg) {
  this->is_coil_at_target = msg->data;
}

void EegDecider::handle_loopback_latency(
    const std::shared_ptr<neurosimo_pipeline_interfaces::msg::LoopbackLatency> msg) {
  this->latest_eeg_device_processing_duration = msg->eeg_device_processing_duration;
}

void EegDecider::handle_attempt_commit(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::AttemptCommit> msg) {
  if (msg->session_id != this->session_id) {
    RCLCPP_ERROR(this->get_logger(), "Received attempt commit with mismatched session id.");
    return;
  }

  this->attempt_commit_received = true;
  this->stimulation_requested = false;
  this->committed_attempt_in_session = msg->attempt_in_session;
  this->current_attempt_type = msg->trial_timing;
  this->current_trial_type = msg->trial_type;
}

void EegDecider::handle_task_start(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::TaskStart> msg) {
  if (msg->session_id != this->session_id) {
    RCLCPP_ERROR(this->get_logger(), "Received task start with mismatched session id.");
    return;
  }

  if (!this->is_initialized) {
    RCLCPP_ERROR(this->get_logger(), "Received task start but decider is not initialized.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Starting task '%s' (id: %lu)", msg->task_name.c_str(), msg->task_id);

  bool success = this->decider_wrapper->process_task(msg->task_name);

  /* Drain and publish logs generated during task processing. */
  this->decider_wrapper->drain_logs();
  publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_RUNTIME, 0.0);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "process_task failed for task '%s' (id: %lu).", msg->task_name.c_str(), msg->task_id);
    this->error_occurred = true;
    this->abort_session("Error in process_task method");
    return;
  }

  /* Publish TaskFinished to signal completion to the coordinator. */
  auto finished_msg = neurosimo_pipeline_interfaces::msg::TaskFinished();
  finished_msg.session_id = this->session_id;
  finished_msg.task_id = msg->task_id;
  this->task_finished_publisher->publish(finished_msg);

  RCLCPP_INFO(this->get_logger(), "Task '%s' (id: %lu) completed.", msg->task_name.c_str(), msg->task_id);
}

void EegDecider::process_sample(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg) {
  /* Return early if decider is not enabled or initialized. */
  if (!this->is_enabled || !this->is_initialized) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         5000,
                         "Decider not processing samples (is_enabled=%d, is_initialized=%d)",
                         this->is_enabled, this->is_initialized);
    return;
  }
  double_t sample_time = msg->time;

  /* Publish logs from the previous sample at the beginning of this sample */
  if (!std::isnan(this->previous_sample_time)) {
    publish_python_logs(neurosimo_pipeline_interfaces::msg::LogMessage::PHASE_RUNTIME, this->previous_sample_time);

    /* Mark that logs have been checked in this sample processing cycle. */
    this->logs_checked_since_last_timer = true;
  }

  /* Update previous sample time for next iteration's log publishing */
  this->previous_sample_time = sample_time;

  /* Check that no error has occurred. */
  if (this->error_occurred) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in decider module, not processing EEG sample at time %.3f (s).",
                         sample_time);
    return;
  }

  /* Cache attempt counter for use when publishing attempt trace. */
  this->current_attempt_in_session = msg->attempt_in_session;

  /* Track attempt reference: update on every is_attempt_start sample.
     This is the timing anchor from which predetermined pulse offsets are computed. */
  if (msg->is_attempt_start) {
    this->attempt_reference_time = msg->time;
    this->attempt_reference_eeg_device_timestamp = msg->eeg_device_timestamp;
  }

  /* Check for sample index discontinuity and handle gaps. */
  detect_and_handle_sample_gap(msg);

  /* Append the sample to the buffer. */
  this->sample_buffer.append(msg);

  /* Process any deferred requests that are now ready (have enough look-ahead samples). */
  process_ready_deferred_requests(sample_time);

  /* Attempt commit is active if it has been received and the attempt in sample matches the committed attempt. */
  bool attempt_commit_active = this->attempt_commit_received && msg->attempt_in_session == this->committed_attempt_in_session;

  /* If attempt commit is active and stimulation has not been requested, handle the attempt. */
  if (attempt_commit_active && !this->stimulation_requested && !msg->in_task && !msg->in_rest) {
    bool is_periodic = (this->current_attempt_type == neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PERIODIC);
    bool is_predetermined = (this->current_attempt_type == neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PREDETERMINED);
    if (is_periodic) {
      handle_periodic_trial(msg);
    }
    if (is_predetermined) {
      handle_predetermined_trial(msg);
    }
  }

  /* Check if any decider-defined events occur at the current sample. */
  auto [has_event, event_time] = consume_next_event(sample_time);
  if (has_event) {
    RCLCPP_INFO(this->get_logger(), "Received decider-defined event at time %.4f (s)", sample_time);
    enqueue_deferred_request(msg, sample_time, ProcessingReason::Event);
  }

  /* Check if the sample contains a pulse trigger. */
  if (msg->pulse_trigger) {
    RCLCPP_INFO(this->get_logger(), "Received pulse trigger at time %.4f (s)", sample_time);
    enqueue_deferred_request(msg, sample_time, ProcessingReason::Pulse);
  }

  /* Check if the sample contains a simulated event trigger (EEG simulator only). */
  if (msg->event_trigger) {
    RCLCPP_INFO(this->get_logger(), "Received event trigger at time %.4f (s)", sample_time);
    enqueue_deferred_request(msg, sample_time, ProcessingReason::Event);
  }

  /* Check if the request we just added can be processed immediately (e.g., if look_ahead_samples == 0). */
  process_ready_deferred_requests(sample_time);
}

void EegDecider::handle_periodic_trial(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg) {
  auto sample_time = msg->time;
  bool periodic_processing_triggered = false;

  // Initialize next periodic processing time if not already set.
  if (std::isnan(this->next_periodic_processing_time)) {
    this->next_periodic_processing_time = this->decider_wrapper->get_periodic_processing_interval();
  }

  /* Reset periodic processing time at the start of a new attempt to ensure alignment with attempt start. */
  if (msg->is_attempt_start) {
    this->next_periodic_processing_time = sample_time + this->decider_wrapper->get_periodic_processing_interval();
  }

  // Check if it's time to trigger periodic processing.
  if (sample_time >= this->next_periodic_processing_time - this->TOLERANCE) {
    /* Move to next processing time and mark that periodic processing should occur. */
    this->next_periodic_processing_time += this->decider_wrapper->get_periodic_processing_interval();
    periodic_processing_triggered = true;
  }

  /* Check if minimum trial interval has passed since last trigger. */
  bool minimum_trial_interval_passed = std::isnan(this->previous_stimulation_time) ||
                                       sample_time >= this->previous_stimulation_time + this->minimum_trial_interval;

  /* Check for backpressure by comparing current time to the appropriate upstream timestamp. */
  bool backpressure_detected = detect_backpressure(msg);

  /* Check if periodic processing should trigger (skip if backpressure detected). */
  if (periodic_processing_triggered && minimum_trial_interval_passed && !backpressure_detected) {
    enqueue_deferred_request(msg, sample_time, ProcessingReason::Periodic);
  }
}

void EegDecider::handle_predetermined_trial(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg) {
  RCLCPP_INFO(this->get_logger(), "Predetermined trial %u in stage '%s' (type: '%s') at time %.3f (s)",
    msg->trial_in_stage, msg->stage_name.c_str(), this->current_trial_type.c_str(), msg->time);

  /* Use the reference time tracked from the most recent is_attempt_start sample. */
  double_t reference_time = this->attempt_reference_time;
  double_t reference_eeg_device_timestamp = this->attempt_reference_eeg_device_timestamp;

  if (std::isnan(reference_time)) {
    RCLCPP_ERROR(this->get_logger(), "No reference time available for predetermined trial at time %.3f (s).", msg->time);
    this->error_occurred = true;
    this->abort_session("Missing reference time for predetermined trial");
    return;
  }

  /* Call process_predetermined on the Python side. */
  auto result = this->decider_wrapper->process_predetermined(
    this->sensory_stimuli,
    this->event_queue,
    reference_time,
    msg->stage_name,
    msg->trial_in_stage,
    this->current_trial_type);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "process_predetermined failed at time %.3f (s).", msg->time);
    this->error_occurred = true;
    this->abort_session("Error in process_predetermined method");
    return;
  }

  /* Publish sensory stimuli if any. */
  if (!this->sensory_stimuli.empty()) {
    for (const auto& sensory_stimulus : this->sensory_stimuli) {
      this->sensory_stimulus_publisher->publish(sensory_stimulus);
    }
    this->sensory_stimuli.clear();
  }

  /* Publish coil target if set. */
  if (!result.coil_target.empty()) {
    auto coil_target_msg = shared_stimulation_interfaces::msg::CoilTarget();
    coil_target_msg.target_name = result.coil_target;
    this->coil_target_publisher->publish(coil_target_msg);
  }

  bool stimulation_requested = result.trigger_offset != nullptr || !result.targeted_pulses.empty();

  if (!stimulation_requested) {
    RCLCPP_ERROR(this->get_logger(), "No stimulation requested for predetermined trial at time %.3f (s).", msg->time);
    this->error_occurred = true;
    this->abort_session("No stimulation requested for predetermined trial");
    return;
  }

  handle_stimulation_request(
    result, reference_time, reference_eeg_device_timestamp,
    neurosimo_pipeline_interfaces::msg::AttemptTrace::ATTEMPT_TIMING_PREDETERMINED,
    this->current_trial_type);
}

bool EegDecider::detect_backpressure(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg) {
  /* Check for backpressure by comparing current time to the appropriate upstream timestamp. */
  uint64_t upstream_timestamp_ns = this->preprocessor_enabled ?
    msg->system_time_preprocessor_published :
    msg->system_time_experiment_coordinator_published;

  if (upstream_timestamp_ns > 0) {
    auto current_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    double_t latency = (current_time_ns - upstream_timestamp_ns) / 1e9;  // Convert to seconds

    if (latency > BACKPRESSURE_CUTOFF) {
      RCLCPP_WARN_THROTTLE(this->get_logger(),
                          *this->get_clock(),
                          1000,  // Throttle to once per second
                          "Backpressure detected: %.3f s latency (cutoff: %.3f s), skipping periodic processing",
                          latency, BACKPRESSURE_CUTOFF);
      return true;
    }
  }

  return false;
}

void EegDecider::detect_and_handle_sample_gap(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg) {
  /* Check for sample index discontinuity (gap detection). */
  if (this->previous_sample_index != -1 && msg->sample_index > this->previous_sample_index + 1) {
    uint64_t gap_size = msg->sample_index - this->previous_sample_index - 1;
    double gap_duration_s = gap_size / static_cast<double>(this->stream_info.sampling_frequency);

    RCLCPP_WARN(this->get_logger(),
                "Sample index discontinuity detected: expected %lu, received %lu (gap of %lu samples, %.3f s). ",
                this->previous_sample_index + 1, msg->sample_index, gap_size, gap_duration_s);

    /* Publish degraded health status. */
    this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::DEGRADED, "Sample gap detected");

    /* Reset the ring buffer to avoid processing with non-continuous windows. */
    size_t buffer_size = this->decider_wrapper->get_envelope_buffer_size();
    this->sample_buffer.reset(buffer_size);
  }

  /* Update previous sample index for next iteration. */
  this->previous_sample_index = msg->sample_index;
}

void EegDecider::spin() {
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(this->shared_from_this());
  executor->spin();
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

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->shutdown_requested) {
    /* XXX: Do not use spin_some here, as we want to block for the time specified to avoid busy-looping,
            and spin_some doesn't block. Use spin_once instead. */
    executor.spin_once(std::chrono::milliseconds(20));
  }

  rclcpp::shutdown();
  return 0;
}
