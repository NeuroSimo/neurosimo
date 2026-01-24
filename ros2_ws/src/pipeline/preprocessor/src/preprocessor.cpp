#include <chrono>
#include <filesystem>

#include "preprocessor_wrapper.h"
#include "preprocessor.h"

#include "realtime_utils/utils.h"
#include "filesystem_utils/filesystem_utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string HEALTHCHECK_TOPIC = "/eeg/preprocessor/healthcheck";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_PREPROCESSOR_NAME = "example";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

EegPreprocessor::EegPreprocessor() : Node("preprocessor"), logger(rclcpp::get_logger("preprocessor")) {
  /* Publisher for healthcheck. */
  this->healthcheck_publisher = this->create_publisher<system_interfaces::msg::Healthcheck>(HEALTHCHECK_TOPIC, 10);

  /* Publisher for preprocessed EEG data. */
  this->preprocessed_eeg_publisher = this->create_publisher<eeg_interfaces::msg::Sample>(EEG_PREPROCESSED_TOPIC, EEG_QUEUE_LENGTH);

  /* Subscriber for EEG data. */
  this->enriched_eeg_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    EEG_ENRICHED_TOPIC,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegPreprocessor::process_sample, this, _1));

  /* Publisher for Python logs from preprocessor. */
  // Logs can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all_logs = rclcpp::QoS(rclcpp::KeepAll())
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->python_log_publisher = this->create_publisher<pipeline_interfaces::msg::LogMessages>(
    "/pipeline/preprocessor/log",
    qos_keep_all_logs);

  /* Initialize action server for component initialization */
  this->initialize_action_server = rclcpp_action::create_server<pipeline_interfaces::action::InitializePreprocessor>(
    this,
    "/pipeline/preprocessor/initialize",
    std::bind(&EegPreprocessor::handle_initialize_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&EegPreprocessor::handle_initialize_cancel, this, std::placeholders::_1),
    std::bind(&EegPreprocessor::handle_initialize_accepted, this, std::placeholders::_1));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<pipeline_interfaces::srv::FinalizePreprocessor>(
    "/pipeline/preprocessor/finalize",
    std::bind(&EegPreprocessor::handle_finalize_preprocessor, this, std::placeholders::_1, std::placeholders::_2));

  /* Initialize variables. */
  this->preprocessor_wrapper = std::make_unique<PreprocessorWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>();
  this->preprocessed_sample = eeg_interfaces::msg::Sample();

  this->healthcheck_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EegPreprocessor::publish_healthcheck, this));
}

rclcpp_action::GoalResponse EegPreprocessor::handle_initialize_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipeline_interfaces::action::InitializePreprocessor::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received initialize goal: project='%s', module='%s', enabled=%s",
              goal->project_name.c_str(), goal->module_filename.c_str(), goal->enabled ? "true" : "false");

  // Accept all goals for now
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EegPreprocessor::handle_initialize_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel initialize goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EegPreprocessor::handle_initialize_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle) {
  // Execute the initialization in a new thread
  std::thread{std::bind(&EegPreprocessor::execute_initialize, this, std::placeholders::_1), goal_handle}.detach();
}

void EegPreprocessor::execute_initialize(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePreprocessor>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipeline_interfaces::action::InitializePreprocessor::Result>();

  // Set enabled state
  this->is_enabled = goal->enabled;

  // If not enabled, just mark as disabled and return early
  if (!goal->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Preprocessor marked as disabled: project=%s, module=%s",
                goal->project_name.c_str(), goal->module_filename.c_str());
    result->success = true;
    goal_handle->succeed(result);
    return;
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / goal->project_name;
  std::filesystem::path preprocessor_path = project_path / "preprocessor";
  std::filesystem::path module_path = preprocessor_path / goal->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  // Store initialization state
  this->initialized_project_name = goal->project_name;
  this->initialized_module_filename = goal->module_filename;
  this->initialized_working_directory = preprocessor_path;

  // Extract module name from filename (remove .py extension)
  std::string module_name = goal->module_filename;
  if (module_name.size() > 3 && module_name.substr(module_name.size() - 3) == ".py") {
    module_name = module_name.substr(0, module_name.size() - 3);
  }

  /* Print underlined, bolded text. */
  std::string text_str = "Loading preprocessor: " + module_name;
  std::wstring underline_str(text_str.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), text_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());

  RCLCPP_INFO(this->get_logger(), "");

  // Initialize the preprocessor wrapper
  bool success = this->preprocessor_wrapper->initialize_module(
    this->initialized_working_directory.string(),
    module_name,
    goal->num_eeg_channels,
    goal->num_emg_channels,
    goal->sampling_frequency);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize preprocessor module");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  // Publish initialization logs from Python constructor
  publish_python_logs(0.0, true);

  // Get buffer size and set up sample buffer
  size_t buffer_size = this->preprocessor_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  // Mark as initialized
  this->is_initialized = true;

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %d Hz", goal->sampling_frequency);
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %d", goal->num_eeg_channels);
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %d", goal->num_emg_channels);
  RCLCPP_INFO(this->get_logger(), " ");

  result->success = true;
  goal_handle->succeed(result);
}

void EegPreprocessor::handle_finalize_preprocessor(
  const std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Received finalize request for session: %s", request->session_id.c_str());

  // Finalize the preprocessor module if initialized
  if (this->is_initialized && this->preprocessor_wrapper) {
    bool finalize_success = this->preprocessor_wrapper->finalize_module();
    if (!finalize_success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to finalize preprocessor module");
      response->success = false;
      return;
    }
  }

  // Reset initialization state
  this->is_initialized = false;
  this->is_enabled = false;
  this->initialized_project_name = UNSET_STRING;
  this->initialized_module_filename = UNSET_STRING;
  this->initialized_working_directory = "";

  // Clear buffers and state
  this->sample_buffer.reset(0);
  while (!this->deferred_processing_queue.empty()) {
    this->deferred_processing_queue.pop();
  }
  this->error_occurred = false;

  // Reset session metadata
  this->session_metadata = SessionMetadataState{};

  RCLCPP_INFO(this->get_logger(), "Preprocessor finalized successfully for session: %s", request->session_id.c_str());
  response->success = true;
}

void EegPreprocessor::publish_healthcheck() {
  auto healthcheck = system_interfaces::msg::Healthcheck();

  switch (this->error_occurred) {
    case true:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
      healthcheck.status_message = "Error occurred";
      healthcheck.actionable_message = "An error occurred in preprocessor.";
      break;

    case false:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
      healthcheck.status_message = "Ready";
      healthcheck.actionable_message = "No error occurred in preprocessor.";
      break;
  } 
  this->healthcheck_publisher->publish(healthcheck);
}

void EegPreprocessor::handle_session_start(const eeg_interfaces::msg::SessionMetadata& metadata) {
  RCLCPP_INFO(this->get_logger(), "Session started");

  this->session_metadata.update(metadata);

  /* Clear deferred processing queue when session starts. */
  while (!this->deferred_processing_queue.empty()) {
    this->deferred_processing_queue.pop();
  }

  /* Clear any stale session markers from previous session. */
  this->pending_session_end = false;

  /* Mark that we need to carry forward the session start marker to the next published sample. */
  this->pending_session_start = true;
}

void EegPreprocessor::handle_session_end() {
  RCLCPP_INFO(this->get_logger(), "Session stopped");

  /* Mark that we need to carry forward the session end marker to the next published sample. */
  this->pending_session_end = true;
}

void EegPreprocessor::publish_python_logs(double sample_time, bool is_initialization) {
  auto logs = this->preprocessor_wrapper->get_and_clear_logs();
  
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

void EegPreprocessor::process_ready_deferred_requests(double_t current_sample_time) {
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

bool EegPreprocessor::is_sample_window_valid() const {
  /* Check that all samples in the current buffer window are valid for processing.
     A window is invalid if:
     1. The buffer is not yet full (not enough samples)
     2. Any sample in the window is paused
     3. Any sample in the window is in a rest period */

  if (!this->sample_buffer.is_full()) {
    return false;
  }

  bool has_invalid_sample = false;
  this->sample_buffer.process_elements([&has_invalid_sample](const std::shared_ptr<eeg_interfaces::msg::Sample>& sample) {
    if (sample->paused || sample->in_rest) {
      has_invalid_sample = true;
    }
  });

  return !has_invalid_sample;
}

void EegPreprocessor::publish_sentinel_sample(double_t sample_time) {
  /* Publish a sentinel sample with the session end marker. */
  auto sentinel_sample = std::make_shared<eeg_interfaces::msg::Sample>();
  sentinel_sample->time = sample_time;
  sentinel_sample->is_session_end = this->pending_session_end;

  /* Clear the pending session end marker. */
  this->pending_session_end = false;

  this->preprocessed_eeg_publisher->publish(*sentinel_sample);
}

void EegPreprocessor::process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time) {
  /* Validate that the current sample window is suitable for processing. */
  if (!is_sample_window_valid()) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  auto triggering_sample = request.triggering_sample;
  double_t sample_time = triggering_sample->time;

  /* Process the sample. */
  bool success = this->preprocessor_wrapper->process(
    preprocessed_sample,
    this->sample_buffer,
    sample_time,
    triggering_sample->pulse_delivered);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(sample_time, false);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(),
                 "Python call failed, not processing EEG sample at time %.3f (s).",
                 sample_time);
    this->error_occurred = true;
    return;
  }

  /* Copy session information from the triggering sample. */
  preprocessed_sample.session.sampling_frequency = triggering_sample->session.sampling_frequency;
  preprocessed_sample.session.num_eeg_channels = triggering_sample->session.num_eeg_channels;
  preprocessed_sample.session.num_emg_channels = triggering_sample->session.num_emg_channels;
  preprocessed_sample.session.is_simulation = triggering_sample->session.is_simulation;
  preprocessed_sample.session.start_time = triggering_sample->session.start_time;

  /* Copy hardware trigger information. */
  preprocessed_sample.pulse_delivered = triggering_sample->pulse_delivered;

  /* Carry forward any pending session markers. */
  preprocessed_sample.is_session_start = this->pending_session_start;
  preprocessed_sample.is_session_end = this->pending_session_end;

  /* Clear the pending markers after carrying them forward. */
  this->pending_session_start = false;
  this->pending_session_end = false;

  /* Calculate preprocessing duration. */
  auto end_time = std::chrono::high_resolution_clock::now();
  preprocessed_sample.preprocessing_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Publish the preprocessed sample. */
  preprocessed_eeg_publisher->publish(preprocessed_sample);
}

void EegPreprocessor::enqueue_deferred_request(const std::shared_ptr<eeg_interfaces::msg::Sample> msg, double_t sample_time) {
  /* For every sample, create a deferred processing request. */
  int look_ahead_samples = this->preprocessor_wrapper->get_look_ahead_samples();

  DeferredProcessingRequest request;
  request.triggering_sample = msg;

  /* Calculate the time when we'll have enough look-ahead samples.
     If look-ahead is 5 samples, we need to wait for 5 more samples after this one.
     Each sample takes sampling period time. */
  if (look_ahead_samples > 0) {
    request.scheduled_time = sample_time + (look_ahead_samples * this->session_metadata.sampling_period);
  } else {
    /* If look-ahead is 0 or negative, no look-ahead is needed, process immediately. */
    request.scheduled_time = sample_time;
  }

  /* Add to deferred processing queue. */
  this->deferred_processing_queue.push(request);
}

void EegPreprocessor::process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Return early if preprocessor is not enabled or initialized. */
  if (!this->is_enabled || !this->is_initialized) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  double_t sample_time = msg->time;

  /* Handle session start marker from upstream. */
  if (msg->is_session_start) {
    handle_session_start(msg->session);
  }

  /* Handle session end marker from upstream. */
  if (msg->is_session_end) {
    handle_session_end();
  }

  /* Check that no error has occurred. */
  if (this->error_occurred) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in preprocessor module, not processing EEG sample at time %.3f (s).",
                         sample_time);
    return;
  }

  /* Check that session metadata stays constant within a session. */
  if (!this->session_metadata.matches(msg->session)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Session metadata changed mid-session. Rejecting sample at %.3f (s).",
                 sample_time);
    this->error_occurred = true;
    return;
  }

  if (msg->pulse_delivered) {
    RCLCPP_INFO(this->get_logger(), "Pulse delivered at: %.1f (s).", sample_time);
  }

  this->sample_buffer.append(msg);

  /* For every sample, create and enqueue a deferred processing request. */
  enqueue_deferred_request(msg, sample_time);

  /* Process any deferred requests that are now ready (= have enough look-ahead samples),
     including the one that was just added. */
  process_ready_deferred_requests(sample_time);

  /* If there's a pending session end marker, meaning that this is the last sample of the session,
     publish a sentinel sample with the marker. Otherwise, downstream nodes will not know when the
     session ends. */
  if (this->pending_session_end) {
    publish_sentinel_sample(sample_time);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("preprocessor");

  realtime_utils::MemoryConfig mem_config;
  mem_config.enable_memory_optimization = true;
  mem_config.preallocate_size = 10 * 1024 * 1024; // 10 MB

  realtime_utils::SchedulingConfig sched_config;
  sched_config.enable_scheduling_optimization = true;
  sched_config.scheduling_policy = SCHED_RR;
  sched_config.priority_level = realtime_utils::PriorityLevel::REALTIME;

  try {
    realtime_utils::initialize_scheduling(sched_config, logger);
    realtime_utils::initialize_memory(mem_config, logger);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Initialization failed: %s", e.what());
    return -1;
  }

  auto node = std::make_shared<EegPreprocessor>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
