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

  /* Initialize service server for component initialization */
  this->initialize_service_server = this->create_service<pipeline_interfaces::srv::InitializePreprocessor>(
    "/pipeline/preprocessor/initialize",
    std::bind(&EegPreprocessor::handle_initialize_preprocessor, this, std::placeholders::_1, std::placeholders::_2));

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

void EegPreprocessor::handle_initialize_preprocessor(
  const std::shared_ptr<pipeline_interfaces::srv::InitializePreprocessor::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::InitializePreprocessor::Response> response) {
  
  // Reset state
  if (!this->reset_state()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reset preprocessor state while initializing");
    response->success = false;
    return;
  }

  // Set enabled state
  this->is_enabled = request->enabled;

  // If not enabled, just mark as disabled and return early
  if (!request->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Preprocessor marked as disabled: project=%s, module=%s",
                request->project_name.c_str(), request->module_filename.c_str());
    response->success = true;
    return;
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / request->project_name;
  std::filesystem::path preprocessor_path = project_path / "preprocessor";
  std::filesystem::path module_path = preprocessor_path / request->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    response->success = false;
    return;
  }

  // Store initialization state
  this->initialized_project_name = request->project_name;
  this->initialized_module_filename = request->module_filename;
  this->initialized_working_directory = preprocessor_path;

  // Store stream info
  this->stream_info = request->stream_info;

  // Change working directory to the module directory
  if (!filesystem_utils::change_working_directory(preprocessor_path.string(), this->get_logger())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s", preprocessor_path.string().c_str());
    response->success = false;
    return;
  }

  // Extract module name from filename (remove .py extension)
  std::string module_name = request->module_filename;
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
    request->subject_id,
    request->stream_info.num_eeg_channels,
    request->stream_info.num_emg_channels,
    request->stream_info.sampling_frequency);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize preprocessor module");
    response->success = false;
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
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %d Hz", request->stream_info.sampling_frequency);
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %d", request->stream_info.num_eeg_channels);
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %d", request->stream_info.num_emg_channels);
  RCLCPP_INFO(this->get_logger(), " ");

  response->success = true;
}

void EegPreprocessor::handle_finalize_preprocessor(
  const std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Response> response) {

  response->success = this->reset_state();

  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to reset preprocessor state");
  }
}

bool EegPreprocessor::reset_state() {
  bool success = true;

  this->initialized_project_name = UNSET_STRING;
  this->initialized_module_filename = UNSET_STRING;
  this->initialized_working_directory = "";

  this->is_initialized = false;
  this->is_enabled = false;
  this->error_occurred = false;
  this->pending_session_start = false;
  this->pending_session_end = false;

  /* Reset sample buffer. */
  this->sample_buffer.reset(0);

  /* Clear deferred processing queue. */
  {
    decltype(this->deferred_processing_queue) empty;
    this->deferred_processing_queue.swap(empty);
  }

  /* Reset preprocessor wrapper. */
  if (this->preprocessor_wrapper) {
    success &= this->preprocessor_wrapper->reset_module_state();
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reset preprocessor module state");
    }
  }
  return success;
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

  /* Copy arrival time from the triggering sample. */
  preprocessed_sample.arrival_time = triggering_sample->arrival_time;

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
    request.scheduled_time = sample_time + (look_ahead_samples * 1.0 / this->stream_info.sampling_frequency);
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

  /* Handle session start marker by marking that we need to carry it over to the next published sample. */
  if (msg->is_session_start) {
    this->pending_session_start = true;
  }

  /* Handle session end marker similarly. */
  if (msg->is_session_end) {
    this->pending_session_end = true;
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
     publish a sentinel sample with the marker for downstream nodes. */
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
