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

  /* Initialize module manager for preprocessor modules. */
  module_utils::ModuleManagerConfig module_config;
  module_config.component_name = "preprocessor";
  module_config.projects_base_directory = PROJECTS_DIRECTORY;
  module_config.module_subdirectory = "preprocessor";
  module_config.file_extensions = {".py"};
  module_config.default_module_name = DEFAULT_PREPROCESSOR_NAME;
  module_config.active_project_topic = "/projects/active";
  module_config.module_list_topic = "/pipeline/preprocessor/list";
  module_config.set_module_service = "/pipeline/preprocessor/module/set";
  module_config.module_topic = "/pipeline/preprocessor/module";
  module_config.set_enabled_service = "/pipeline/preprocessor/enabled/set";
  module_config.enabled_topic = "/pipeline/preprocessor/enabled";
  
  this->module_manager = std::make_unique<module_utils::ModuleManager>(this, module_config);

  /* Publisher for Python logs from preprocessor. */
  // Logs can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all_logs = rclcpp::QoS(rclcpp::KeepAll())
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->python_log_publisher = this->create_publisher<pipeline_interfaces::msg::LogMessages>(
    "/pipeline/preprocessor/log",
    qos_keep_all_logs);

  /* Initialize variables. */
  this->preprocessor_wrapper = std::make_unique<PreprocessorWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>();
  this->preprocessed_sample = eeg_interfaces::msg::Sample();

  this->healthcheck_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EegPreprocessor::publish_healthcheck, this));
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

  this->is_session_ongoing = true;

  this->session_metadata.update(metadata);

  /* Clear deferred processing queue when session starts. */
  while (!this->deferred_processing_queue.empty()) {
    this->deferred_processing_queue.pop();
  }

  this->error_occurred = !this->initialize_module();
}

void EegPreprocessor::handle_session_end() {
  RCLCPP_INFO(this->get_logger(), "Session stopped");
  this->is_session_ongoing = false;
}

bool EegPreprocessor::initialize_module() {
  RCLCPP_INFO(this->get_logger(), " ");

  /* Print underlined, bolded text. */
  std::string text_str = "Loading preprocessor: " + this->module_manager->get_module_name();
  std::wstring underline_str(text_str.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), text_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());

  RCLCPP_INFO(this->get_logger(), "");

  bool success = this->preprocessor_wrapper->initialize_module(
    this->module_manager->get_working_directory(),
    this->module_manager->get_module_name(),
    this->session_metadata.num_eeg_channels,
    this->session_metadata.num_emg_channels,
    this->session_metadata.sampling_frequency);

  /* Publish initialization logs from Python constructor */
  publish_python_logs(0.0, true);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize preprocessor.");
    return false;
  }

  size_t buffer_size = this->preprocessor_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %d Hz", this->session_metadata.sampling_frequency);
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %d", this->session_metadata.num_eeg_channels);
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %d", this->session_metadata.num_emg_channels);
  RCLCPP_INFO(this->get_logger(), " ");

  return true;
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

bool EegPreprocessor::set_preprocessor_enabled(bool enabled) {
  RCLCPP_INFO(this->get_logger(), "Preprocessor %s.", enabled ? "enabled" : "disabled");
  return true;
}

void EegPreprocessor::unset_preprocessor_module() {
  RCLCPP_INFO(this->get_logger(), "Preprocessor module unset.");

  /* Reset the Python module state. */
  this->preprocessor_wrapper->reset_module_state();

  /* Disable the preprocessor. */
  set_preprocessor_enabled(false);
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

void EegPreprocessor::process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time) {
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

  /* Calculate preprocessing duration. */
  auto end_time = std::chrono::high_resolution_clock::now();
  preprocessed_sample.preprocessing_duration = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Publish the preprocessed sample. */
  preprocessed_eeg_publisher->publish(preprocessed_sample);
}

void EegPreprocessor::process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Return early if preprocessor module is not enabled. */
  if (!this->module_manager->is_enabled()) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  double_t sample_time = msg->time;

  /* Handle session start marker from upstream. */
  if (msg->is_session_start) {
    handle_session_start(msg->session);
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

  if (!this->sample_buffer.is_full()) {
    return;
  }

  /* Process any deferred requests that are now ready (= have enough look-ahead samples). */
  process_ready_deferred_requests(sample_time);

  /* For every sample, create a deferred processing request.
     Calculate when processing should actually occur based on the look-ahead window. */
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

  /* Check requests once more so that a new request that was just added can be processed immediately if needed. */
  process_ready_deferred_requests(sample_time);

  /* Handle session end marker from upstream (after processing the sample)
     TODO: Early returns make this not work correctly in all cases. Please fix. */
  if (msg->is_session_end) {
    handle_session_end();
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
