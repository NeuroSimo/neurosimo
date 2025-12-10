#include <chrono>
#include <filesystem>

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "preprocessor_wrapper.h"
#include "preprocessor.h"

#include "realtime_utils/utils.h"

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

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegPreprocessor::handle_set_active_project, this, _1));

  /* Publisher for listing preprocessors. */
  this->preprocessor_list_publisher = this->create_publisher<project_interfaces::msg::PreprocessorList>(
    "/pipeline/preprocessor/list",
    qos_persist_latest);

  /* Service for changing preprocessor module. */
  this->set_preprocessor_module_service = this->create_service<project_interfaces::srv::SetPreprocessorModule>(
    "/pipeline/preprocessor/module/set",
    std::bind(&EegPreprocessor::handle_set_preprocessor_module, this, _1, _2));

  /* Publisher for preprocessor module. */
  this->preprocessor_module_publisher = this->create_publisher<std_msgs::msg::String>(
    "/pipeline/preprocessor/module",
    qos_persist_latest);

  /* Service for enabling and disabling preprocessor. */
  this->set_preprocessor_enabled_service = this->create_service<std_srvs::srv::SetBool>(
    "/pipeline/preprocessor/enabled/set",
    std::bind(&EegPreprocessor::handle_set_preprocessor_enabled, this, _1, _2));

  /* Publisher for preprocessor enabled message. */
  this->preprocessor_enabled_publisher = this->create_publisher<std_msgs::msg::Bool>(
    "/pipeline/preprocessor/enabled",
    qos_persist_latest);

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
    std::bind(&EegPreprocessor::inotify_timer_callback, this));

  this->healthcheck_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EegPreprocessor::publish_healthcheck, this));
}

EegPreprocessor::~EegPreprocessor() {
  inotify_rm_watch(inotify_descriptor, watch_descriptor);
  close(inotify_descriptor);
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
  this->error_occurred = false;

  this->session_metadata.update(metadata);

  /* Clear deferred processing queue when session starts. */
  while (!this->deferred_processing_queue.empty()) {
    this->deferred_processing_queue.pop();
  }

  initialize_module();
}

void EegPreprocessor::handle_session_end() {
  RCLCPP_INFO(this->get_logger(), "Session stopped");
  this->is_session_ongoing = false;
}

void EegPreprocessor::initialize_module() {
  if (this->working_directory == UNSET_STRING ||
      this->module_name == UNSET_STRING) {

    RCLCPP_INFO(this->get_logger(), "Not initializing preprocessor module, module unset.");
    this->error_occurred = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(), " ");

  /* Print underlined, bolded text. */
  std::string text_str = "Loading preprocessor: " + this->module_name;
  std::wstring underline_str(text_str.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), text_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());

  RCLCPP_INFO(this->get_logger(), "");

  bool success = this->preprocessor_wrapper->initialize_module(
    this->working_directory,
    this->module_name,
    this->session_metadata.num_eeg_channels,
    this->session_metadata.num_emg_channels,
    this->session_metadata.sampling_frequency);

  /* Publish initialization logs from Python constructor */
  publish_python_logs(0.0, true);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize preprocessor.");
    this->error_occurred = true;
    return;
  }

  size_t buffer_size = this->preprocessor_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %d Hz", this->session_metadata.sampling_frequency);
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %d", this->session_metadata.num_eeg_channels);
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %d", this->session_metadata.num_emg_channels);
  RCLCPP_INFO(this->get_logger(), " ");
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

/* Listing and setting EEG preprocessors. */
bool EegPreprocessor::set_preprocessor_enabled(bool enabled) {
  /* Update global state variable. */
  this->enabled = enabled;

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = enabled;
  this->preprocessor_enabled_publisher->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Preprocessor %s.", this->enabled ? "enabled" : "disabled");

  return true;
}

void EegPreprocessor::handle_set_preprocessor_enabled(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  response->success = set_preprocessor_enabled(request->data);
  response->message = "";
}

std::string EegPreprocessor::get_module_name_with_fallback(const std::string module_name) {
  if (std::find(this->modules.begin(), this->modules.end(), module_name) != this->modules.end()) {
    return module_name;
  }
  if (std::find(this->modules.begin(), this->modules.end(), DEFAULT_PREPROCESSOR_NAME) != this->modules.end()) {
    RCLCPP_WARN(this->get_logger(), "Module %s not found, setting to default", module_name.c_str());
    return DEFAULT_PREPROCESSOR_NAME;
  }
  if (!this->modules.empty()) {
    RCLCPP_WARN(this->get_logger(), "Module %s not found, setting to first module on the list: %s", module_name.c_str(), this->modules[0].c_str());
    return this->modules[0];
  }
  RCLCPP_WARN(this->get_logger(), "No preprocessors found in project: %s%s%s.", bold_on.c_str(), this->active_project.c_str(), bold_off.c_str());
  return UNSET_STRING;
}

void EegPreprocessor::unset_preprocessor_module() {
  this->module_name = UNSET_STRING;

  RCLCPP_INFO(this->get_logger(), "Preprocessor module unset.");

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->preprocessor_module_publisher->publish(msg);

  /* Reset the Python module state. */
  this->preprocessor_wrapper->reset_module_state();

  /* Disable the preprocessor. */
  set_preprocessor_enabled(false);
}

bool EegPreprocessor::set_preprocessor_module(const std::string module_name) {
  this->module_name = get_module_name_with_fallback(module_name);

  if (this->module_name == UNSET_STRING) {
    RCLCPP_ERROR(this->get_logger(), "No preprocessor module set.");
    this->unset_preprocessor_module();

    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Preprocessor set to: %s.", this->module_name.c_str());

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->preprocessor_module_publisher->publish(msg);

  return true;
}

void EegPreprocessor::handle_set_preprocessor_module(
      const std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPreprocessorModule::Response> response) {

  response->success = set_preprocessor_module(request->module);
}

void EegPreprocessor::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;

  RCLCPP_INFO(this->get_logger(), "Project set to: %s.", this->active_project.c_str());

  this->is_working_directory_set = change_working_directory(PROJECTS_DIRECTORY + "/" + this->active_project + "/preprocessor");
  update_preprocessor_list();

  update_inotify_watch();
}

/* File-system related functions */

bool EegPreprocessor::change_working_directory(const std::string path) {
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

std::vector<std::string> EegPreprocessor::list_python_modules_in_working_directory() {
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

void EegPreprocessor::update_inotify_watch() {
  /* Remove the old watch. */
  inotify_rm_watch(inotify_descriptor, watch_descriptor);

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

  /* Add a new watch. */
  watch_descriptor = inotify_add_watch(inotify_descriptor, this->working_directory.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE | IN_MOVE);
  if (watch_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error adding watch for: %s", this->working_directory.c_str());
      return;
  }
}

void EegPreprocessor::inotify_timer_callback() {
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
      if ((event->mask & IN_MODIFY) &&
          (event_name == this->module_name + ".py")) {

        RCLCPP_INFO(this->get_logger(), "The current module '%s' was modified.", this->module_name.c_str());
      }
      if (event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) {
        RCLCPP_INFO(this->get_logger(), "File '%s' created, deleted, or moved, updating preprocessor list.", event_name.c_str());
        this->update_preprocessor_list();
      }
    }
    i += sizeof(struct inotify_event) + event->len;
  }
}

void EegPreprocessor::update_preprocessor_list() {
  if (this->is_working_directory_set) {
    this->modules = this->list_python_modules_in_working_directory();
  } else {
    this->modules.clear();
  }
  auto msg = project_interfaces::msg::PreprocessorList();
  msg.scripts = this->modules;

  this->preprocessor_list_publisher->publish(msg);
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
