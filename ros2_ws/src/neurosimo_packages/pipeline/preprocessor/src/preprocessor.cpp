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

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string HEALTHCHECK_TOPIC = "/eeg/preprocessor/healthcheck";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_PREPROCESSOR_NAME = "example";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

/* XXX: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 5ms;


EegPreprocessor::EegPreprocessor() : Node("preprocessor"), logger(rclcpp::get_logger("preprocessor")) {
  /* Read ROS parameter: dropped sample threshold */
  auto dropped_sample_threshold_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  dropped_sample_threshold_descriptor.description = "The threshold for the number of dropped samples before the decider enters an error state";
  dropped_sample_threshold_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("dropped-sample-threshold", 4, dropped_sample_threshold_descriptor);
  this->get_parameter("dropped-sample-threshold", this->dropped_sample_threshold);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Dropped samples per second threshold: %d", this->dropped_sample_threshold);

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
    std::bind(&EegPreprocessor::handle_session, this, _1),
    subscription_options);

  /* Publisher for preprocessed EEG data. */
  this->preprocessed_eeg_publisher = this->create_publisher<eeg_msgs::msg::PreprocessedSample>(EEG_PREPROCESSED_TOPIC, EEG_QUEUE_LENGTH);

  /* Subscriber for EEG data. */
  this->raw_eeg_subscriber = create_subscription<eeg_msgs::msg::Sample>(
    EEG_RAW_TOPIC,
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
  this->set_preprocessor_enabled_service = this->create_service<project_interfaces::srv::SetPreprocessorEnabled>(
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

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>>();
  this->preprocessed_sample = eeg_msgs::msg::PreprocessedSample();

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

  switch (this->preprocessor_state) {
    case PreprocessorState::WAITING_FOR_ENABLED:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
      healthcheck.status_message = "EEG preprocessor not enabled";
      healthcheck.actionable_message = "Please enable the EEG preprocessor.";
      break;

    case PreprocessorState::READY:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
      healthcheck.status_message = "Ready";
      healthcheck.actionable_message = "Ready";
      break;

    case PreprocessorState::SAMPLES_DROPPED:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Samples dropped";
      healthcheck.actionable_message = "Sample(s) dropped in preprocessor.";
      break;

    case PreprocessorState::DROPPED_SAMPLE_THRESHOLD_EXCEEDED:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Dropped sample threshold exceeded";
      healthcheck.actionable_message = "Dropped sample threshold (" + std::to_string(this->dropped_sample_threshold) + " per second) exceeded.";
      break;

    case PreprocessorState::MODULE_ERROR:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Module error";
      healthcheck.actionable_message = "Preprocessor has encountered an error.";
      break;
  }
  this->healthcheck_publisher->publish(healthcheck);
}

void EegPreprocessor::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  bool state_changed = this->session_state.value != msg->state.value;
  this->session_state = msg->state;

  if (state_changed && this->session_state.value == system_interfaces::msg::SessionState::STOPPED) {
    this->first_sample_of_session = true;
    this->total_dropped_samples = 0;
    this->reinitialize = true;

    /* Clear deferred processing queue when session stops. */
    while (!this->deferred_processing_queue.empty()) {
      this->deferred_processing_queue.pop();
    }

    /* Reset the preprocessor state when the session is stopped. */
    reset_preprocessor_state();
  }
}

void EegPreprocessor::update_eeg_info(const eeg_msgs::msg::SampleMetadata& msg) {
  this->sampling_frequency = msg.sampling_frequency;
  this->num_of_eeg_channels = msg.num_of_eeg_channels;
  this->num_of_emg_channels = msg.num_of_emg_channels;

  this->sampling_period = 1.0 / this->sampling_frequency;
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

void EegPreprocessor::initialize_module() {
  if (this->working_directory == UNSET_STRING ||
      this->module_name == UNSET_STRING) {

    RCLCPP_INFO(this->get_logger(), "Not initializing preprocessor module, module unset.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), " ");

  /* Print underlined, bolded text. */
  std::string text_str = "Loading preprocessor: " + this->module_name;
  std::wstring underline_str(text_str.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), text_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());

  RCLCPP_INFO(this->get_logger(), "");

  this->preprocessor_wrapper->initialize_module(
    this->working_directory,
    this->module_name,
    this->num_of_eeg_channels,
    this->num_of_emg_channels,
    this->sampling_frequency);

  /* Publish initialization logs from Python constructor */
  publish_python_logs(0.0, true);

  if (this->preprocessor_wrapper->get_state() != WrapperState::READY) {
    RCLCPP_INFO(this->get_logger(), "Failed to initialize preprocessor.");
    return;
  }

  size_t buffer_size = this->preprocessor_wrapper->get_buffer_size();
  this->sample_buffer.reset(buffer_size);

  RCLCPP_INFO(this->get_logger(), "EEG configuration:");
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "  - Sampling frequency: %d Hz", this->sampling_frequency);
  RCLCPP_INFO(this->get_logger(), "  - # of EEG channels: %d", this->num_of_eeg_channels);
  RCLCPP_INFO(this->get_logger(), "  - # of EMG channels: %d", this->num_of_emg_channels);
  RCLCPP_INFO(this->get_logger(), " ");
}

void EegPreprocessor::reset_preprocessor_state() {
  this->preprocessor_state = this->enabled ? PreprocessorState::READY : PreprocessorState::WAITING_FOR_ENABLED;
}

/* Listing and setting EEG preprocessors. */
bool EegPreprocessor::set_preprocessor_enabled(bool enabled) {

  /* Only allow enabling the preprocessor if a module is set. */
  if (enabled && this->module_name == UNSET_STRING) {
    RCLCPP_WARN(this->get_logger(), "Cannot enable preprocessor, no module set.");

    return false;
  }

  /* Update global state variable. */
  this->enabled = enabled;

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = enabled;

  this->preprocessor_enabled_publisher->publish(msg);

  /* Re-initialize the module each time the preprocessor is enabled. */
  if (enabled) {
    this->reinitialize = true;
  }

  /* Reset preprocessor state. */
  reset_preprocessor_state();

  RCLCPP_INFO(this->get_logger(), "Preprocessor %s.", this->enabled ? "enabled" : "disabled");

  return true;
}

void EegPreprocessor::handle_set_preprocessor_enabled(
      const std::shared_ptr<project_interfaces::srv::SetPreprocessorEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPreprocessorEnabled::Response> response) {

  response->success = set_preprocessor_enabled(request->enabled);
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

  /* Re-initialize the module each time the module is reset. */
  this->reinitialize = true;

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

        RCLCPP_INFO(this->get_logger(), "The current module '%s' was modified, re-initializing.", this->module_name.c_str());
        this->reinitialize = true;
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

/* EEG functions */

/* XXX: Very close to a similar check in other pipeline stages. Unify? */
void EegPreprocessor::check_dropped_samples(double_t sample_time) {
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
        this->preprocessor_state = PreprocessorState::DROPPED_SAMPLE_THRESHOLD_EXCEEDED;
        RCLCPP_ERROR(this->get_logger(),
            "Dropped samples exceeded threshold! Recent dropped samples: %d, Threshold: %d",
            recent_dropped_samples, this->dropped_sample_threshold);
      } else {
        RCLCPP_WARN(this->get_logger(),
            "Dropped samples detected. Time difference: %.5f, Dropped samples: %d",
            time_diff, dropped_samples);
      }
      // Note: Dropped sample count is published by the decider, not the preprocessor

    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "Time difference between consecutive samples: %.5f", time_diff);
    }
  }

  /* Update the previous time. */
  this->previous_time = sample_time;
}

/* Handle direct pulse feedback from the mTMS device via Decider.

   Note: The mTMS device sends a pulse feedback message when a pulse is given, but this does not apply to
   TMS devices in general. Hence, we also handle EEG triggers as pulses, allowing other TMS devices to work
   with the EEG preprocessor.

   The downside to this logic is that when using a mTMS device with concurrent pulse and trigger out, connected to
   the EEG device, we will get an indication of a pulse twice. This is not a problem, as the direct feedback and the
   EEG trigger should arrive approximately at the same time, hence both will usually be cleared from the queue during the same sample -
   if not, we will have two consecutive samples marked as having a pulse, which is not a major problem for current use cases.

   TODO: However, we should probably have a more robust logic here in the long term; most likely, we would need to know explicitly
   which one to use.
*/

/* TODO: Re-implement. */
/*
void EegPreprocessor::handle_pulse_feedback(const std::shared_ptr<eeg_msgs::msg::PulseFeedback> msg) {
  double_t execution_time = msg->execution_time;
  this->pulse_execution_times.push(execution_time);

  RCLCPP_INFO(this->get_logger(), "Registered pulse feedback from the mTMS device at: %.5f (s).", execution_time);
}
*/

void EegPreprocessor::process_ready_deferred_requests(double_t current_sample_time) {
  /* Process any deferred requests that are now ready (have enough look-ahead samples). */
  while (!this->deferred_processing_queue.empty()) {
    const auto& next_request = this->deferred_processing_queue.top();
    
    /* Check if our current sample time is within the tolerance of the processing time of the next request. */
    if (current_sample_time >= next_request.processing_time - this->TOLERANCE_S) {
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
  
  /* Determine if a pulse was given. */
  bool pulse_given = is_pulse_feedback_received(sample_time) || triggering_sample->is_trigger;
  
  /* Process the sample. */
  bool success = this->preprocessor_wrapper->process(
    preprocessed_sample,
    this->sample_buffer,
    sample_time,
    pulse_given);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(sample_time, false);

  if (success) {
    /* Copy metadata from the triggering sample. */
    preprocessed_sample.metadata.sampling_frequency = triggering_sample->metadata.sampling_frequency;
    preprocessed_sample.metadata.num_of_eeg_channels = triggering_sample->metadata.num_of_eeg_channels;
    preprocessed_sample.metadata.num_of_emg_channels = triggering_sample->metadata.num_of_emg_channels;
    preprocessed_sample.metadata.is_simulation = triggering_sample->metadata.is_simulation;
    preprocessed_sample.metadata.system_time = triggering_sample->metadata.system_time;

    /* Copy event and trigger information. */
    preprocessed_sample.is_trigger = triggering_sample->is_trigger;
    preprocessed_sample.is_event = triggering_sample->is_event;
    preprocessed_sample.event_type = triggering_sample->event_type;

    /* Calculate processing time. */
    auto end_time = std::chrono::high_resolution_clock::now();
    preprocessed_sample.metadata.processing_time = std::chrono::duration<double_t>(end_time - start_time).count();

    /* Publish the preprocessed sample. */
    preprocessed_eeg_publisher->publish(preprocessed_sample);
  }
}

bool EegPreprocessor::is_pulse_feedback_received(double_t sample_time) {
  bool pulse_feedback_received = false;

  /* Remove all execution times from the queue that have happened before the current sample time. */
  while (!pulse_execution_times.empty() && sample_time >= pulse_execution_times.front()) {
    pulse_execution_times.pop();
    pulse_feedback_received = true;
  }
  return pulse_feedback_received;
}

void EegPreprocessor::process_sample(const std::shared_ptr<eeg_msgs::msg::Sample> msg) {
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

  /* Update EEG info with every new session OR if this is the first EEG sample received ever. */
  if (this->first_sample_of_session || this->first_sample_ever) {
    update_eeg_info(msg->metadata);

    /* Avoid checking for dropped samples on the first sample. */
    this->previous_time = UNSET_PREVIOUS_TIME;

    this->first_sample_ever = false;
  }

  /* Check that the preprocessor is enabled. */
  if (!this->enabled) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         2000,
                         "Preprocessor disabled, not processing EEG sample at time %.1f (s).",
                         sample_time);
    return;
  }

  check_dropped_samples(sample_time);

  /* Assert that module name is set - we shouldn't otherwise allow to enable the preprocessor. */
  assert(this->module_name != UNSET_STRING);

  if (this->reinitialize ||
      this->preprocessor_wrapper->get_state() == WrapperState::UNINITIALIZED ||
      this->first_sample_of_session) {

    initialize_module();
    reset_preprocessor_state();

    this->reinitialize = false;
  }
  this->first_sample_of_session = false;

  /* Check that the preprocessor module has not encountered an error. */
  if (this->preprocessor_wrapper->get_state() == WrapperState::ERROR) {
    this->preprocessor_state = PreprocessorState::MODULE_ERROR;

    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in preprocessor module.");
    return;
  }

  if (msg->is_trigger) {
    RCLCPP_INFO(this->get_logger(), "Registered trigger at: %.1f (s).", sample_time);
  }

  if (msg->is_event) {
    RCLCPP_INFO(this->get_logger(), "Registered event at: %.1f (s).", sample_time);
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
     Each sample takes sampling_period time. */
  if (look_ahead_samples > 0) {
    request.processing_time = sample_time + (look_ahead_samples * this->sampling_period);
    RCLCPP_DEBUG(this->get_logger(), 
                 "Deferring processing for sample at %.4f (s) until %.4f (s) (look-ahead: %d samples)",
                 sample_time, request.processing_time, look_ahead_samples);
  } else {
    /* If look-ahead is 0 or negative, no look-ahead is needed, process immediately. */
    request.processing_time = sample_time;
  }
  
  /* Add to deferred processing queue. */
  this->deferred_processing_queue.push(request);
  
  /* Check requests once more so that a new request that was just added can be processed immediately if needed. */
  process_ready_deferred_requests(sample_time);
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
