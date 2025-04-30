#include <chrono>
#include <filesystem>

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

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_DECIDER_NAME = "example";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

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

  /* Is mTMS device enabled. */
  auto mtms_device_enabled_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  mtms_device_enabled_descriptor.description = "Is mTMS device enabled";
  mtms_device_enabled_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  this->declare_parameter("mtms-device-enabled", false, mtms_device_enabled_descriptor);

  this->get_parameter("mtms-device-enabled", this->mtms_device_enabled);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Minimum pulse interval: %.1f (s)", this->minimum_intertrial_interval);
  RCLCPP_INFO(this->get_logger(), "  Dropped samples per second threshold: %d", this->dropped_sample_threshold);
  RCLCPP_INFO(this->get_logger(), "  Timing latency threshold: %.1f (ms)", 1000 * this->timing_latency_threshold);
  RCLCPP_INFO(this->get_logger(), "  mTMS device %s", this->mtms_device_enabled ? "enabled" : "not enabled");

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

  /* Subscriber for preprocessed EEG data. */
  this->preprocessed_eeg_subscriber = create_subscription<eeg_msgs::msg::PreprocessedSample>(
    EEG_PREPROCESSED_TOPIC,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegDecider::process_preprocessed_sample, this, _1));

  /* Subscriber for raw EEG data. Used only when preprocessor is disabled. */
  this->raw_eeg_subscriber = create_subscription<eeg_msgs::msg::Sample>(
    EEG_RAW_TOPIC,
    /* TODO: Should the queue be 1 samples long to make it explicit if we are too slow? */
    EEG_QUEUE_LENGTH,
    std::bind(&EegDecider::process_raw_sample, this, _1));

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegDecider::handle_set_active_project, this, _1));

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
  this->set_decider_enabled_service = this->create_service<project_interfaces::srv::SetDeciderEnabled>(
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

  /* Action client for performing mTMS trials, only if mTMS device is available. */
  if (this->mtms_device_enabled) {
    this->perform_trial_client = rclcpp_action::create_client<mtms_trial_interfaces::action::PerformTrial>(
      this, "/trial/perform");

    while (!perform_trial_client->wait_for_action_server(2s)) {
      RCLCPP_INFO(get_logger(), "Action /trial/perform not available, waiting...");
    }
  }

  /* Service client for timed trigger. */
  this->timed_trigger_client = this->create_client<pipeline_interfaces::srv::RequestTimedTrigger>("/pipeline/timed_trigger");

  while (!timed_trigger_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /pipeline/timed_trigger not available, waiting...");
  }

  /* Initialize variables. */
  this->decider_wrapper = std::make_unique<DeciderWrapper>(logger);

  this->sample_buffer = RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>>();
  this->sensory_stimulus = pipeline_interfaces::msg::SensoryStimulus();

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

    /* Reset the decider state when the session is stopped. */
    reset_decider_state();
  }
}

void EegDecider::handle_timing_latency(const std::shared_ptr<pipeline_interfaces::msg::TimingLatency> msg) {
  this->timing_latency = msg->latency;
}

void EegDecider::update_eeg_info(const eeg_msgs::msg::PreprocessedSampleMetadata& msg) {
  this->sampling_frequency = msg.sampling_frequency;
  this->num_of_eeg_channels = msg.num_of_eeg_channels;
  this->num_of_emg_channels = msg.num_of_emg_channels;

  this->sampling_period = 1.0 / this->sampling_frequency;
}

void EegDecider::initialize_module() {
  if (this->working_directory == UNSET_STRING ||
      this->module_name == UNSET_STRING) {

    RCLCPP_INFO(this->get_logger(), "Not initializing, decider module unset.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "");

  /* Print underlined, bolded text. */
  std::string text_str = "Loading decider: " + this->module_name;
  std::wstring underline_str(text_str.size(), L'–');
  RCLCPP_INFO(this->get_logger(), "%s%s%s", bold_on.c_str(), text_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());

  RCLCPP_INFO(this->get_logger(), "");

  std::vector<pipeline_interfaces::msg::SensoryStimulus> initial_sensory_stimuli;

  this->decider_wrapper->initialize_module(
    PROJECTS_DIRECTORY,
    this->working_directory,
    this->module_name,
    this->num_of_eeg_channels,
    this->num_of_emg_channels,
    this->sampling_frequency,
    initial_sensory_stimuli);

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

  /* Send the initial sensory stimuli to the presenter. */
  for (auto& sensory_stimulus : initial_sensory_stimuli) {
    this->sensory_stimulus_publisher->publish(sensory_stimulus);
  }
}

/* Note: This method is only relevant in the mTMS context. */
void EegDecider::precompute_trials() {
  /* XXX: Naming is a bit confusing here. */
  auto trials = this->decider_wrapper->get_targets();
  auto num_of_trials = trials.size();

  if (num_of_trials == 0) {
    RCLCPP_INFO(this->get_logger(), "No trials to pre-compute.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Pre-computing %s%zu%s trials.", bold_on.c_str(), num_of_trials, bold_off.c_str());

  for (auto targets : trials) {
    auto trial = mtms_trial_interfaces::msg::Trial();

    trial.targets = targets;

    auto num_of_targets = targets.size();
    trial.pulse_times_since_trial_start = std::vector<double_t>(num_of_targets, 0.0);

    trial.analyze_mep = false;

    auto timing = mtms_trial_interfaces::msg::TrialTiming();

    timing.desired_start_time = 0.0;
    timing.allow_late = true;

    auto config = mtms_trial_interfaces::msg::TrialConfig();

    config.voltage_tolerance_proportion_for_precharging = 0.0;
    config.recharge_after_trial = true;
    config.use_pulse_width_modulation_approximation = true;

    config.dry_run = true;

    trial.timing = timing;
    trial.config = config;

    /* Use a dummy decision time for pre-computed trials. */
    double_t decision_time = 0.0;

    this->trial_queue.push({trial, decision_time});
  }
}

/* Helpers */

std::string EegDecider::goal_id_to_string(const rclcpp_action::GoalUUID &uuid) {
  std::ostringstream oss;
  for (auto byte : uuid) {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  }
  return oss.str();
}

/* Action clients */

/* Note: This method is only called in the mTMS context. */
void EegDecider::perform_trial(const mtms_trial_interfaces::msg::Trial& trial, double decision_time) {
  if (!this->mtms_device_enabled) {
    RCLCPP_ERROR(this->get_logger(), "mTMS device not enabled, cannot perform trial.");
  }
  auto goal = mtms_trial_interfaces::action::PerformTrial::Goal();
  goal.trial = trial;

  auto send_goal_options = rclcpp_action::Client<mtms_trial_interfaces::action::PerformTrial>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this, trial, decision_time](std::shared_ptr<rclcpp_action::ClientGoalHandle<mtms_trial_interfaces::action::PerformTrial>> goal_handle) {
    this->goal_response_callback(goal_handle, trial, decision_time);
  };
  send_goal_options.result_callback = std::bind(&EegDecider::trial_performed_callback, this, std::placeholders::_1);

  perform_trial_client->async_send_goal(goal, send_goal_options);

  this->performing_trial = true;
}

void EegDecider::goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<mtms_trial_interfaces::action::PerformTrial>> goal_handle, const mtms_trial_interfaces::msg::Trial& trial, double decision_time) {
  if (goal_handle) {
    GoalMetadata metadata = {trial, decision_time};
    auto goal_id_str = goal_id_to_string(goal_handle->get_goal_id());
    this->goal_to_metadata_map[goal_id_str] = metadata;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
  }
}

/* Note: This method is only called in the mTMS context. */
void EegDecider::trial_performed_callback(const rclcpp_action::ClientGoalHandle<mtms_trial_interfaces::action::PerformTrial>::WrappedResult &result) {
  this->performing_trial = false;

  auto trial_result = result.result->trial_result;
  auto actual_start_time = trial_result.actual_start_time;

  RCLCPP_INFO(this->get_logger(), "Trial performed at: %.5f (s)", actual_start_time);
  /* TODO: Send the actual start time to the preprocessor. */

  auto goal_id_str = goal_id_to_string(result.goal_id);
  auto map_entry = this->goal_to_metadata_map.find(goal_id_str);

  if (map_entry == this->goal_to_metadata_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Could not find the corresponding metadata for the goal handle");
    return;
  }

  GoalMetadata metadata = map_entry->second;
  this->goal_to_metadata_map.erase(map_entry);

  bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success;

  RCLCPP_INFO(this->get_logger(), " ");
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Trial %sfailed%s", bold_on.c_str(), bold_off.c_str());
    RCLCPP_INFO(this->get_logger(), " ");

    /* If the trial failed, return early without computing the latency. */
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Trial %ssucceeded%s", bold_on.c_str(), bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  /* If the trial was a dry run, return early without computing the latency. */
  auto trial_config = metadata.trial.config;
  if (trial_config.dry_run) {
    RCLCPP_INFO(this->get_logger(), " ");
    return;
  }

  auto decision_time = metadata.decision_time;
  auto earliest_start_time = result.result->trial_result.earliest_start_time;

  /* Calculate the time difference between the earliest possible start time for the trial and the time based on which
     the decision was made. */
  double_t time_difference = earliest_start_time - decision_time;
  RCLCPP_INFO(this->get_logger(), "  - End-to-end latency of the trial: %s%.1f%s (ms)", bold_on.c_str(), time_difference * 1000, bold_off.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  /* Publish latency ROS message. */
  auto msg = pipeline_interfaces::msg::TimingLatency();
  msg.latency = time_difference;

  this->timing_latency_publisher->publish(msg);
}

/* Service clients */

void EegDecider::request_timed_trigger(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request) {
  using ServiceResponseFuture = rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    this->timed_trigger_callback(future);
  };

  auto future_result = this->timed_trigger_client->async_send_request(request, response_received_callback);

  this->processing_timed_trigger = true;
}

void EegDecider::timed_trigger_callback(rclcpp::Client<pipeline_interfaces::srv::RequestTimedTrigger>::SharedFutureWithRequest future) {
  auto result = future.get().second;
  if (!result->success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send timed trigger.");
  }
  this->processing_timed_trigger = false;
}

/* Initialization and reset functions */

void EegDecider::reset_decider_state() {
  this->decider_state = this->enabled ? DeciderState::READY : DeciderState::WAITING_FOR_ENABLED;
}

void EegDecider::empty_trial_queue() {
  while (!this->trial_queue.empty()) {
    this->trial_queue.pop();
  }
}

void EegDecider::handle_preprocessor_enabled(const std::shared_ptr<std_msgs::msg::Bool> msg) {
  this->preprocessor_enabled = msg->data;

  if (this->preprocessor_enabled) {
    RCLCPP_INFO(this->get_logger(), "Reading %spreprocessed%s EEG data.", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Reading %sraw%s EEG data.", bold_on.c_str(), bold_off.c_str());
  }
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
      const std::shared_ptr<project_interfaces::srv::SetDeciderEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDeciderEnabled::Response> response) {

  response->success = set_decider_enabled(request->enabled);;
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

/* File-system related functions */

bool EegDecider::change_working_directory(const std::string path) {
  this->working_directory = path;

  /* Check that the directory exists. */
  if (!std::filesystem::exists(this->working_directory) || !std::filesystem::is_directory(this->working_directory)) {
    RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s.", path.c_str());
    return false;
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

  /* List all .py files in the current working directory. */
  auto path = std::filesystem::current_path();
  for (const auto &entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".py") {
      modules.push_back(entry.path().stem().string());
    }
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

  /* Collect working directory and its subdirectories. */
  std::vector<std::filesystem::path> directories;
  directories.push_back(std::filesystem::path(this->working_directory));

  for (const auto& entry : std::filesystem::recursive_directory_iterator(this->working_directory)) {
    if (std::filesystem::is_directory(entry)) {
      directories.push_back(entry.path());
    }
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

void EegDecider::process_raw_sample(const std::shared_ptr<eeg_msgs::msg::Sample> msg) {
  /* Only process raw sample if preprocessor is bypassed. */
  if (this->preprocessor_enabled) {
    return;
  }

  /* Copy the raw sample to a preprocessed sample message. There might be a cleaner way to do this. */
  auto preprocessed_msg = std::make_shared<eeg_msgs::msg::PreprocessedSample>();
  preprocessed_msg->eeg_data = msg->eeg_data;
  preprocessed_msg->emg_data = msg->emg_data;

  preprocessed_msg->time = msg->time;

  preprocessed_msg->is_trigger = msg->is_trigger;
  preprocessed_msg->is_event = msg->is_event;
  preprocessed_msg->event_type = msg->event_type;

  /* Always mark sample as valid if preprocessor is bypassed. */
  preprocessed_msg->valid = true;

  auto preprocessed_metadata = eeg_msgs::msg::PreprocessedSampleMetadata();
  preprocessed_metadata.sampling_frequency = msg->metadata.sampling_frequency;
  preprocessed_metadata.num_of_eeg_channels = msg->metadata.num_of_eeg_channels;
  preprocessed_metadata.num_of_emg_channels = msg->metadata.num_of_emg_channels;
  preprocessed_metadata.is_simulation = msg->metadata.is_simulation;
  preprocessed_metadata.system_time = msg->metadata.system_time;

  /* XXX: Use dummy value for processing time if not available. */
  preprocessed_metadata.processing_time = 0.0;

  preprocessed_msg->metadata = preprocessed_metadata;

  process_preprocessed_sample(preprocessed_msg);
}

void EegDecider::process_preprocessed_sample(const std::shared_ptr<eeg_msgs::msg::PreprocessedSample> msg) {
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
    empty_trial_queue();
    precompute_trials();

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
  if (msg->is_trigger) {
    handle_trigger_from_eeg_device(sample_time);
  }

  /* Append the sample to the buffer. */
  this->sample_buffer.append(msg);

  /* Only proceed if the buffer is full. This removes the inconvenience of handling partial buffers on the Python side. */
  if (!this->sample_buffer.is_full()) {
    return;
  }

  /* Infer whether the sample is an event. */
  bool is_event = false;
  uint16_t event_type = 0;

  auto [next_event_time, next_event_type] = this->decider_wrapper->get_next_event();

  /* Skip events that we are past by at least one sampling period. */
  while (sample_time - this->sampling_period >= next_event_time - this->TOLERANCE_S) {
    RCLCPP_INFO(this->get_logger(), "Current time (%.3f s) is past event at %.3f (s), skipping...", sample_time, next_event_time);

    this->decider_wrapper->pop_event();
    std::tie(next_event_time, next_event_type) = this->decider_wrapper->get_next_event();
  }

  /* If decider module defines events and we are past the event time, consider the current sample an event
     (and override the is_event flag from the message). */
  if (sample_time >= next_event_time - this->TOLERANCE_S) {
    is_event = true;
    event_type = next_event_type;

    RCLCPP_INFO(this->get_logger(), "Decider-defined event (time: %.4f s, type: %d) occurred at time %.4f (s)", next_event_time, next_event_type, sample_time);

    this->decider_wrapper->pop_event();

  } else {
    /* Otherwise, use the is_event flag from the message. */
    is_event = msg->is_event;
    event_type = msg->event_type;

    if (is_event) {
      RCLCPP_INFO(this->get_logger(), "Received event at time %.4f (s), event type: %d", sample_time, event_type);
    }
  }

  bool process_current_sample = false;

  /* If process on trigger is enabled, process if the sample includes a trigger. */
  bool is_trigger = msg->is_trigger;
  if (this->decider_wrapper->is_process_on_trigger_enabled() && is_trigger) {
    process_current_sample = true;
  }

  /* If processing interval is enabled, process every N samples, where N is defined on the Python side. */
  if (this->decider_wrapper->is_processing_interval_enabled()) {
    this->samples_since_last_processing++;

    if (this->samples_since_last_processing == this->decider_wrapper->get_processing_interval_in_samples()) {
      process_current_sample = true;
    }
  }

  /* Always processing if the sample is considered an event. */
  if (is_event) {
    process_current_sample = true;
  }

  /* If neither condition is met, return early. */
  if (!process_current_sample) {
    return;
  }
  this->samples_since_last_processing = 0;

  /* Determine if we are ready for a trial. */
  auto time_since_previous_trial = sample_time - this->previous_stimulation_time;
  auto has_minimum_intertrial_interval_passed = std::isnan(this->previous_stimulation_time) ||
                                                time_since_previous_trial >= this->minimum_intertrial_interval;
  auto ready_for_trial = !performing_trial &&
                         !processing_timed_trigger &&
                         this->trial_queue.empty() &&
                         has_minimum_intertrial_interval_passed;

  /* Process the sample. */
  auto [success, trial, timed_trigger, request_sensory_stimulus] = this->decider_wrapper->process(
    this->sensory_stimulus,
    this->sample_buffer,
    sample_time,
    ready_for_trial,
    is_trigger,
    is_event,
    event_type);

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
  double_t decider_processing_time = std::chrono::duration<double_t>(end_time - start_time).count();

  /* Combine both trials (for mTMS device) and timed triggers (for other TMS devices). */
  bool is_decision_positive = trial || timed_trigger;

  /* Create decision info, but only publish in Decider if the pathway doesn't reach the Trigger Timer. */
  auto decision_info = pipeline_interfaces::msg::DecisionInfo();
  decision_info.stimulate = is_decision_positive;

  /* Ultimately, Trigger Timer knows if the decision is feasible. Use a dummy value here. */
  decision_info.feasible = true;

  decision_info.decision_time = sample_time;
  decision_info.decider_latency = decider_processing_time;
  decision_info.preprocessor_latency = msg->metadata.processing_time;

  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Time sample_time_rcl(msg->metadata.system_time);
  double_t total_latency = now.seconds() - sample_time_rcl.seconds();

  decision_info.total_latency = total_latency;

  /* Check timing latency threshold.

     XXX: Enabled only when using NeuroSimo without the mTMS device. The reason is that without the mTMS device,
       latency is periodically and automatically measured by the pipeline (as described in the NeuroSimo article),
       hence it is feasible to disable stimulation when it exceeds a threshold.

       However, when the mTMS device is enabled, latency is measured only when a pulse is delivered. Even though
       the latest measured latency is also in that case propagated into this->timing_latency, it won't be updated
       before a new pulse is delivered, and therefore this check, when failing, prevents all future pulses. */
  if (this->timing_latency > this->timing_latency_threshold && !this->mtms_device_enabled) {
    this->decision_info_publisher->publish(decision_info);

    RCLCPP_ERROR(this->get_logger(), "Timing latency (%.1f ms) exceeds threshold (%.1f ms), ignoring stimulation request.", this->timing_latency * 1000, this->timing_latency_threshold * 1000);
    return;
  }

  /* Check if a trial or timed trigger has already been requested. */
  bool is_already_stimulating = this->performing_trial || this->processing_timed_trigger;

  /* Check that the decider is not already stimulating. */
  if (is_decision_positive && is_already_stimulating) {
    this->decision_info_publisher->publish(decision_info);

    RCLCPP_ERROR(this->get_logger(), "Stimulation requested but already performing trial or timed trigger, ignoring request.");
    return;
  }

  /* Check that the minimum pulse interval is respected. */
  if (is_decision_positive && !has_minimum_intertrial_interval_passed) {
    this->decision_info_publisher->publish(decision_info);

    RCLCPP_ERROR(this->get_logger(), "Stimulation requested but minimum intertrial interval (%.1f s) not respected (time since previous stimulation: %.3f s), ignoring request.",
                 this->minimum_intertrial_interval,
                 time_since_previous_trial);
    return;
  }

  /* In case of a positive stimulation decision including a timed trigger, the decision-making pathway extends to
     Trigger Timer; hence, only publish the decision here if it is not a timed trigger. */
  if (!timed_trigger) {
    this->decision_info_publisher->publish(decision_info);
  }

  /* Add trial to the queue if requested. */
  if (trial) {
    this->trial_queue.push({*trial, sample_time});

    /* Update the previous stimulation time. */
    this->previous_stimulation_time = sample_time;
  }

  /* Send timed trigger if requested. */
  if (timed_trigger) {
    double_t trigger_time = timed_trigger->time;

    auto request = std::make_shared<pipeline_interfaces::srv::RequestTimedTrigger::Request>();
    request->timed_trigger = *timed_trigger;
    request->decision_time = sample_time;
    request->system_time_for_sample = msg->metadata.system_time;
    request->preprocessor_latency = msg->metadata.processing_time;
    request->decider_latency = decider_processing_time;

    this->request_timed_trigger(request);

    RCLCPP_INFO(this->get_logger(), "Timing trigger at time %.3f (s).", trigger_time);

    /* Update the previous stimulation time. */
    this->previous_stimulation_time = trigger_time;
  }

  /* Request sensory stimulus if requested. */
  if (request_sensory_stimulus) {
    RCLCPP_INFO(this->get_logger(), "Requesting sensory stimulus at time %.3f (s).", sample_time);

    this->sensory_stimulus_publisher->publish(this->sensory_stimulus);
  }
}

void EegDecider::spin() {
  auto base_interface = this->get_node_base_interface();

  while (rclcpp::ok()) {
    rclcpp::spin_some(base_interface);

    if (!this->trial_queue.empty() && !this->performing_trial) {
      auto num_of_remaining_trials = this->trial_queue.size();

      auto [trial, decision_time] = this->trial_queue.front();
      this->trial_queue.pop();

      this->perform_trial(trial, decision_time);
      this->log_trial(trial, num_of_remaining_trials);
    }
  }
}

void EegDecider::log_trial(const mtms_trial_interfaces::msg::Trial& trial, size_t num_of_remaining_trials) {
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "%s trial (remaining: %zu)", trial.config.dry_run ? "Pre-computing" : "Performing", num_of_remaining_trials);
  RCLCPP_INFO(this->get_logger(), "  - Targets:");

  auto targets = trial.targets;
  auto num_of_targets = targets.size();

  if (num_of_targets == 1) {
    RCLCPP_INFO(this->get_logger(), "      Single pulse: x = %d (mm), y = %d (mm), rotation angle = %d (deg), intensity = %d (V/m)",
                targets[0].displacement_x, targets[0].displacement_y, targets[0].rotation_angle, targets[0].intensity);

  } else if (num_of_targets == 2) {
    RCLCPP_INFO(this->get_logger(), "      Paired-pulse:");
    RCLCPP_INFO(this->get_logger(), "        Pulse #1: x = %d (mm), y = %d (mm), rotation angle = %d (deg), intensity = %d (V/m)",
                targets[0].displacement_x, targets[0].displacement_y, targets[0].rotation_angle, targets[0].intensity);
    RCLCPP_INFO(this->get_logger(), "        Pulse #2: x = %d (mm), y = %d (mm), rotation angle = %d (deg), intensity = %d (V/m)",
                targets[1].displacement_x, targets[1].displacement_y, targets[1].rotation_angle, targets[1].intensity);

  } else {
    RCLCPP_ERROR(this->get_logger(), "      Invalid number of pulses: %zu", num_of_targets);
  }

  if (!trial.config.dry_run) {
    RCLCPP_INFO(this->get_logger(), "  - Pulse times:");
    for (const auto& pulse_time : trial.pulse_times_since_trial_start) {
      RCLCPP_INFO(this->get_logger(), "    - %.3f (s)", pulse_time + trial.timing.desired_start_time);
    }
  }
}

int main(int argc, char *argv[]) {
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
