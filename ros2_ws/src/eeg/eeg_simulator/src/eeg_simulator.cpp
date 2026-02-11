#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <thread>
#include <string>
#include <filesystem>

#define XXH_INLINE_ALL
#include <xxhash.h>

#include "realtime_utils/utils.h"

#include <nlohmann/json.hpp>

#include "eeg_simulator.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string EEG_RAW_TOPIC = "/eeg/raw";

const milliseconds STREAMING_INTERVAL = 1ms;
/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;
/* TODO: Simulating the EEG device to the level of sending UDP packets not implemented on the C++
     side yet. For a previous Python reference implementation, see commit c0afb515b. */
EegSimulator::EegSimulator() : Node("eeg_simulator") {
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /* Create a single SubscriptionOptions object */
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;

  /* Publisher for EEG simulator heartbeat. */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/eeg_simulator/heartbeat",
    10);

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  /* Publisher for health. */
  this->health_publisher = this->create_publisher<system_interfaces::msg::ComponentHealth>(
    "/eeg_simulator/health",
    qos_persist_latest);

  this->global_config_subscriber = create_subscription<system_interfaces::msg::GlobalConfig>(
    "/global_configurator/config",
    qos_persist_latest,
    std::bind(&EegSimulator::handle_global_config, this, std::placeholders::_1),
    subscription_options);

  /* Publisher for streamer state. */
  data_source_state_publisher = this->create_publisher<system_interfaces::msg::DataSourceState>(
    "/eeg_simulator/state",
    qos_persist_latest);

  /* Initialize dataset manager. */
  this->dataset_manager_ = std::make_unique<DatasetManager>(this);

  /* Services for starting/stopping streaming. */
  this->start_streaming_service = this->create_service<eeg_interfaces::srv::StartStreaming>(
    "/eeg_simulator/streaming/start",
    std::bind(&EegSimulator::handle_start_streaming, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    callback_group);

  this->stop_streaming_service = this->create_service<eeg_interfaces::srv::StopStreaming>(
    "/eeg_simulator/streaming/stop",
    std::bind(&EegSimulator::handle_stop_streaming, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    callback_group);

  /* Publisher for EEG samples.

     NB: It is crucial to not use the shared, re-entrant callback group for this publisher, as EEG sample messages
       are time-critical and using the shared callback group seems to hinder the performance. */
  eeg_publisher = this->create_publisher<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    EEG_QUEUE_LENGTH);

  /* Timer to drive streaming - created/destroyed on demand to avoid CPU usage when idle. */
  this->stream_timer = nullptr;

  /* Create a timer for publishing heartbeat. */
  this->heartbeat_publisher_timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this] { publish_heartbeat(); },
      callback_group);

  /* Initialize action server for simulator initialization */
  this->initialize_action_server = rclcpp_action::create_server<eeg_interfaces::action::InitializeSimulatorStream>(
    this,
    "/eeg_simulator/initialize",
    std::bind(&EegSimulator::handle_initialize_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&EegSimulator::handle_initialize_cancel, this, std::placeholders::_1),
    std::bind(&EegSimulator::handle_initialize_accepted, this, std::placeholders::_1));

  /* Initialize client for aborting session */
  this->abort_session_client = this->create_client<system_interfaces::srv::AbortSession>("/session/abort");

  /* Publish initial streamer state. */
  publish_data_source_state();
}

void EegSimulator::publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void EegSimulator::publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void EegSimulator::abort_session() {
  auto request = std::make_shared<system_interfaces::srv::AbortSession::Request>();
  request->source = "eeg_simulator";

  auto result = this->abort_session_client->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Requested session abort: dataset finished (non-looping mode)");
}

rclcpp_action::GoalResponse EegSimulator::handle_initialize_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const eeg_interfaces::action::InitializeSimulatorStream::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Initializing simulator with dataset '%s' and start time %.3f",
              goal->dataset_filename.c_str(), goal->start_time);

  // Accept all goals for now
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EegSimulator::handle_initialize_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel initialize goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EegSimulator::handle_initialize_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle) {
  // Execute the initialization in a new thread
  std::thread{std::bind(&EegSimulator::execute_initialize, this, std::placeholders::_1), goal_handle}.detach();
}

void EegSimulator::execute_initialize(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<eeg_interfaces::action::InitializeSimulatorStream>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<eeg_interfaces::action::InitializeSimulatorStream::Result>();

  // Store initialization parameters
  this->initialized_project_name = goal->project_name;
  this->initialized_dataset_filename = goal->dataset_filename;
  this->initialized_start_time = goal->start_time;

  // Set the dataset and start time
  std::string data_directory = "projects/" + std::string(goal->project_name) + "/eeg_simulator/";
  auto [success, dataset_info, pulse_times] = this->dataset_manager_->get_dataset_info(goal->dataset_filename, data_directory);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get dataset info: %s", goal->dataset_filename.c_str());
    this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR, "Failed to get dataset info");
    goal_handle->abort(result);
    return;
  }
  this->dataset_info = dataset_info;
  this->pulse_times = pulse_times;

  // Load dataset into buffer
  this->data_source_state = system_interfaces::msg::DataSourceState::LOADING;
  publish_data_source_state();

  auto [load_success, error_msg] = this->dataset_manager_->load_dataset(goal->project_name, dataset_info, this->dataset_buffer);
  if (!load_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load dataset: %s", error_msg.c_str());
    this->data_source_state = system_interfaces::msg::DataSourceState::ERROR;
    this->error_message = error_msg;
    publish_data_source_state();
    this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR, "Failed to load dataset: " + error_msg);
    goal_handle->abort(result);
    return;
  }

  // Initialize streaming parameters
  this->sampling_period = 1.0 / this->dataset_info.sampling_frequency;

  this->current_index = 0;
  this->current_pulse_index = 0;
  this->time_offset = 0.0;

  /* Log pulse times info if present. */
  if (!this->pulse_times.empty()) {
    RCLCPP_INFO(this->get_logger(), "Dataset has %zu pulse times. First pulse at %.4f s.",
                this->pulse_times.size(), this->pulse_times[0]);
  }

  this->data_source_state = system_interfaces::msg::DataSourceState::READY;
  publish_data_source_state();

  this->publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");

  // Set the start time
  this->play_dataset_from = goal->start_time;

  // Mark as initialized
  this->is_initialized = true;

  RCLCPP_INFO(this->get_logger(), "Simulator initialized successfully: project=%s, dataset=%s, start_time=%.3f",
              goal->project_name.c_str(), goal->dataset_filename.c_str(), goal->start_time);

  result->stream_info.sampling_frequency = this->dataset_info.sampling_frequency;
  result->stream_info.num_eeg_channels = this->dataset_info.num_eeg_channels;
  result->stream_info.num_emg_channels = this->dataset_info.num_emg_channels;

  goal_handle->succeed(result);
}

void EegSimulator::publish_data_source_state() {
  system_interfaces::msg::DataSourceState msg;
  msg.state = this->data_source_state;
  this->data_source_state_publisher->publish(msg);
}

void EegSimulator::stop_streaming_timer() {
  if (this->stream_timer) {
    this->stream_timer->cancel();
    this->stream_timer = nullptr;
    RCLCPP_INFO(this->get_logger(), "Streaming timer stopped");
  }
}

void EegSimulator::handle_global_config(const std::shared_ptr<system_interfaces::msg::GlobalConfig> msg) {
  std::string project_name = msg->active_project;
  
  RCLCPP_INFO(this->get_logger(), "Global config received: active_project=%s", project_name.c_str());
  
  // Only process if active project has actually changed
  if (project_name == this->active_project_name) {
    return;
  }
  
  this->active_project_name = project_name;
  this->dataset_manager_->set_active_project(project_name);
  RCLCPP_INFO(this->get_logger(), "Active project changed to: %s", project_name.c_str());
}

void EegSimulator::handle_start_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StartStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StartStreaming::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received start streaming request");
  if (!this->is_initialized) {
    response->success = false;
    return;
  }
  this->session_start_time = this->get_clock()->now().seconds();
  this->session_sample_index = 0;
  this->is_session_start = true;
  this->session_data_fingerprint = 0;
  this->data_source_state = system_interfaces::msg::DataSourceState::RUNNING;
  
  /* Create the streaming timer to drive sample publication. */
  if (!this->stream_timer) {
    this->stream_timer = this->create_wall_timer(STREAMING_INTERVAL,
                                                 std::bind(&EegSimulator::stream_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Streaming timer started (interval: %ld ms)", STREAMING_INTERVAL.count());
  }
  
  response->success = true;
  publish_data_source_state();
}

void EegSimulator::handle_stop_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StopStreaming::Request> request,
      std::shared_ptr<eeg_interfaces::srv::StopStreaming::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received stop streaming request");

  if (this->data_source_state != system_interfaces::msg::DataSourceState::RUNNING) {
    RCLCPP_ERROR(this->get_logger(), "Not streaming, cannot stop streaming.");

    response->success = false;
    response->data_fingerprint = 0;
    return;
  }

  stop_streaming_timer();

  /* Store the final fingerprint before resetting state */
  response->data_fingerprint = this->session_data_fingerprint;
  RCLCPP_INFO(this->get_logger(), "Session data fingerprint: 0x%016lx", response->data_fingerprint);

  this->data_source_state = system_interfaces::msg::DataSourceState::READY;
  publish_data_source_state();

  response->success = true;
}


void EegSimulator::stream_timer_callback() {
  if (this->data_source_state != system_interfaces::msg::DataSourceState::RUNNING) {
    return;
  }

  if (std::isnan(this->session_start_time)) {
    return;
  }

  double_t elapsed = this->get_clock()->now().seconds() - this->session_start_time;
  double_t target_time = this->play_dataset_from + elapsed;

  bool success = this->publish_until(target_time);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish samples until target time. Stopping streaming.");
    this->data_source_state = system_interfaces::msg::DataSourceState::ERROR;
    stop_streaming_timer();
    publish_data_source_state();
    return;
  }
}

/* Publish a single sample at the given index with the specified session flags. */
bool EegSimulator::publish_single_sample(size_t sample_index, bool is_session_start) {
  if (sample_index >= dataset_buffer.size()) {
    RCLCPP_ERROR(this->get_logger(), "Sample index %zu out of bounds (max: %zu)", sample_index, dataset_buffer.size() - 1);
    return false;
  }

  const std::vector<double_t>& data = dataset_buffer[sample_index];

  uint8_t total_channels = this->dataset_info.num_eeg_channels + this->dataset_info.num_emg_channels;
  if (total_channels > static_cast<int>(data.size())) {
    RCLCPP_ERROR(this->get_logger(), "Total # of EEG and EMG channels (%d) exceeds # of channels in data (%zu)", total_channels, data.size());
    return false;
  }

  /* Compute sample time from index and sampling frequency. */
  double_t sample_time = sample_index * this->sampling_period;
  double_t time = sample_time + this->time_offset;

  /* Create the sample message. */
  eeg_interfaces::msg::Sample msg;

  msg.eeg.insert(msg.eeg.end(), data.begin(), data.begin() + this->dataset_info.num_eeg_channels);
  msg.emg.insert(msg.emg.end(), data.begin() + this->dataset_info.num_eeg_channels, data.end());

  msg.is_session_start = is_session_start;

  msg.time = time;

  /* Set the sample index. */
  msg.sample_index = this->session_sample_index;
  this->session_sample_index++;

  /* Mark the sample as valid by default. The preprocessor can later mark it as invalid if needed. */
  msg.valid = true;

  /* Check if a pulse should be delivered at this sample time. */
  msg.pulse_trigger = false;
  if (!this->pulse_times.empty() && this->current_pulse_index < this->pulse_times.size()) {
    double_t next_pulse_time = this->pulse_times[this->current_pulse_index];
    if (sample_time >= next_pulse_time) {
      msg.pulse_trigger = true;
      this->current_pulse_index++;

      RCLCPP_INFO(this->get_logger(), "Pulse delivered at %.4f s (pulse %zu/%zu).",
                  sample_time, this->current_pulse_index, this->pulse_times.size());

      /* Log next pulse time if there is one. */
      if (this->current_pulse_index < this->pulse_times.size()) {
        RCLCPP_INFO(this->get_logger(), "Next pulse at %.4f s.",
                    this->pulse_times[this->current_pulse_index]);
      } else {
        RCLCPP_INFO(this->get_logger(), "No more pulses in this dataset.");
      }
    }
  }

  /* Set the system time when the sample was published. */
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t system_time_data_source_published = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();

  msg.system_time_data_source_published = system_time_data_source_published;

  /* Update data fingerprint with EEG data */
  if (!msg.eeg.empty()) {
    this->session_data_fingerprint = XXH64(msg.eeg.data(),
                                            msg.eeg.size() * sizeof(double),
                                            this->session_data_fingerprint);
  }
  
  /* Update data fingerprint with EMG data */
  if (!msg.emg.empty()) {
    this->session_data_fingerprint = XXH64(msg.emg.data(),
                                            msg.emg.size() * sizeof(double),
                                            this->session_data_fingerprint);
  }

  eeg_publisher->publish(msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       2000,
                       "Published sample at %.1f s.", time);

  return true;
}

/* Publish samples from start_index until (but not including) the first sample that is after until_time.
   Returns true if the samples were published successfully. */
bool EegSimulator::publish_until(double_t until_time) {
  if (dataset_buffer.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No data available to publish");
    return false;
  }

  size_t samples_published = 0;

  while (true) {
    /* Compute sample time from index and sampling frequency. */
    double_t sample_time = this->current_index * this->sampling_period;
    double_t adjusted_time = sample_time + this->time_offset;

    // If we've reached a sample after until_time, stop
    if (adjusted_time > until_time) {
      break;
    }

    // Publish this sample with the captured flag values
    bool success = publish_single_sample(this->current_index, this->is_session_start);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish sample at index %zu", current_index);
      break;
    }

    // Check if this is the last sample in the dataset (non-looping mode)
    bool is_last_sample = !this->dataset_info.loop && 
                          (this->current_index == dataset_buffer.size() - 1);
    if (is_last_sample) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset. Marking session end.");

      stop_streaming_timer();

      this->data_source_state = system_interfaces::msg::DataSourceState::READY;
      publish_data_source_state();

      abort_session();

      return true;
    }

    // Clear the session start marker after publishing.
    this->is_session_start = false;

    // Move to next sample
    this->current_index = (this->current_index + 1) % dataset_buffer.size();

    // If we've wrapped around and loop is enabled, update time offset
    if (this->dataset_info.loop && this->current_index == 0) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset, looping back to beginning.");
      double_t dataset_duration = dataset_buffer.size() * this->sampling_period;
      this->time_offset = this->time_offset + dataset_duration;
    }

    samples_published++;

    // Prevent infinite loops in case of edge cases. TODO: What is this for?
    if (samples_published > dataset_buffer.size() * 2) {
      RCLCPP_WARN(this->get_logger(), "Published too many samples, breaking to prevent infinite loop.");
      break;
    }
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("eeg_simulator");

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

  auto node = std::make_shared<EegSimulator>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
