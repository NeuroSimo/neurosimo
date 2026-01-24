#include <chrono>
#include <filesystem>

#include "presenter_wrapper.h"
#include "presenter.h"

#include "realtime_utils/utils.h"
#include "filesystem_utils/filesystem_utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string SENSORY_STIMULUS_TOPIC = "/pipeline/sensory_stimulus";
const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_PRESENTER_NAME = "example";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

EegPresenter::EegPresenter() : Node("presenter"), logger(rclcpp::get_logger("presenter")) {
  /* Subscriber for EEG samples (to get session markers and time). */
  this->eeg_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    EEG_ENRICHED_TOPIC,
    EEG_QUEUE_LENGTH,
    std::bind(&EegPresenter::handle_eeg_sample, this, _1));

  /* Subscriber for sensory stimuli. */

  // Messages can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all = rclcpp::QoS(rclcpp::KeepAll())
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->sensory_stimulus_subscriber = create_subscription<pipeline_interfaces::msg::SensoryStimulus>(
    SENSORY_STIMULUS_TOPIC,
    qos_keep_all,
    std::bind(&EegPresenter::handle_sensory_stimulus, this, _1));

  /* Publisher for Python logs from presenter. */
  // Logs can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all_logs = rclcpp::QoS(rclcpp::KeepAll())
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->python_log_publisher = this->create_publisher<pipeline_interfaces::msg::LogMessages>(
    "/pipeline/presenter/log",
    qos_keep_all_logs);

  /* Initialize action server for component initialization */
  this->initialize_action_server = rclcpp_action::create_server<pipeline_interfaces::action::InitializePresenter>(
    this,
    "/pipeline/presenter/initialize",
    std::bind(&EegPresenter::handle_initialize_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&EegPresenter::handle_initialize_cancel, this, std::placeholders::_1),
    std::bind(&EegPresenter::handle_initialize_accepted, this, std::placeholders::_1));

  /* Initialize variables. */
  this->presenter_wrapper = std::make_unique<PresenterWrapper>(logger);
}

rclcpp_action::GoalResponse EegPresenter::handle_initialize_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const pipeline_interfaces::action::InitializePresenter::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received initialize goal: project='%s', module='%s', enabled=%s",
              goal->project_name.c_str(), goal->module_filename.c_str(), goal->enabled ? "true" : "false");

  // Accept all goals for now
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EegPresenter::handle_initialize_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel initialize goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EegPresenter::handle_initialize_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle) {
  // Execute the initialization in a new thread
  std::thread{std::bind(&EegPresenter::execute_initialize, this, std::placeholders::_1), goal_handle}.detach();
}

void EegPresenter::execute_initialize(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<pipeline_interfaces::action::InitializePresenter::Result>();

  // Set enabled state
  this->is_enabled = goal->enabled;

  // If not enabled, just mark as disabled and return early
  if (!goal->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Presenter marked as disabled: project=%s, module=%s",
                goal->project_name.c_str(), goal->module_filename.c_str());
    result->success = true;
    goal_handle->succeed(result);
    return;
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / goal->project_name;
  std::filesystem::path presenter_path = project_path / "presenter";
  std::filesystem::path module_path = presenter_path / goal->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  // Store initialization state
  this->initialized_project_name = goal->project_name;
  this->initialized_module_filename = goal->module_filename;
  this->initialized_working_directory = presenter_path;

  // Extract module name from filename (remove .py extension)
  std::string module_name = goal->module_filename;
  if (module_name.size() > 3 && module_name.substr(module_name.size() - 3) == ".py") {
    module_name = module_name.substr(0, module_name.size() - 3);
  }

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Loading presenter: %s.", module_name.c_str());
  RCLCPP_INFO(this->get_logger(), "");

  // Initialize the presenter wrapper
  bool success = this->presenter_wrapper->initialize_module(
    this->initialized_working_directory.string(),
    module_name);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize presenter module");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  // Publish initialization logs from Python constructor
  publish_python_logs(0.0, true);

  // Mark as initialized
  this->is_initialized = true;

  RCLCPP_INFO(this->get_logger(), "Presenter initialized successfully: project=%s, module=%s",
              goal->project_name.c_str(), goal->module_filename.c_str());

  result->success = true;
  goal_handle->succeed(result);
}

void EegPresenter::publish_python_logs(double sample_time, bool is_initialization) {
  auto logs = this->presenter_wrapper->get_and_clear_logs();
  
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

/* EEG sample handler (for session markers and time updates). */
void EegPresenter::handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Return early if the presenter is not enabled or initialized. */
  if (!this->is_enabled || !this->is_initialized) {
    return;
  }

  /* Handle session start marker. */
  if (msg->is_session_start) {
    RCLCPP_INFO(this->get_logger(), "Session started");
    this->session_started = true;

    /* Clear the sensory stimuli queue. */
    while (!this->sensory_stimuli.empty()) {
      this->sensory_stimuli.pop();
    }
  }

  /* Handle session end marker. */
  if (msg->is_session_end) {
    RCLCPP_INFO(this->get_logger(), "Session stopped");
    this->session_started = false;

    /* Reset the state of the existing module so that any windows etc. created by the Python module are closed. */
    this->presenter_wrapper->reset_module_state();
  }

  /* Update time for stimulus presentation. */
  if (this->session_started && !this->error_occurred) {
    update_time(msg->time);
  }
}

void EegPresenter::update_time(double_t time) {
  // Return early if no stimuli are queued
  if (this->sensory_stimuli.empty()) {
    return;
  }

  auto stimulus = this->sensory_stimuli.top();
  double_t stimulus_time = stimulus->time;

  // If the stimulus time is in the future, return early
  if (time <= stimulus_time) {
    return;
  }

  this->sensory_stimuli.pop();

  // Log the stimulus information
  RCLCPP_INFO(this->get_logger(), "Presenting stimulus:");
  RCLCPP_INFO(this->get_logger(), "  - Time: %.3f\n", stimulus_time);
  RCLCPP_INFO(this->get_logger(), "  - Type: %s\n", stimulus->type.c_str());

  if (!stimulus->parameters.empty()) {
    RCLCPP_INFO(this->get_logger(), "  - Parameters:");
    for (const auto &kv : stimulus->parameters) {
      RCLCPP_INFO(this->get_logger(), "    - %s: %s", kv.key.c_str(), kv.value.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "");  // blank line
  }

  // Process the stimulus using the presenter wrapper
  bool success = this->presenter_wrapper->process(*stimulus);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(stimulus_time, false);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Error presenting stimulus");
    this->error_occurred = true;
  }
}

void EegPresenter::handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg) {
  RCLCPP_INFO(this->get_logger(), "Received sensory stimulus (type: %s, time: %.3f)", msg->type.c_str(), msg->time);

  if (!this->is_enabled || !this->is_initialized) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Presenter not enabled, ignoring stimulus.");
    return;
  }
  if (this->error_occurred) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in presenter module. Ignoring stimulus.");
    return;
  }
  this->sensory_stimuli.push(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("presenter");

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

  auto node = std::make_shared<EegPresenter>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
