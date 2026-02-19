#include <chrono>
#include <filesystem>

#include "presenter_wrapper.h"
#include "presenter.h"

#include "realtime_utils/utils.h"
#include "filesystem_utils/filesystem_utils.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "system_interfaces/msg/component_health.hpp"

using namespace std::chrono;

const double_t HEARTBEAT_INTERVAL_SEC = 0.5;
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

  /* Initialize service server for component initialization */
  this->initialize_service_server = this->create_service<pipeline_interfaces::srv::InitializePresenter>(
    "/pipeline/presenter/initialize",
    std::bind(&EegPresenter::handle_initialize_presenter, this, std::placeholders::_1, std::placeholders::_2));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<pipeline_interfaces::srv::FinalizePresenter>(
    "/pipeline/presenter/finalize",
    std::bind(&EegPresenter::handle_finalize_presenter, this, std::placeholders::_1, std::placeholders::_2));

  /* Service client for session abort. */
  this->abort_session_client = this->create_client<system_interfaces::srv::AbortSession>("/session/abort");

  while (!abort_session_client->wait_for_service(2s)) {
    RCLCPP_INFO(this->get_logger(), "Service /session/abort not available, waiting...");
  }

  /* Create QoS profile for latched topics */
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  /* Create heartbeat publisher */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/presenter/heartbeat",
    10);

  /* Create health publisher */
  this->health_publisher = this->create_publisher<system_interfaces::msg::ComponentHealth>(
    "/presenter/health",
    status_qos);

  /* Create heartbeat timer */
  this->heartbeat_timer = this->create_wall_timer(
    std::chrono::duration<double>(HEARTBEAT_INTERVAL_SEC),
    std::bind(&EegPresenter::_publish_heartbeat, this));

  /* Publish initial READY state */
  this->_publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");

  /* Initialize variables. */
  this->presenter_wrapper = std::make_unique<PresenterWrapper>(logger);
}

void EegPresenter::handle_initialize_presenter(
  const std::shared_ptr<pipeline_interfaces::srv::InitializePresenter::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::InitializePresenter::Response> response) {
  
  // Set enabled state
  this->is_enabled = request->enabled;

  // If not enabled, just mark as disabled and return early
  if (!request->enabled) {
    this->is_enabled = false;
    RCLCPP_INFO(this->get_logger(), "Presenter marked as disabled: project=%s, module=%s",
                request->project_name.c_str(), request->module_filename.c_str());
    response->success = true;
    return;
  }

  // Change to project working directory
  std::filesystem::path project_path = std::filesystem::path(PROJECTS_DIRECTORY) / request->project_name;
  std::filesystem::path presenter_path = project_path / "presenter";
  std::filesystem::path module_path = presenter_path / request->module_filename;

  if (!std::filesystem::exists(module_path)) {
    RCLCPP_ERROR(this->get_logger(), "Module file does not exist: %s", module_path.c_str());
    response->success = false;
    return;
  }

  // Store initialization state
  this->initialized_project_name = request->project_name;
  this->initialized_module_filename = request->module_filename;
  this->initialized_working_directory = presenter_path;

  // Change working directory to the module directory
  if (!filesystem_utils::change_working_directory(presenter_path.string(), this->get_logger())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s", presenter_path.string().c_str());
    response->success = false;
    return;
  }

  // Extract module name from filename (remove .py extension)
  std::string module_name = request->module_filename;
  if (module_name.size() > 3 && module_name.substr(module_name.size() - 3) == ".py") {
    module_name = module_name.substr(0, module_name.size() - 3);
  }

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Loading presenter: %s.", module_name.c_str());
  RCLCPP_INFO(this->get_logger(), " ");

  // Initialize the presenter wrapper
  bool success = this->presenter_wrapper->initialize_module(
    this->initialized_working_directory.string(),
    module_name,
    request->subject_id);

  // Publish initialization logs from Python constructor
  publish_python_logs(pipeline_interfaces::msg::LogMessage::PHASE_INITIALIZATION, 0.0);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize presenter module");
    response->success = false;
    return;
  }

  // Mark as initialized
  this->is_initialized = true;

  /* Publish ready health status */
  this->_publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");

  RCLCPP_INFO(this->get_logger(), "Presenter initialized successfully: project=%s, module=%s",
              request->project_name.c_str(), request->module_filename.c_str());

  response->success = true;
}

void EegPresenter::handle_finalize_presenter(
  const std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Request> /* request */,
  std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Received finalize request");

  /* Drain and publish any remaining logs. */
  if (this->presenter_wrapper) {
    this->presenter_wrapper->drain_logs();
    publish_python_logs(pipeline_interfaces::msg::LogMessage::PHASE_FINALIZATION, 0.0);
  }

  // Finalize the presenter module if initialized
  if (this->is_initialized && this->presenter_wrapper) {
    bool success = this->presenter_wrapper->reset_module_state();
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to reset presenter module state");
      response->success = false;
      return;
    }
  }

  // Reset state
  this->is_initialized = false;
  this->is_enabled = false;
  this->error_occurred = false;
  this->initialized_project_name = UNSET_STRING;
  this->initialized_module_filename = UNSET_STRING;
  this->initialized_working_directory = "";

  // Clear sensory stimuli queue
  while (!this->sensory_stimuli.empty()) {
    this->sensory_stimuli.pop();
  }

  RCLCPP_INFO(this->get_logger(), "Presenter finalized successfully");
  response->success = true;
}

void EegPresenter::publish_python_logs(uint8_t phase, double sample_time) {
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
    log_msg.phase = phase;
    
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
  // Return early if the presenter is not enabled or initialized.
  if (!this->is_enabled || !this->is_initialized) {
    return;
  }

  // Check that no error has occurred.
  if (this->error_occurred) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in presenter module. Ignoring EEG sample.");
    return;
  }

  // Return early if no stimuli are queued
  if (this->sensory_stimuli.empty()) {
    return;
  }

  auto stimulus = this->sensory_stimuli.top();
  double_t stimulus_time = stimulus->time;

  // If the stimulus time is in the future, return early
  if (msg->time <= stimulus_time) {
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
    RCLCPP_INFO(this->get_logger(), " ");  // blank line
  }

  // Process the stimulus using the presenter wrapper
  bool success = this->presenter_wrapper->process(*stimulus);

  /* Publish buffered Python logs after process() completes to avoid interfering with timing */
  publish_python_logs(pipeline_interfaces::msg::LogMessage::PHASE_RUNTIME, stimulus_time);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Error presenting stimulus");
    this->error_occurred = true;
    this->abort_session("Presenter Python error");
    return;
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

void EegPresenter::_publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void EegPresenter::_publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void EegPresenter::abort_session(const std::string& reason) {
  auto request = std::make_shared<system_interfaces::srv::AbortSession::Request>();
  request->source = "presenter";
  request->reason = reason;

  auto result = this->abort_session_client->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Requested session abort: %s", reason.c_str());
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
