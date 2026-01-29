#include <chrono>
#include <thread>

#include "trigger_timer.h"
#include "labjack_manager.h"
#include "mock_labjack_manager.h"

#include "realtime_utils/utils.h"

using namespace std::chrono;

const double_t HEARTBEAT_INTERVAL_SEC = 0.5;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string TIMED_TRIGGER_SERVICE = "/pipeline/timed_trigger";
const std::string EEG_RAW_TOPIC = "/eeg/raw";

const double_t loopback_interval = 0.1;

const char* tms_trigger_fio = "FIO4";
const char* loopback_trigger_fio = "FIO5";

TriggerTimer::TriggerTimer() : Node("trigger_timer"), logger(rclcpp::get_logger("trigger_timer")) {
  /* Read ROS parameter: Maximum triggering error */
  auto triggering_tolerance_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  triggering_tolerance_descriptor.description = "The maximum triggering error (in seconds)";
  this->declare_parameter("triggering-tolerance", 0.0, triggering_tolerance_descriptor);
  this->get_parameter("triggering-tolerance", this->triggering_tolerance);

  /* Check that the triggering tolerance is non-negative. */
  if (this->triggering_tolerance < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Triggering tolerance must be non-negative.");
    rclcpp::shutdown();
    return;
  }

  /* Read ROS parameter: LabJack simulation mode */
  auto simulate_labjack_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  simulate_labjack_descriptor.description = "Simulate LabJack device when hardware is not available";
  simulate_labjack_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  this->declare_parameter("simulate-labjack", false, simulate_labjack_descriptor);
  this->get_parameter("simulate-labjack", this->simulate_labjack);

  /* Read ROS parameter: loopback latency threshold */
  auto loopback_latency_threshold_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  loopback_latency_threshold_descriptor.description = "The threshold for the loopback latency (in seconds) before stimulation is prevented";
  this->declare_parameter("loopback-latency-threshold", 0.005, loopback_latency_threshold_descriptor);
  this->get_parameter("loopback-latency-threshold", this->loopback_latency_threshold);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Triggering tolerance (ms): %.1f", 1000 * this->triggering_tolerance);
  RCLCPP_INFO(this->get_logger(), "  Loopback latency threshold: %.1f (ms)", 1000 * this->loopback_latency_threshold);
  RCLCPP_INFO(this->get_logger(), "  LabJack simulation: %s", this->simulate_labjack ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), " ");

  /* Subscriber for EEG raw data. */
  this->eeg_raw_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    10,
    std::bind(&TriggerTimer::handle_eeg_raw, this, _1));

  /* Service for trigger request. */
  this->trigger_request_service = create_service<pipeline_interfaces::srv::RequestTimedTrigger>(
    TIMED_TRIGGER_SERVICE,
    std::bind(&TriggerTimer::handle_request_timed_trigger, this, _1, _2));

  /* Service for initialization. */
  this->initialize_service = create_service<pipeline_interfaces::srv::InitializeTriggerTimer>(
    "/pipeline/trigger_timer/initialize",
    std::bind(&TriggerTimer::handle_initialize_trigger_timer, this, _1, _2));

  /* Service for finalization. */
  this->finalize_service = create_service<pipeline_interfaces::srv::FinalizeTriggerTimer>(
    "/pipeline/trigger_timer/finalize",
    std::bind(&TriggerTimer::handle_finalize_trigger_timer, this, _1, _2));

  /* Publisher for decision trace. */
  this->decision_trace_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionTrace>(
    "/pipeline/decision_trace",
    10);

  /* Publisher for timing latency. */
  this->loopback_latency_publisher = this->create_publisher<pipeline_interfaces::msg::LoopbackLatency>(
    "/pipeline/latency/trigger_loopback",
    10);

  /* Create QoS profile for latched topics */
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  /* Create heartbeat publisher */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/trigger_timer/heartbeat",
    10);

  /* Create health publisher */
  this->health_publisher = this->create_publisher<system_interfaces::msg::ComponentHealth>(
    "/trigger_timer/health",
    status_qos);

  /* Create heartbeat timer */
  this->heartbeat_timer = this->create_wall_timer(
    std::chrono::duration<double>(HEARTBEAT_INTERVAL_SEC),
    std::bind(&TriggerTimer::_publish_heartbeat, this));

  /* Publish initial READY state */
  this->_publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");
}

TriggerTimer::~TriggerTimer() {
  if (timer) {
    timer->cancel();
  }
  if (labjack_manager) {
    labjack_manager->stop();
  }
}

void TriggerTimer::_publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void TriggerTimer::_publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void TriggerTimer::attempt_labjack_connection() {
  if (labjack_manager) {
    labjack_manager->request_connection_attempt();
  }
}

void TriggerTimer::trigger_pulses_until_time(double_t sample_time) {
  double_t latency_corrected_time = sample_time + this->current_loopback_latency;

  /* Trigger all events until the given time. */
  while (!trigger_queue.empty() && trigger_queue.top()->timed_trigger.time <= latency_corrected_time) {
    auto request = trigger_queue.top();
    double_t scheduled_time = request->timed_trigger.time;
    double_t error = latency_corrected_time - scheduled_time;

    uint8_t status = pipeline_interfaces::msg::DecisionTrace::STATUS_REJECTED;

    /* Capture timing when hardware trigger is fired. */
    auto now = std::chrono::high_resolution_clock::now();
    uint64_t system_time_hardware_fired = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now.time_since_epoch()).count();

    /* Check that timing latency is within threshold. */
    if (this->current_loopback_latency <= this->loopback_latency_threshold) {
      RCLCPP_INFO(logger, "Triggering at scheduled time: %.4f (current time: %.4f, error: %.4f)",
                  scheduled_time, latency_corrected_time, error);

      if (labjack_manager && labjack_manager->trigger_output(tms_trigger_fio)) {
        status = pipeline_interfaces::msg::DecisionTrace::STATUS_FIRED;
      } else {
        RCLCPP_ERROR(logger, "Failed to trigger TMS trigger.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Timing latency (%.1f ms) exceeds threshold (%.1f ms) at triggering time, rejecting stimulation.",
                   this->current_loopback_latency * 1000, this->loopback_latency_threshold * 1000);

      /* Publish degraded health status */
      this->_publish_health_status(system_interfaces::msg::ComponentHealth::DEGRADED,
                                   "Timing latency exceeds threshold, stimulation rejected");
    }
    trigger_queue.pop();

    /* Publish second DecisionTrace (fired). */
    auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
    decision_trace.session_id = request->session_id;
    decision_trace.decision_id = request->decision_id;
    decision_trace.status = pipeline_interfaces::msg::DecisionTrace::STATUS_FIRED;
    decision_trace.system_time_hardware_fired = system_time_hardware_fired;
    decision_trace.sample_time_at_firing = sample_time;
    decision_trace.latency_corrected_time_at_firing = latency_corrected_time;
    decision_trace.loopback_latency_at_firing = this->current_loopback_latency;

    this->decision_trace_publisher->publish(decision_trace);
  }
}

void TriggerTimer::measure_loopback_latency(bool loopback_trigger, double_t sample_time) {
  /* Update current latency if loopback trigger is present. */
  if (loopback_trigger) {
    this->current_loopback_latency = sample_time - this->last_loopback_time;

    /* Publish loopback latency ROS message. */
    auto msg = pipeline_interfaces::msg::LoopbackLatency();
    msg.latency = this->current_loopback_latency;
  
    this->loopback_latency_publisher->publish(msg);  
  }

  /* Trigger loopback trigger at specific intervals. */
  double_t time_since_last_loopback = sample_time - this->last_loopback_time;
  if (time_since_last_loopback < loopback_interval) {
    return;
  }

  this->last_loopback_time = sample_time;

  if (!labjack_manager || !labjack_manager->trigger_output(loopback_trigger_fio)) {
    RCLCPP_ERROR(logger, "Failed to trigger loopback trigger.");
    return;
  }
}

void TriggerTimer::handle_eeg_raw(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  double_t sample_time = msg->time;

  // Update latest sample information for calculating stimulation horizon
  this->latest_sample_time = sample_time;

  std::lock_guard<std::mutex> lock(queue_mutex);

  trigger_pulses_until_time(sample_time);
  measure_loopback_latency(msg->loopback_trigger, sample_time);
}

void TriggerTimer::handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response) {

  /* Capture timing at request receipt. */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t trigger_time = request->timed_trigger.time;

  bool is_labjack_connected = labjack_manager && labjack_manager->is_connected();

  if (!is_labjack_connected) {
    RCLCPP_WARN(logger, "LabJack is not connected, rejecting trigger at time: %.4f (s).", trigger_time);
    response->success = false;

    /* Publish degraded health status */
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::DEGRADED,
                                 "LabJack device not connected");
  } else {
    std::lock_guard<std::mutex> lock(queue_mutex);
    trigger_queue.push(request);

    RCLCPP_INFO(logger, "Scheduled trigger at time: %.4f (s).", trigger_time);
    response->success = true;
  }

  /* Capture timing at request finish. */
  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    end_time.time_since_epoch()).count();

  /* Calculate stimulation horizon. */
  double_t stimulation_horizon = this->latest_sample_time - request->reference_sample_time;

  /* Determine status. */
  uint8_t status = is_labjack_connected ? pipeline_interfaces::msg::DecisionTrace::STATUS_SCHEDULED
                                        : pipeline_interfaces::msg::DecisionTrace::STATUS_REJECTED;

  /* Publish first DecisionTrace (scheduled or rejected). */
  auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
  decision_trace.session_id = request->session_id;
  decision_trace.decision_id = request->decision_id;
  decision_trace.status = status;
  decision_trace.stimulation_horizon = stimulation_horizon;
  decision_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
  decision_trace.system_time_trigger_timer_finished = system_time_trigger_timer_finished;

  this->decision_trace_publisher->publish(decision_trace);
}

void TriggerTimer::reset_state() {
  /* Cancel and reset the connection timer */
  if (timer) {
    timer->cancel();
    timer.reset();
  }

  /* Stop and reset LabJack manager */
  if (labjack_manager) {
    labjack_manager->stop();
    labjack_manager.reset();
  }

  /* Reset latency state */
  this->current_loopback_latency = 0.0;
  this->last_loopback_time = 0.0;

  /* Reset latest sample information */
  this->latest_sample_time = 0.0;
}

void TriggerTimer::handle_initialize_trigger_timer(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Initializing trigger timer for session");

  reset_state();

  /* Initialize LabJack manager. */
  if (this->simulate_labjack) {
    labjack_manager = std::make_unique<MockLabJackManager>(this->get_logger());
  } else {
    labjack_manager = std::make_unique<LabJackManager>(this->get_logger(), false);
  }
  labjack_manager->start();

  /* Set up a timer to signal connection attempts every second. */
  timer = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TriggerTimer::attempt_labjack_connection, this));

  /* Wait for LabJack connection - try for up to 30 seconds */
  auto start_time = std::chrono::steady_clock::now();
  bool connected = false;

  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(30)) {
    if (labjack_manager && labjack_manager->is_connected()) {
      connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!connected) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to LabJack device during initialization");
    response->success = false;

    /* Publish error health status */
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                 "Failed to connect to LabJack device during initialization");

    /* Clean up on failure */
    reset_state();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Trigger timer initialized successfully");
  response->success = true;

  /* Publish ready health status */
  this->_publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");
}

void TriggerTimer::handle_finalize_trigger_timer(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizeTriggerTimer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizeTriggerTimer::Response> response) {

  reset_state();

  RCLCPP_INFO(this->get_logger(), "Trigger timer finalized successfully");
  response->success = true;
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("trigger_timer");

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

  auto node = std::make_shared<TriggerTimer>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
