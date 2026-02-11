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
  RCLCPP_INFO(this->get_logger(), "Initializing trigger timer...");

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
  std::lock_guard<std::mutex> lock(handler_mutex);

  double_t sample_time = msg->time;

  // Update latest sample information for calculating stimulation horizon
  this->latest_sample_time = sample_time;

  // Store sample time and corresponding system time for time estimation
  this->stored_sample_time = sample_time;
  this->stored_system_time = std::chrono::high_resolution_clock::now();

  measure_loopback_latency(msg->loopback_trigger, sample_time);
}

double_t TriggerTimer::estimate_current_sample_time() {
  if (this->stored_sample_time == 0.0) {
    return 0.0;
  }

  auto current_system_time = std::chrono::high_resolution_clock::now();
  auto elapsed_system_time = std::chrono::duration_cast<std::chrono::duration<double>>(
    current_system_time - this->stored_system_time);

  return this->stored_sample_time + elapsed_system_time.count() + this->current_loopback_latency;
}

TriggerTimer::SchedulingResult TriggerTimer::schedule_trigger_with_timer(
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request) {

  double_t desired_pulse_time = request->timed_trigger.time;
  double_t trigger_time = desired_pulse_time - this->trigger_to_pulse_delay;
  double_t estimated_current_time = estimate_current_sample_time();

  if (estimated_current_time == 0.0) {
    RCLCPP_WARN(logger, "No sample time reference available yet, cannot estimate timing.");
    return SchedulingResult::ERROR;
  }

  /* Check if a trigger is already scheduled */
  if (active_trigger_timer && !active_trigger_timer->is_canceled()) {
    RCLCPP_ERROR(logger, "Trigger already scheduled, rejecting new trigger request for pulse time %.4f (trigger time %.4f)", 
                 desired_pulse_time, trigger_time);
    return SchedulingResult::ERROR;
  }

  /* Check that loopback latency is within threshold. */
  if (this->current_loopback_latency > this->maximum_loopback_latency) {
    RCLCPP_ERROR(this->get_logger(), "Loopback latency (%.1f ms) exceeds maximum (%.1f ms), rejecting stimulation.",
                 this->current_loopback_latency * 1000, this->maximum_loopback_latency * 1000);

    /* Publish degraded health status */
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::DEGRADED,
                                 "Loopback latency exceeds threshold, stimulation rejected");
    return SchedulingResult::LOOPBACK_LATENCY_EXCEEDED;
  }

  double_t time_until_trigger = trigger_time - estimated_current_time;

  /* Check if trigger time is in the past */
  if (time_until_trigger < 0.0) {
    double_t timing_offset = -time_until_trigger;
    
    /* Reject if timing offset exceeds maximum allowed */
    if (timing_offset > this->maximum_timing_offset) {
      RCLCPP_ERROR(logger, "Trigger time %.4f (pulse time %.4f) is too late (current estimated: %.4f, error: %.4f ms exceeds maximum %.4f ms), rejecting trigger.",
                   trigger_time, desired_pulse_time, estimated_current_time, timing_offset * 1000, this->maximum_timing_offset * 1000);
      return SchedulingResult::TOO_LATE;
    }
    
    /* Within tolerance, trigger immediately */
    RCLCPP_WARN(logger, "Trigger time %.4f (pulse time %.4f) is in the past (current estimated: %.4f, error: %.4f ms within maximum %.4f ms), triggering immediately.",
                trigger_time, desired_pulse_time, estimated_current_time, timing_offset * 1000, this->maximum_timing_offset * 1000);
    time_until_trigger = 0.0;
  }

  // Create a one-shot timer to fire the trigger
  active_trigger_timer = this->create_wall_timer(
    std::chrono::duration<double>(time_until_trigger),
    [this, request, desired_pulse_time]() {
      std::lock_guard<std::mutex> lock(handler_mutex);

      double_t trigger_time = desired_pulse_time - this->trigger_to_pulse_delay;
      double_t estimated_current_time = estimate_current_sample_time();
      double_t error = estimated_current_time - trigger_time;

      uint8_t status = pipeline_interfaces::msg::DecisionTrace::STATUS_ERROR;

      /* Capture timing when hardware trigger is fired. */
      auto now = std::chrono::high_resolution_clock::now();
      uint64_t system_time_hardware_fired = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();

      RCLCPP_INFO(logger, "Triggering at scheduled time: %.4f for pulse time: %.4f (estimated current time: %.4f, error: %.4f)",
                  trigger_time, desired_pulse_time, estimated_current_time, error);

      if (labjack_manager && labjack_manager->trigger_output(tms_trigger_fio)) {
        status = pipeline_interfaces::msg::DecisionTrace::STATUS_FIRED;
      } else {
        RCLCPP_ERROR(logger, "Failed to trigger TMS trigger.");
      }

      /* Publish second DecisionTrace (fired). */
      auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
      decision_trace.session_id = request->session_id;
      decision_trace.decision_id = request->decision_id;
      decision_trace.status = status;
      decision_trace.system_time_hardware_fired = system_time_hardware_fired;
      decision_trace.latency_corrected_time_at_firing = estimated_current_time;

      this->decision_trace_publisher->publish(decision_trace);

      /* Cancel timer to make it one-shot. */
      active_trigger_timer->cancel();
    });

  return SchedulingResult::SCHEDULED;
}

void TriggerTimer::handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response) {

  std::lock_guard<std::mutex> lock(handler_mutex);

  /* Capture timing at request receipt. */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t trigger_time = request->timed_trigger.time;

  bool is_labjack_connected = labjack_manager && labjack_manager->is_connected();
  SchedulingResult result;

  if (!is_labjack_connected) {
    RCLCPP_WARN(logger, "LabJack is not connected, rejecting trigger at time: %.4f (s).", trigger_time);

    /* Publish degraded health status */
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::DEGRADED,
                                 "LabJack device not connected");
    result = SchedulingResult::ERROR;
  } else {
    result = schedule_trigger_with_timer(request);
  }

  /* Capture timing at request finish. */
  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    end_time.time_since_epoch()).count();

  /* Calculate stimulation horizons. */
  double_t strict_stimulation_horizon = estimate_current_sample_time() - request->reference_sample_time;
  double_t stimulation_horizon = std::max(0.0, strict_stimulation_horizon - this->maximum_timing_offset);

  /* Determine status. */
  uint8_t status;
  switch (result) {
    case SchedulingResult::SCHEDULED:
      status = pipeline_interfaces::msg::DecisionTrace::STATUS_SCHEDULED;
      break;
    case SchedulingResult::TOO_LATE:
      status = pipeline_interfaces::msg::DecisionTrace::STATUS_TOO_LATE;
      break;
    case SchedulingResult::LOOPBACK_LATENCY_EXCEEDED:
      status = pipeline_interfaces::msg::DecisionTrace::STATUS_LOOPBACK_LATENCY_EXCEEDED;
      break;
    case SchedulingResult::ERROR:
      status = pipeline_interfaces::msg::DecisionTrace::STATUS_ERROR;
      break;
  }

  /* Publish first DecisionTrace (scheduled or rejected). */
  auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
  decision_trace.session_id = request->session_id;
  decision_trace.decision_id = request->decision_id;
  decision_trace.status = status;
  decision_trace.stimulation_horizon = stimulation_horizon;
  decision_trace.strict_stimulation_horizon = strict_stimulation_horizon;
  decision_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
  decision_trace.system_time_trigger_timer_finished = system_time_trigger_timer_finished;
  decision_trace.loopback_latency_at_scheduling = this->current_loopback_latency;
  decision_trace.maximum_timing_offset = this->maximum_timing_offset;
  decision_trace.maximum_loopback_latency = this->maximum_loopback_latency;
  decision_trace.trigger_to_pulse_delay = this->trigger_to_pulse_delay;

  this->decision_trace_publisher->publish(decision_trace);

  /* Set response success based on scheduling result. */
  bool success = (result == SchedulingResult::SCHEDULED);
  response->success = success;
  if (success) {
    RCLCPP_INFO(logger, "Scheduled trigger for pulse time: %.4f (s), trigger time: %.4f (s).", 
                request->timed_trigger.time, request->timed_trigger.time - this->trigger_to_pulse_delay);
  } else {
    RCLCPP_WARN(logger, "Failed to schedule trigger for pulse time: %.4f (s).", request->timed_trigger.time);
  }
}

void TriggerTimer::reset_state() {
  /* Cancel and reset the connection timer */
  if (timer) {
    timer->cancel();
    timer.reset();
  }

  /* Cancel and reset active trigger timer */
  if (active_trigger_timer) {
    active_trigger_timer->cancel();
    active_trigger_timer.reset();
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

  /* Reset stored time tracking */
  this->stored_sample_time = 0.0;
  this->stored_system_time = std::chrono::high_resolution_clock::time_point();
}

void TriggerTimer::handle_initialize_trigger_timer(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Initializing trigger timer for session");

  reset_state();

  /* Set configuration from request */
  this->maximum_timing_offset = request->maximum_timing_offset;
  this->maximum_loopback_latency = request->maximum_loopback_latency;
  this->trigger_to_pulse_delay = request->trigger_to_pulse_delay;
  this->simulate_labjack = request->simulate_labjack;

  /* Validate the maximum timing offset is non-negative */
  if (this->maximum_timing_offset < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Maximum timing offset must be non-negative");
    response->success = false;
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                 "Invalid configuration: maximum timing offset must be non-negative");
    return;
  }

  /* Validate the trigger to pulse delay is non-negative */
  if (this->trigger_to_pulse_delay < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Trigger to pulse delay must be non-negative");
    response->success = false;
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                 "Invalid configuration: trigger to pulse delay must be non-negative");
    return;
  }

  /* Validate the maximum loopback latency */
  if (this->maximum_loopback_latency <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Maximum loopback latency must be positive");
    response->success = false;
    this->_publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                 "Invalid configuration: maximum loopback latency must be positive");
    return;
  }

  /* Log the configuration */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Timing tolerance (ms): %.1f", 1000 * this->maximum_timing_offset);
  RCLCPP_INFO(this->get_logger(), "  Maximum loopback latency (ms): %.1f", 1000 * this->maximum_loopback_latency);
  RCLCPP_INFO(this->get_logger(), "  Trigger to pulse delay (ms): %.1f", 1000 * this->trigger_to_pulse_delay);
  RCLCPP_INFO(this->get_logger(), "  LabJack simulation: %s", this->simulate_labjack ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), " ");

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

  /* Wait for LabJack connection - try for up to 1 second */
  auto start_time = std::chrono::steady_clock::now();
  bool connected = false;

  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(1)) {
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
                                 "Failed to connect to LabJack device");

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
