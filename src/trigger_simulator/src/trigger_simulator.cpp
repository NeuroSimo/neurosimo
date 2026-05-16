#include <chrono>
#include <cmath>

#include "trigger_simulator.h"

using namespace std::chrono;
using namespace std::placeholders;

const double_t HEARTBEAT_INTERVAL_SEC = 0.5;
const double_t INJECT_TRIGGER_TIMEOUT_SEC = 0.1;

TriggerSimulator::TriggerSimulator() : Node("trigger_simulator"), logger(rclcpp::get_logger("trigger_simulator")) {
  RCLCPP_INFO(this->get_logger(), "Initializing trigger simulator...");

  /* Service for trigger request — same topic as trigger_timer so upstream callers are agnostic. */
  this->trigger_request_service = create_service<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>(
    "/neurosimo/pipeline/timed_trigger",
    std::bind(&TriggerSimulator::handle_request_timed_trigger, this, _1, _2));

  /* Service for initialization. */
  this->initialize_service = create_service<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator>(
    "/neurosimo/pipeline/trigger_simulator/initialize",
    std::bind(&TriggerSimulator::handle_initialize, this, _1, _2));

  /* Service for finalization. */
  this->finalize_service = create_service<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator>(
    "/neurosimo/pipeline/trigger_simulator/finalize",
    std::bind(&TriggerSimulator::handle_finalize, this, _1, _2));

  /* Publisher for attempt trace. */
  this->attempt_trace_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>(
    "/neurosimo/pipeline/attempt_trace",
    10);

  /* Create QoS profile for latched topics */
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

  /* Create heartbeat publisher */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/neurosimo/trigger_simulator/heartbeat",
    10);

  /* Create health publisher */
  this->health_publisher = this->create_publisher<neurosimo_system_interfaces::msg::ComponentHealth>(
    "/neurosimo/trigger_simulator/health",
    status_qos);

  /* Create heartbeat timer */
  this->heartbeat_timer = this->create_wall_timer(
    std::chrono::duration<double>(HEARTBEAT_INTERVAL_SEC),
    std::bind(&TriggerSimulator::publish_heartbeat, this));

  /* Client for injecting triggers into the EEG simulator */
  this->inject_trigger_client = this->create_client<neurosimo_eeg_interfaces::srv::InjectTrigger>(
    "/neurosimo/eeg_simulator/inject_trigger");

  /* Publish initial READY state */
  this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::READY, "");

  RCLCPP_INFO(this->get_logger(), "Trigger simulator initialized");
}

void TriggerSimulator::publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void TriggerSimulator::publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = neurosimo_system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void TriggerSimulator::reset_state() {
  this->is_initialized = false;
  this->session_id.clear();
  this->minimum_trial_interval = 0.0;
  this->last_trigger_time = std::numeric_limits<double_t>::quiet_NaN();
}

void TriggerSimulator::handle_initialize(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Initializing trigger simulator for session");

  reset_state();

  /* Reset health status to a good state at the start of a new session */
  this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::READY, "");

  /* Store session configuration */
  this->session_id = std::vector<uint8_t>(request->session_id.begin(), request->session_id.end());
  this->minimum_trial_interval = request->minimum_trial_interval;

  /* Check that the inject_trigger service is available */
  if (!this->inject_trigger_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "InjectTrigger service not available on eeg_simulator");
    response->success = false;
    this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::ERROR,
                                "EEG simulator inject_trigger service not available");
    reset_state();
    return;
  }

  this->is_initialized = true;

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Minimum trial interval (s): %.1f", this->minimum_trial_interval);
  RCLCPP_INFO(this->get_logger(), " ");

  RCLCPP_INFO(this->get_logger(), "Trigger simulator initialized successfully");
  response->success = true;

  this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::READY, "");
}

void TriggerSimulator::handle_finalize(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator::Request> /* request */,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator::Response> response) {

  reset_state();

  RCLCPP_INFO(this->get_logger(), "Trigger simulator finalized successfully");
  response->success = true;
}

void TriggerSimulator::handle_request_timed_trigger(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Response> response) {

  std::lock_guard<std::mutex> lock(handler_mutex);

  /* Capture timing at request receipt. */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t desired_pulse_time = request->reference_sample_time + request->trigger_offset;

  /* If not initialized, reject immediately. */
  if (!this->is_initialized) {
    RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 5000, "Trigger request received but trigger simulator is not initialized");

    auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
    attempt_trace.session_id = request->session_id;
    attempt_trace.attempt_in_session = request->attempt_in_session;
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR;
    attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
    attempt_trace.system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    this->attempt_trace_publisher->publish(attempt_trace);

    response->success = false;
    return;
  }

  /* Enforce minimum trial interval. */
  if (!std::isnan(this->last_trigger_time)) {
    double_t time_since_last = desired_pulse_time - this->last_trigger_time;
    if (time_since_last < this->minimum_trial_interval) {
      RCLCPP_ERROR(logger, "Minimum trial interval (%.1f s) not met (%.3f s since last trigger), rejecting.",
                   this->minimum_trial_interval, time_since_last);

      auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
      attempt_trace.session_id = request->session_id;
      attempt_trace.attempt_in_session = request->attempt_in_session;
      attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR;
      attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
      attempt_trace.system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
      this->attempt_trace_publisher->publish(attempt_trace);

      response->success = false;
      return;
    }
  }

  /* Publish SCHEDULED trace before calling the simulator. */
  {
    auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
    attempt_trace.session_id = request->session_id;
    attempt_trace.attempt_in_session = request->attempt_in_session;
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_SCHEDULED;
    attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;

    /* Hardware-specific fields: set to 0.0 since not applicable in simulation. */
    attempt_trace.maximum_timing_offset = 0.0;
    attempt_trace.maximum_loopback_latency = 0.0;
    attempt_trace.trigger_to_pulse_delay = 0.0;
    attempt_trace.stimulation_horizon = 0.0;
    attempt_trace.strict_stimulation_horizon = 0.0;
    attempt_trace.loopback_latency_at_scheduling = 0.0;

    this->attempt_trace_publisher->publish(attempt_trace);
  }

  /* Call InjectTrigger on the eeg_simulator. */
  auto inject_request = std::make_shared<neurosimo_eeg_interfaces::srv::InjectTrigger::Request>();
  inject_request->sample_time = desired_pulse_time;
  inject_request->session_id = request->session_id;
  inject_request->attempt_in_session = request->attempt_in_session;

  auto future = this->inject_trigger_client->async_send_request(inject_request);

  /* Wait for the response with a short timeout. */
  auto timeout = std::chrono::duration<double>(INJECT_TRIGGER_TIMEOUT_SEC);
  auto status = future.wait_for(timeout);

  uint64_t system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(logger, "InjectTrigger service call timed out (%.0f ms)", INJECT_TRIGGER_TIMEOUT_SEC * 1000);

    this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::DEGRADED,
                                "EEG simulator inject_trigger service timed out");

    auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
    attempt_trace.session_id = request->session_id;
    attempt_trace.attempt_in_session = request->attempt_in_session;
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR;
    attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
    attempt_trace.system_time_trigger_timer_finished = system_time_trigger_timer_finished;
    this->attempt_trace_publisher->publish(attempt_trace);

    response->success = false;
    return;
  }

  auto inject_response = future.get();

  /* Capture system time at the moment the injection was confirmed. */
  uint64_t system_time_hardware_fired = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  /* Translate the response into an AttemptTrace. */
  auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
  attempt_trace.session_id = request->session_id;
  attempt_trace.attempt_in_session = request->attempt_in_session;
  attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
  attempt_trace.system_time_trigger_timer_finished = system_time_trigger_timer_finished;

  /* Hardware-specific fields: not applicable in simulation. */
  attempt_trace.maximum_timing_offset = 0.0;
  attempt_trace.maximum_loopback_latency = 0.0;
  attempt_trace.trigger_to_pulse_delay = 0.0;
  attempt_trace.stimulation_horizon = 0.0;
  attempt_trace.strict_stimulation_horizon = 0.0;
  attempt_trace.loopback_latency_at_scheduling = 0.0;

  if (inject_response->success) {
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_FIRED;
    attempt_trace.system_time_hardware_fired = system_time_hardware_fired;
    attempt_trace.latency_corrected_time_at_firing = desired_pulse_time;

    this->last_trigger_time = desired_pulse_time;
    response->success = true;

    RCLCPP_INFO(logger, "Trigger injected at sample time %.4f s", desired_pulse_time);
  } else {
    uint8_t rejection = inject_response->rejection_reason;

    if (rejection == neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_TOO_LATE) {
      attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_TOO_LATE;
      RCLCPP_WARN(logger, "Trigger injection rejected: too late (playback at %.4f s, requested %.4f s)",
                   inject_response->current_playback_time, desired_pulse_time);
    } else {
      attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR;

      if (rejection == neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_NOT_PLAYING) {
        RCLCPP_WARN(logger, "Trigger injection rejected: simulator not playing");
        this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::DEGRADED,
                                    "EEG simulator is not playing back data");
      } else {
        RCLCPP_ERROR(logger, "Trigger injection failed with rejection reason %d", rejection);
        this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::DEGRADED,
                                    "Trigger injection failed");
      }
    }

    response->success = false;
  }

  this->attempt_trace_publisher->publish(attempt_trace);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TriggerSimulator>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
