#include <chrono>
#include <cmath>

#include "trigger_simulator.h"

using namespace std::chrono;
using namespace std::placeholders;

const double_t HEARTBEAT_INTERVAL_SEC = 0.5;
const double_t INJECT_TRIGGER_TIMEOUT_SEC = 0.2;

TriggerSimulator::TriggerSimulator() : Node("trigger_simulator"), logger(rclcpp::get_logger("trigger_simulator")) {
  RCLCPP_INFO(this->get_logger(), "Initializing trigger simulator...");

  this->client_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
    "/neurosimo/eeg_simulator/inject_trigger",
    rclcpp::ServicesQoS(),
    this->client_callback_group);

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

void TriggerSimulator::publish_attempt_trace(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request>& request,
    uint8_t status,
    uint64_t system_time_trigger_timer_received_ns) {

  auto trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
  trace.session_id = request->session_id;
  trace.attempt_in_session = request->attempt_in_session;
  trace.status = status;
  trace.system_time_trigger_timer_received = system_time_trigger_timer_received_ns;
  trace.system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  this->attempt_trace_publisher->publish(trace);
}

TriggerSimulator::InjectTriggerCallResult TriggerSimulator::call_service_inject_trigger(
    double_t sample_time,
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request>& request) {

  auto inject_request = std::make_shared<neurosimo_eeg_interfaces::srv::InjectTrigger::Request>();
  inject_request->sample_time = sample_time;
  inject_request->session_id = request->session_id;
  inject_request->attempt_in_session = request->attempt_in_session;

  auto future = this->inject_trigger_client->async_send_request(inject_request);
  auto timeout = std::chrono::duration<double>(INJECT_TRIGGER_TIMEOUT_SEC);

  InjectTriggerCallResult result;
  if (future.wait_for(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) != std::future_status::ready) {
    result.status = InjectTriggerCallResult::Status::Timeout;
    return result;
  }

  result.status = InjectTriggerCallResult::Status::Success;
  result.response = future.get();
  return result;
}

void TriggerSimulator::reset_state() {
  this->trigger_request_service.reset();
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

  this->trigger_request_service = create_service<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>(
    "/neurosimo/pipeline/timed_trigger",
    std::bind(&TriggerSimulator::handle_request_timed_trigger, this, _1, _2));

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

  /* Capture timing at request receipt. */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t desired_pulse_time = request->reference_sample_time + request->trigger_offset;

  /* If not initialized, reject immediately. */
  if (!this->is_initialized) {
    RCLCPP_ERROR(logger, "Trigger request received but trigger simulator is not initialized");

    publish_attempt_trace(
      request,
      neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR,
      system_time_trigger_timer_received);

    response->success = false;
    return;
  }

  /* Enforce minimum trial interval. */
  if (!std::isnan(this->last_trigger_time)) {
    double_t time_since_last = desired_pulse_time - this->last_trigger_time;
    if (time_since_last < this->minimum_trial_interval) {
      RCLCPP_ERROR(logger, "Minimum trial interval (%.1f s) not met (%.3f s since last trigger), rejecting.",
                   this->minimum_trial_interval, time_since_last);

      publish_attempt_trace(
        request,
        neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR,
        system_time_trigger_timer_received);

      response->success = false;
      return;
    }
  }

  publish_attempt_trace(
    request,
    neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_SCHEDULED,
    system_time_trigger_timer_received);

  auto inject_result = call_service_inject_trigger(desired_pulse_time, request);

  if (inject_result.status == InjectTriggerCallResult::Status::Timeout) {
    RCLCPP_ERROR(logger, "InjectTrigger service call timed out (%.0f ms)", INJECT_TRIGGER_TIMEOUT_SEC * 1000);

    this->publish_health_status(neurosimo_system_interfaces::msg::ComponentHealth::DEGRADED,
                                "EEG simulator inject_trigger service timed out");

    publish_attempt_trace(
      request,
      neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR,
      system_time_trigger_timer_received);

    response->success = false;
    return;
  }

  auto inject_response = inject_result.response;
  uint8_t rejection_reason = inject_response->rejection_reason;

  bool success = inject_response->success && rejection_reason == neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_NONE;

  /* Capture system time at the moment the injection was confirmed. */
  uint64_t system_time_hardware_fired = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::high_resolution_clock::now().time_since_epoch()).count();

  auto attempt_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();
  attempt_trace.session_id = request->session_id;
  attempt_trace.attempt_in_session = request->attempt_in_session;
  attempt_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;

  /* Technically not completely true, but close enough for our purposes. */
  attempt_trace.system_time_trigger_timer_finished = system_time_hardware_fired;

  if (success) {
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_FIRED;
    attempt_trace.system_time_hardware_fired = system_time_hardware_fired;
    attempt_trace.latency_corrected_time_at_firing = desired_pulse_time;

    this->last_trigger_time = desired_pulse_time;
    response->success = true;

    RCLCPP_INFO(logger, "Trigger injected at sample time %.4f s", desired_pulse_time);
  } else {
    attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR;

    std::string rejection_reason_str;
    switch (rejection_reason) {
      case neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_TOO_LATE:
        rejection_reason_str = "TOO_LATE";
        attempt_trace.status = neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_TOO_LATE;
        break;
      case neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_NOT_PLAYING:
        rejection_reason_str = "NOT_PLAYING";
        break;
      case neurosimo_eeg_interfaces::srv::InjectTrigger::Response::REJECTION_INTERNAL_ERROR:
        rejection_reason_str = "INTERNAL_ERROR";
        break;
      default:
        rejection_reason_str = inject_response->success ? "UNKNOWN (inject reported success but with a rejection reason)"
                                                          : "UNKNOWN";
        break;
    }

    RCLCPP_ERROR(logger,
      "InjectTrigger rejected for desired pulse time %.4f s: %s (simulator playback time: %.4f s, error: %.4f s).",
      desired_pulse_time, rejection_reason_str.c_str(), inject_response->current_playback_time,
      desired_pulse_time - inject_response->current_playback_time);

    response->success = false;
  }

  this->attempt_trace_publisher->publish(attempt_trace);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TriggerSimulator>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();

  return 0;
}
