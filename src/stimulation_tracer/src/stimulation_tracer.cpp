#include <chrono>
#include <algorithm>

#include "stimulation_tracer.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string ATTEMPT_TRACE_TOPIC = "/neurosimo/pipeline/attempt_trace";
const std::string ATTEMPT_TRACE_FINAL_TOPIC = "/neurosimo/pipeline/attempt_trace/final";

StimulationTracer::StimulationTracer() : Node("stimulation_tracer"), logger(rclcpp::get_logger("stimulation_tracer")) {
  /* Subscriber for attempt traces (from Decider and TriggerTimer). */
  this->attempt_trace_subscriber = create_subscription<neurosimo_pipeline_interfaces::msg::AttemptTrace>(
    ATTEMPT_TRACE_TOPIC,
    100,
    std::bind(&StimulationTracer::handle_attempt_trace, this, _1));

  /* Publisher for attempt trace updates. */
  this->attempt_trace_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>(
    ATTEMPT_TRACE_TOPIC,
    10);

  /* Publisher for final merged attempt traces. */
  this->attempt_trace_final_publisher = this->create_publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>(
    ATTEMPT_TRACE_FINAL_TOPIC,
    10);

  /* Initialize service server */
  this->initialize_service_server = this->create_service<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer>(
    "/neurosimo/pipeline/stimulation_tracer/initialize",
    std::bind(&StimulationTracer::handle_initialize_stimulation_tracer, this, std::placeholders::_1, std::placeholders::_2));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer>(
    "/neurosimo/pipeline/stimulation_tracer/finalize",
    std::bind(&StimulationTracer::handle_finalize_stimulation_tracer, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "StimulationTracer initialized");
}

StimulationTracer::~StimulationTracer() {
  // Cleanup if needed
}

void StimulationTracer::handle_initialize_stimulation_tracer(
  const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer::Request> request,
  std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer::Response> response) {

  /* Store the session ID, data source, and mark as initialized. */
  this->current_session_id = request->session_id;
  this->is_initialized = true;

  /* Clear any existing traces. */
  this->attempt_traces.clear();

  response->success = true;
}

void StimulationTracer::handle_finalize_stimulation_tracer(
  const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer::Request> request,
  std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer::Response> response) {

  /* Verify session ID matches. */
  if (request->session_id != this->current_session_id) {
    RCLCPP_WARN(this->get_logger(), "Finalize called with mismatched session_id");
    response->success = false;
    return;
  }

  /* Clear traces and reset state. */
  this->attempt_traces.clear();
  this->is_initialized = false;
  this->current_session_id = {};

  RCLCPP_INFO(this->get_logger(), "StimulationTracer finalized");

  response->success = true;
}

void StimulationTracer::handle_attempt_trace(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::AttemptTrace> msg) {
  RCLCPP_INFO(this->get_logger(), "Received attempt trace: attempt_in_session=%lu, status=%u, requested_stimulation_time=%f",
              msg->attempt_in_session, msg->status, msg->requested_stimulation_time);

  /* Skip if not initialized. */
  if (!this->is_initialized) {
    return;
  }

  /* Only accept traces from current session. */
  if (msg->session_id != this->current_session_id) {
    return;
  }

  /* Store attempt trace in memory keyed by attempt_in_session. */
  uint64_t attempt_in_session = msg->attempt_in_session;

  this->attempt_traces[attempt_in_session].push_back(*msg);

  /* If this trace has a terminal status, finalize the attempt. */
  if (this->is_terminal_status(msg->status)) {
    finalize_attempt(attempt_in_session);
  }
}

void StimulationTracer::finalize_attempt(uint64_t attempt_in_session) {
  RCLCPP_INFO(this->get_logger(), "Finalizing attempt: attempt_in_session=%lu", attempt_in_session);

  /* Find all traces for this attempt. */
  auto it = this->attempt_traces.find(attempt_in_session);
  if (it == this->attempt_traces.end() || it->second.empty()) {
    return;
  }

  /* Merge all traces into one final attempt trace. */
  auto final_trace = neurosimo_pipeline_interfaces::msg::AttemptTrace();

  for (const auto& trace : it->second) {
    /* Copy all non-zero/non-empty fields from each trace. */

    /* Metadata */
    if (trace.session_id != std::array<uint8_t, 16>{}) {
      final_trace.session_id = trace.session_id;
    }
    if (trace.attempt_in_session != 0) {
      final_trace.attempt_in_session = trace.attempt_in_session;
    }

    /* Status - keep the latest (highest priority) status */
    if (trace.status > final_trace.status) {
      final_trace.status = trace.status;
    }

    /* Requested stimulation time */
    if (trace.requested_stimulation_time != 0.0) final_trace.requested_stimulation_time = trace.requested_stimulation_time;
    if (trace.reference_time != 0.0) final_trace.reference_time = trace.reference_time;

    /* Attempt information (from Decider) */
    if (trace.attempt_timing != 0) final_trace.attempt_timing = trace.attempt_timing;
    if (!trace.attempt_type.empty()) final_trace.attempt_type = trace.attempt_type;
    if (trace.decision_id != 0) final_trace.decision_id = trace.decision_id;

    /* TriggerTimer fields */
    if (trace.stimulation_horizon != 0.0) final_trace.stimulation_horizon = trace.stimulation_horizon;
    if (trace.strict_stimulation_horizon != 0.0) final_trace.strict_stimulation_horizon = trace.strict_stimulation_horizon;
    if (trace.system_time_trigger_timer_received != 0) final_trace.system_time_trigger_timer_received = trace.system_time_trigger_timer_received;
    if (trace.system_time_trigger_timer_finished != 0) final_trace.system_time_trigger_timer_finished = trace.system_time_trigger_timer_finished;
    if (trace.system_time_hardware_fired != 0) final_trace.system_time_hardware_fired = trace.system_time_hardware_fired;
    if (trace.loopback_latency_at_scheduling != 0.0) final_trace.loopback_latency_at_scheduling = trace.loopback_latency_at_scheduling;
    if (trace.latency_corrected_time_at_firing != 0.0) final_trace.latency_corrected_time_at_firing = trace.latency_corrected_time_at_firing;
    if (trace.maximum_timing_offset != 0.0) final_trace.maximum_timing_offset = trace.maximum_timing_offset;
    if (trace.maximum_loopback_latency != 0.0) final_trace.maximum_loopback_latency = trace.maximum_loopback_latency;
    if (trace.trigger_to_pulse_delay != 0.0) final_trace.trigger_to_pulse_delay = trace.trigger_to_pulse_delay;

    /* Observed pulse fields */
    if (trace.actual_stimulation_time != 0.0) final_trace.actual_stimulation_time = trace.actual_stimulation_time;
    if (trace.actual_stimulation_sample_index != 0) final_trace.actual_stimulation_sample_index = trace.actual_stimulation_sample_index;
    if (trace.timing_offset != 0.0) final_trace.timing_offset = trace.timing_offset;

    /* Trial validity - if any trace marks it invalid, the final trace is invalid */
    if (trace.invalid_trial) final_trace.invalid_trial = true;
  }

  /* Publish the final merged trace. */
  this->attempt_trace_final_publisher->publish(final_trace);

  RCLCPP_INFO(this->get_logger(), "Finalized attempt trace: attempt_in_session=%lu, status=%u, invalid_trial=%s",
              final_trace.attempt_in_session, final_trace.status, final_trace.invalid_trial ? "true" : "false");

  /* Remove from memory. */
  this->attempt_traces.erase(it);
}

bool StimulationTracer::is_terminal_status(uint8_t status) {
  /* An attempt is finalized when it reaches one of these terminal statuses. */
  if (status == neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_LOOPBACK_LATENCY_EXCEEDED ||
      status == neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_TOO_LATE ||
      status == neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_PULSE_PROCESSED ||
      status == neurosimo_pipeline_interfaces::msg::AttemptTrace::STATUS_ERROR) {
    return true;
  }
  return false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StimulationTracer>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
