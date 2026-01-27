#include <chrono>
#include <algorithm>

#include "stimulation_tracer.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string DECISION_TRACE_TOPIC = "/pipeline/decision_trace";
const std::string DECISION_TRACE_FINAL_TOPIC = "/pipeline/decision_trace/final";

StimulationTracer::StimulationTracer() : Node("stimulation_tracer"), logger(rclcpp::get_logger("stimulation_tracer")) {
  /* Subscriber for EEG raw data. */
  this->eeg_sample_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    10,
    std::bind(&StimulationTracer::handle_eeg_sample, this, _1));

  /* Subscriber for decision traces. */
  this->decision_trace_subscriber = create_subscription<pipeline_interfaces::msg::DecisionTrace>(
    DECISION_TRACE_TOPIC,
    100,
    std::bind(&StimulationTracer::handle_decision_trace, this, _1));

  /* Publisher for decision trace updates. */
  this->decision_trace_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionTrace>(
    DECISION_TRACE_TOPIC,
    10);

  /* Publisher for final merged decision traces. */
  this->decision_trace_final_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionTrace>(
    DECISION_TRACE_FINAL_TOPIC,
    10);

  /* Initialize service server */
  this->initialize_service_server = this->create_service<pipeline_interfaces::srv::InitializeStimulationTracer>(
    "/pipeline/stimulation_tracer/initialize",
    std::bind(&StimulationTracer::handle_initialize_stimulation_tracer, this, std::placeholders::_1, std::placeholders::_2));

  /* Finalize service server */
  this->finalize_service_server = this->create_service<pipeline_interfaces::srv::FinalizeStimulationTracer>(
    "/pipeline/stimulation_tracer/finalize",
    std::bind(&StimulationTracer::handle_finalize_stimulation_tracer, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "StimulationTracer initialized");
}

StimulationTracer::~StimulationTracer() {
  // Cleanup if needed
}

void StimulationTracer::handle_initialize_stimulation_tracer(
  const std::shared_ptr<pipeline_interfaces::srv::InitializeStimulationTracer::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::InitializeStimulationTracer::Response> response) {

  /* Store the session ID, data source, and mark as initialized. */
  this->current_session_id = request->session_id;
  this->data_source = request->data_source;
  this->is_initialized = true;

  /* Clear any existing decision traces. */
  this->decision_traces.clear();

  RCLCPP_INFO(this->get_logger(), "StimulationTracer initialized for session with data_source: %s", this->data_source.c_str());

  response->success = true;
}

void StimulationTracer::handle_finalize_stimulation_tracer(
  const std::shared_ptr<pipeline_interfaces::srv::FinalizeStimulationTracer::Request> request,
  std::shared_ptr<pipeline_interfaces::srv::FinalizeStimulationTracer::Response> response) {

  /* Verify session ID matches. */
  if (request->session_id != this->current_session_id) {
    RCLCPP_WARN(this->get_logger(), "Finalize called with mismatched session_id");
    response->success = false;
    return;
  }

  /* Clear decision traces and reset state. */
  this->decision_traces.clear();
  this->is_initialized = false;
  this->current_session_id = {};
  this->data_source = "";

  RCLCPP_INFO(this->get_logger(), "StimulationTracer finalized");

  response->success = true;
}

void StimulationTracer::handle_decision_trace(const std::shared_ptr<pipeline_interfaces::msg::DecisionTrace> msg) {
  /* Skip if not initialized. */
  if (!this->is_initialized) {
    return;
  }

  /* Only accept traces from current session. */
  if (msg->session_id != this->current_session_id) {
    return;
  }

  /* Store decision trace in memory keyed by decision_id. */
  uint64_t decision_id = msg->decision_id;
  
  this->decision_traces[decision_id].push_back(*msg);

  RCLCPP_DEBUG(this->get_logger(), "Stored decision trace: decision_id=%lu, status=%u", 
               decision_id, msg->status);

  /* If this trace has a terminal status, finalize the decision. */
  if (this->is_terminal_status(msg->status)) {
    finalize_decision(decision_id);
  }
}

void StimulationTracer::handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Check if this sample contains a pulse trigger. */
  if (!msg->pulse_trigger) {
    return;
  }

  /* Capture system time when pulse was received. */
  auto pulse_receive_time = std::chrono::high_resolution_clock::now();
  uint64_t pulse_system_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
    pulse_receive_time.time_since_epoch()).count();

  double_t actual_stimulation_time = msg->time;
  uint64_t actual_stimulation_sample_index = msg->sample_index;

  RCLCPP_INFO(this->get_logger(), "Pulse trigger detected at time: %.4f (s), sample_index: %lu", 
              actual_stimulation_time, actual_stimulation_sample_index);

  /* Find the matching decision trace. */
  auto* matching_trace = find_matching_decision(pulse_system_time);

  if (matching_trace == nullptr) {
    RCLCPP_WARN(this->get_logger(), "No matching decision trace found for pulse at time: %.4f (s)", 
                actual_stimulation_time);
    return;
  }

  /* Create and publish the observation trace. */
  auto observation_trace = pipeline_interfaces::msg::DecisionTrace();
  observation_trace.session_id = matching_trace->session_id;
  observation_trace.decision_id = matching_trace->decision_id;
  observation_trace.status = pipeline_interfaces::msg::DecisionTrace::STATUS_PULSE_OBSERVED;
  observation_trace.actual_stimulation_time = actual_stimulation_time;
  observation_trace.actual_stimulation_sample_index = actual_stimulation_sample_index;

  /* Calculate timing error: actual - scheduled. */
  observation_trace.timing_error = actual_stimulation_time - matching_trace->requested_stimulation_time;

  RCLCPP_INFO(this->get_logger(), 
              "Matched pulse to decision_id=%lu, timing_error=%.4f ms",
              matching_trace->decision_id, observation_trace.timing_error * 1000.0);

  /* Publish observation trace. */
  this->decision_trace_publisher->publish(observation_trace);
}

pipeline_interfaces::msg::DecisionTrace* StimulationTracer::find_matching_decision(uint64_t pulse_system_time) {
  /* Find decision trace with latest system_time_decider_finished that's before pulse_system_time. */
  pipeline_interfaces::msg::DecisionTrace* best_match = nullptr;
  uint64_t latest_decision_time = 0;

  for (auto& [key, traces] : this->decision_traces) {
    for (auto& trace : traces) {
      /* Check if this trace has system_time_decider_finished set and it's before pulse. */
      if (trace.system_time_decider_finished > 0 && 
          trace.system_time_decider_finished < pulse_system_time) {
        
        /* Check if this is the latest decision so far. */
        if (trace.system_time_decider_finished > latest_decision_time) {
          latest_decision_time = trace.system_time_decider_finished;
          best_match = &trace;
        }
      }
    }
  }

  return best_match;
}

void StimulationTracer::finalize_decision(uint64_t decision_id) {
  /* Find all traces for this decision. */
  auto it = this->decision_traces.find(decision_id);
  if (it == this->decision_traces.end() || it->second.empty()) {
    return;
  }

  /* Merge all traces into one final trace. */
  auto final_trace = pipeline_interfaces::msg::DecisionTrace();

  for (const auto& trace : it->second) {
    /* Copy all non-zero/non-empty fields from each trace. */
    
    /* Metadata */
    if (trace.session_id != std::array<uint8_t, 16>{}) {
      final_trace.session_id = trace.session_id;
    }
    if (trace.decision_id != 0) {
      final_trace.decision_id = trace.decision_id;
    }

    /* Status - keep the latest (highest priority) status */
    if (trace.status > final_trace.status) {
      final_trace.status = trace.status;
    }

    /* Decision info */
    if (trace.reference_sample_time != 0.0) final_trace.reference_sample_time = trace.reference_sample_time;
    if (trace.reference_sample_index != 0) final_trace.reference_sample_index = trace.reference_sample_index;
    if (trace.stimulate) final_trace.stimulate = trace.stimulate;
    if (trace.requested_stimulation_time != 0.0) final_trace.requested_stimulation_time = trace.requested_stimulation_time;

    /* Timing */
    if (trace.decider_duration != 0.0) final_trace.decider_duration = trace.decider_duration;
    if (trace.preprocessor_duration != 0.0) final_trace.preprocessor_duration = trace.preprocessor_duration;
    if (trace.system_time_decider_received != 0) final_trace.system_time_decider_received = trace.system_time_decider_received;
    if (trace.system_time_decider_finished != 0) final_trace.system_time_decider_finished = trace.system_time_decider_finished;

    /* TriggerTimer fields */
    if (trace.system_time_trigger_timer_received != 0) final_trace.system_time_trigger_timer_received = trace.system_time_trigger_timer_received;
    if (trace.system_time_trigger_timer_finished != 0) final_trace.system_time_trigger_timer_finished = trace.system_time_trigger_timer_finished;
    if (trace.system_time_hardware_fired != 0) final_trace.system_time_hardware_fired = trace.system_time_hardware_fired;
    if (trace.sample_time_at_firing != 0.0) final_trace.sample_time_at_firing = trace.sample_time_at_firing;
    if (trace.pipeline_latency_at_firing != 0.0) final_trace.pipeline_latency_at_firing = trace.pipeline_latency_at_firing;
    if (trace.latency_corrected_time_at_firing != 0.0) final_trace.latency_corrected_time_at_firing = trace.latency_corrected_time_at_firing;

    /* Observed pulse fields */
    if (trace.actual_stimulation_time != 0.0) final_trace.actual_stimulation_time = trace.actual_stimulation_time;
    if (trace.actual_stimulation_sample_index != 0) final_trace.actual_stimulation_sample_index = trace.actual_stimulation_sample_index;
    if (trace.timing_error != 0.0) final_trace.timing_error = trace.timing_error;
  }

  /* Publish the final merged trace. */
  this->decision_trace_final_publisher->publish(final_trace);

  RCLCPP_INFO(this->get_logger(), "Finalized decision trace: decision_id=%lu, status=%u", 
              final_trace.decision_id, final_trace.status);

  /* Remove from memory. */
  this->decision_traces.erase(it);
}

bool StimulationTracer::is_terminal_status(uint8_t status) {
  /* A decision is finalized when it reaches one of these terminal statuses. */
  if (status == pipeline_interfaces::msg::DecisionTrace::STATUS_REJECTED ||
      status == pipeline_interfaces::msg::DecisionTrace::STATUS_PULSE_OBSERVED ||
      status == pipeline_interfaces::msg::DecisionTrace::STATUS_MISSED ||
      status == pipeline_interfaces::msg::DecisionTrace::STATUS_ERROR) {
    return true;
  }

  /* For playback and simulator data sources, STATUS_FIRED is also sufficient to finalize. */
  if ((this->data_source == "playback" || this->data_source == "simulator") &&
      status == pipeline_interfaces::msg::DecisionTrace::STATUS_FIRED) {
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
