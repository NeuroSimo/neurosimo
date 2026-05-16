//
// Created by stimulation_tracer component
//

#ifndef STIMULATION_TRACER_H
#define STIMULATION_TRACER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <map>
#include <vector>
#include <array>

#include "neurosimo_pipeline_interfaces/msg/decision_trace.hpp"
#include "neurosimo_pipeline_interfaces/msg/attempt_trace.hpp"
#include "neurosimo_pipeline_interfaces/srv/initialize_stimulation_tracer.hpp"
#include "neurosimo_pipeline_interfaces/srv/finalize_stimulation_tracer.hpp"

class StimulationTracer : public rclcpp::Node {
public:
  StimulationTracer();
  ~StimulationTracer();

private:
  rclcpp::Logger logger;

  rclcpp::Subscription<neurosimo_pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_subscriber;
  rclcpp::Subscription<neurosimo_pipeline_interfaces::msg::AttemptTrace>::SharedPtr attempt_trace_subscriber;
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>::SharedPtr attempt_trace_publisher;
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>::SharedPtr attempt_trace_final_publisher;

  /* Service servers for initialization and finalization */
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer>::SharedPtr initialize_service_server;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer>::SharedPtr finalize_service_server;

  /* Current session tracking */
  bool is_initialized = false;
  std::array<uint8_t, 16> current_session_id = {};

  /* Storage for attempt traces keyed by attempt_in_session */
  std::map<uint64_t, std::vector<neurosimo_pipeline_interfaces::msg::AttemptTrace>> attempt_traces;

  /* Storage for decision traces, used to match decisions to trials */
  std::vector<neurosimo_pipeline_interfaces::msg::DecisionTrace> decision_traces;

  void handle_decision_trace(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::DecisionTrace> msg);
  void handle_attempt_trace(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::AttemptTrace> msg);

  void handle_initialize_stimulation_tracer(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeStimulationTracer::Response> response);

  void handle_finalize_stimulation_tracer(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeStimulationTracer::Response> response);

  /* Merge all attempt traces for a given key and publish final */
  void finalize_attempt(uint64_t attempt_in_session);

  /* Check if a trial trace is terminal (finalized) */
  bool is_terminal_status(uint8_t status);
};

#endif // STIMULATION_TRACER_H
