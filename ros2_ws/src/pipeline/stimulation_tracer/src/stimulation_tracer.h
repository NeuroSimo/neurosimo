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

#include "std_msgs/msg/empty.hpp"
#include "eeg_interfaces/msg/sample.hpp"
#include "pipeline_interfaces/msg/decision_trace.hpp"
#include "pipeline_interfaces/srv/initialize_stimulation_tracer.hpp"
#include "pipeline_interfaces/srv/finalize_stimulation_tracer.hpp"
#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

class StimulationTracer : public rclcpp::Node {
public:
  StimulationTracer();
  ~StimulationTracer();

private:
  rclcpp::Logger logger;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_sample_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_subscriber;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_final_publisher;

  /* Service servers for initialization and finalization */
  rclcpp::Service<pipeline_interfaces::srv::InitializeStimulationTracer>::SharedPtr initialize_service_server;
  rclcpp::Service<pipeline_interfaces::srv::FinalizeStimulationTracer>::SharedPtr finalize_service_server;

  /* Current session tracking */
  bool is_initialized = false;
  std::array<uint8_t, 16> current_session_id = {};
  std::string data_source = "";

  /* Storage for decision traces keyed by decision_id */
  std::map<uint64_t, std::vector<pipeline_interfaces::msg::DecisionTrace>> decision_traces;

  void handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_decision_trace(const std::shared_ptr<pipeline_interfaces::msg::DecisionTrace> msg);

  void handle_initialize_stimulation_tracer(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeStimulationTracer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeStimulationTracer::Response> response);

  void handle_finalize_stimulation_tracer(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizeStimulationTracer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizeStimulationTracer::Response> response);

  /* Find the matching decision trace for a pulse trigger */
  pipeline_interfaces::msg::DecisionTrace* find_matching_decision(uint64_t pulse_system_time);

  /* Merge all decision traces for a given key and publish final */
  void finalize_decision(uint64_t decision_id);

  /* Check if a decision trace is terminal (finalized) */
  bool is_terminal_status(uint8_t status);

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;
};

#endif // STIMULATION_TRACER_H