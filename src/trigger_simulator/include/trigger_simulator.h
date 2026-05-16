#ifndef TRIGGER_SIMULATOR_H
#define TRIGGER_SIMULATOR_H

#include "rclcpp/rclcpp.hpp"
#include <limits>
#include <mutex>
#include <optional>

#include "neurosimo_pipeline_interfaces/msg/attempt_trace.hpp"
#include "neurosimo_pipeline_interfaces/srv/request_timed_trigger.hpp"
#include "neurosimo_pipeline_interfaces/srv/initialize_trigger_simulator.hpp"
#include "neurosimo_pipeline_interfaces/srv/finalize_trigger_simulator.hpp"

#include "neurosimo_eeg_interfaces/srv/inject_trigger.hpp"

#include "neurosimo_system_interfaces/msg/component_health.hpp"
#include "std_msgs/msg/empty.hpp"


class TriggerSimulator : public rclcpp::Node {
public:
  TriggerSimulator();

private:
  rclcpp::Logger logger;

  /* Services advertised */
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr trigger_request_service;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator>::SharedPtr initialize_service;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator>::SharedPtr finalize_service;

  /* Publishers */
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::AttemptTrace>::SharedPtr attempt_trace_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<neurosimo_system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;

  /* Service client for eeg_simulator trigger injection */
  rclcpp::CallbackGroup::SharedPtr client_callback_group;
  rclcpp::Client<neurosimo_eeg_interfaces::srv::InjectTrigger>::SharedPtr inject_trigger_client;

  /* Timers */
  rclcpp::TimerBase::SharedPtr heartbeat_timer;

  /* State */
  bool is_initialized = false;
  std::vector<uint8_t> session_id;
  double_t minimum_trial_interval = 0.0;
  double_t last_trigger_time = std::numeric_limits<double_t>::quiet_NaN();

  /* Handlers */
  void handle_request_timed_trigger(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Response> response);

  void handle_initialize(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeTriggerSimulator::Response> response);

  void handle_finalize(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeTriggerSimulator::Response> response);

  struct InjectTriggerCallResult {
    enum class Status { Success, Timeout } status;
    std::shared_ptr<neurosimo_eeg_interfaces::srv::InjectTrigger::Response> response;
  };

  /* Helpers */
  void publish_heartbeat();
  void publish_health_status(uint8_t health_level, const std::string& message);
  void reset_state();

  void publish_attempt_trace(const neurosimo_pipeline_interfaces::msg::AttemptTrace& trace);
  void publish_attempt_trace(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request>& request,
    uint8_t status,
    uint64_t system_time_trigger_timer_received_ns);

  InjectTriggerCallResult call_service_inject_trigger(
    double_t sample_time,
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::RequestTimedTrigger::Request>& request);
};

#endif // TRIGGER_SIMULATOR_H
