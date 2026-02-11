//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_TRIGGERTIMER_H
#define EEG_PROCESSOR_TRIGGERTIMER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "labjack_interface.h"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/loopback_latency.hpp"
#include "pipeline_interfaces/msg/decision_trace.hpp"

#include "pipeline_interfaces/msg/timed_trigger.hpp"
#include "pipeline_interfaces/srv/request_timed_trigger.hpp"
#include "pipeline_interfaces/srv/initialize_trigger_timer.hpp"
#include "pipeline_interfaces/srv/finalize_trigger_timer.hpp"

#include "system_interfaces/msg/component_health.hpp"
#include "std_msgs/msg/empty.hpp"


class TriggerTimer : public rclcpp::Node {
public:
  TriggerTimer();
  ~TriggerTimer();

private:
  rclcpp::Logger logger;
  rclcpp::Service<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr trigger_request_service;
  rclcpp::Service<pipeline_interfaces::srv::InitializeTriggerTimer>::SharedPtr initialize_service;
  rclcpp::Service<pipeline_interfaces::srv::FinalizeTriggerTimer>::SharedPtr finalize_service;
  rclcpp::Publisher<pipeline_interfaces::msg::LoopbackLatency>::SharedPtr loopback_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  rclcpp::TimerBase::SharedPtr active_trigger_timer;

  std::unique_ptr<LabJackInterface> labjack_manager;

  double_t last_loopback_time = 0.0;
  double_t current_loopback_latency = 0.0;
  double_t current_time = 0.0;

  // Latest sample information for calculating stimulation horizon
  double_t latest_sample_time = 0.0;

  // Synchronized time tracking for sample time and system time
  double_t stored_sample_time = 0.0;
  std::chrono::high_resolution_clock::time_point stored_system_time;

  // Configuration parameters
  double_t maximum_timing_offset = 0.0;
  double_t maximum_loopback_latency = 0.0;
  double_t trigger_to_pulse_delay = 0.0;
  bool simulate_labjack = false;

  enum class SchedulingResult {
    SCHEDULED,
    TOO_LATE,
    LOOPBACK_LATENCY_EXCEEDED,
    ERROR
  };

  /* Comparator for priority queue - sorts by trigger time. */
  struct TriggerRequestComparator {
    bool operator()(const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>& a,
                    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>& b) const {
      return a->timed_trigger.time > b->timed_trigger.time;  // min-heap by time
    }
  };

  std::mutex handler_mutex;  // Mutex for trigger request and EEG raw handlers

  void attempt_labjack_connection();
  void reset_state();

  void measure_loopback_latency(bool loopback_trigger, double_t sample_time);
  SchedulingResult schedule_trigger_with_timer(std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request);
  double_t estimate_current_sample_time();

  void _publish_heartbeat();
  void _publish_health_status(uint8_t health_level, const std::string& message);

  void handle_eeg_raw(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response);
  void handle_initialize_trigger_timer(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeTriggerTimer::Response> response);
  void handle_finalize_trigger_timer(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizeTriggerTimer::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizeTriggerTimer::Response> response);
};

#endif //EEG_PROCESSOR_TRIGGERTIMER_H
