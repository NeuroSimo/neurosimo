//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_TRIGGERTIMER_H
#define EEG_PROCESSOR_TRIGGERTIMER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "labjack_interface.h"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/trigger_loopback_latency.hpp"
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
  rclcpp::Publisher<pipeline_interfaces::msg::TriggerLoopbackLatency>::SharedPtr trigger_loopback_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr heartbeat_timer;

  std::unique_ptr<LabJackInterface> labjack_manager;

  double_t last_latency_measurement_time = 0.0;
  double_t current_latency = 0.0;
  double_t current_time = 0.0;

  double_t triggering_tolerance = 0.0;
  double_t trigger_loopback_latency_threshold = 0.0;
  bool simulate_labjack = false;

  /* Comparator for priority queue - sorts by trigger time. */
  struct TriggerRequestComparator {
    bool operator()(const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>& a,
                    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>& b) const {
      return a->timed_trigger.time > b->timed_trigger.time;  // min-heap by time
    }
  };

  /* Priority queue for trigger requests. */
  std::priority_queue<std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>,
                      std::vector<std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request>>,
                      TriggerRequestComparator> trigger_queue;
  std::mutex queue_mutex;

  void attempt_labjack_connection();
  void reset_state();

  void trigger_pulses_until_time(double_t sample_time);
  void measure_latency(bool latency_trigger, double_t sample_time);

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
