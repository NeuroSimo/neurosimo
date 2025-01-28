//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_TRIGGERTIMER_H
#define EEG_PROCESSOR_TRIGGERTIMER_H

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "eeg_msgs/msg/sample.hpp"

#include "pipeline_interfaces/msg/timing_latency.hpp"
#include "pipeline_interfaces/msg/decision_info.hpp"
#include "pipeline_interfaces/msg/trigger_info.hpp"
#include "pipeline_interfaces/msg/timing_error.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"
#include "system_interfaces/msg/session.hpp"

#include "pipeline_interfaces/msg/timed_trigger.hpp"
#include "pipeline_interfaces/srv/request_timed_trigger.hpp"


class TriggerTimer : public rclcpp::Node {
public:
  TriggerTimer();
  ~TriggerTimer();

private:
  rclcpp::Logger logger;
  rclcpp::Subscription<system_interfaces::msg::Healthcheck>::SharedPtr mtms_device_healthcheck_subscriber;
  rclcpp::Service<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr trigger_request_service;
  rclcpp::Publisher<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::DecisionInfo>::SharedPtr decision_info_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::TriggerInfo>::SharedPtr trigger_info_publisher;
  rclcpp::Subscription<eeg_msgs::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::TimedTrigger>::SharedPtr latency_measurement_trigger_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::TimingError>::SharedPtr timing_error_subscriber;
  rclcpp::TimerBase::SharedPtr timer;

  int labjack_handle = -1;
  bool mtms_device_available = false;

  double_t latest_timing_error = 0.0;
  double_t latest_latency_measurement_time = 0.0;
  double_t current_latency = 0.0;
  double_t current_latency_corrected_time = 0.0;

  double_t triggering_tolerance = 0.0;

  /* Priority queue for trigger times. */
  std::priority_queue<double_t, std::vector<double_t>, std::greater<double_t>> trigger_queue;
  std::mutex queue_mutex;

  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);
  void attempt_labjack_connection();
  void handle_mtms_device_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg);
  void handle_eeg_raw(const std::shared_ptr<eeg_msgs::msg::Sample> msg);
  void handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response);
  void handle_latency_measurement_trigger(const std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> msg);
  void handle_timing_error(const std::shared_ptr<pipeline_interfaces::msg::TimingError> msg);
  void trigger_labjack(const char* name);
  bool safe_error_check(int err, const char* action);

  /* Session management */
  bool session_received = false;
  system_interfaces::msg::SessionState session_state;

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;
};

#endif //EEG_PROCESSOR_TRIGGERTIMER_H
