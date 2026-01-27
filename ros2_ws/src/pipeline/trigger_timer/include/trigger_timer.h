//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_TRIGGERTIMER_H
#define EEG_PROCESSOR_TRIGGERTIMER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "labjack_manager.h"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/timing_latency.hpp"
#include "pipeline_interfaces/msg/trigger_info.hpp"
#include "pipeline_interfaces/msg/timing_error.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "pipeline_interfaces/msg/timed_trigger.hpp"
#include "pipeline_interfaces/msg/latency_measurement_trigger.hpp"
#include "pipeline_interfaces/srv/request_timed_trigger.hpp"


class TriggerTimer : public rclcpp::Node {
public:
  TriggerTimer();
  ~TriggerTimer();

private:
  rclcpp::Logger logger;
  rclcpp::Service<pipeline_interfaces::srv::RequestTimedTrigger>::SharedPtr trigger_request_service;
  rclcpp::Publisher<pipeline_interfaces::msg::TimingLatency>::SharedPtr timing_latency_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::TriggerInfo>::SharedPtr trigger_info_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pulse_event_publisher;
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::LatencyMeasurementTrigger>::SharedPtr latency_measurement_trigger_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::TimingError>::SharedPtr timing_error_subscriber;
  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<LabJackManager> labjack_manager;

  double_t latest_timing_error = 0.0;
  double_t last_latency_measurement_time = 0.0;
  double_t current_latency = 0.0;
  double_t current_time = 0.0;

  double_t triggering_tolerance = 0.0;
  bool simulate_labjack = false;

  /* Priority queue for trigger times. */
  std::priority_queue<double_t, std::vector<double_t>, std::greater<double_t>> trigger_queue;
  std::mutex queue_mutex;

  void attempt_labjack_connection();

  void trigger_pulses_until_time(double_t until_time);
  void trigger_latency_measurement(double_t sample_time);

  void handle_eeg_raw(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response);
  void handle_latency_measurement_trigger(const std::shared_ptr<pipeline_interfaces::msg::LatencyMeasurementTrigger> msg);
  void handle_timing_error(const std::shared_ptr<pipeline_interfaces::msg::TimingError> msg);

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;
};

#endif //EEG_PROCESSOR_TRIGGERTIMER_H
