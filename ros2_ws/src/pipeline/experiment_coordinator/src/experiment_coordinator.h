#ifndef EXPERIMENT_COORDINATOR_H
#define EXPERIMENT_COORDINATOR_H

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "eeg_interfaces/msg/sample.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"
#include "pipeline_interfaces/msg/experiment_state.hpp"
#include "pipeline_interfaces/msg/decision_trace.hpp"
#include "pipeline_interfaces/srv/initialize_protocol.hpp"

#include "protocol.h"
#include "protocol_loader.h"

const std::string UNSET_STRING = "<unset>";

class ExperimentCoordinator : public rclcpp::Node {
public:
  ExperimentCoordinator();

private:
  /* ROS2 interfaces */
  // Subscribers
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr raw_eeg_subscriber;
  rclcpp::Subscription<pipeline_interfaces::msg::DecisionTrace>::SharedPtr decision_trace_final_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  
  // Publishers
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::ExperimentState>::SharedPtr experiment_state_publisher;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_simulator_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_bridge_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_session_client;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service;
  rclcpp::Service<pipeline_interfaces::srv::InitializeProtocol>::SharedPtr initialize_protocol_service;
    
  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  
  /* Protocol and state */
  std::optional<experiment_coordinator::Protocol> protocol;
  experiment_coordinator::ExperimentState state;
  experiment_coordinator::ProtocolLoader protocol_loader;
  bool is_protocol_initialized = false;
  bool error_occurred = false;
    
  /* Logger */
  rclcpp::Logger logger;

  /* Callbacks */
  void handle_raw_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_decision_trace_final(const std::shared_ptr<pipeline_interfaces::msg::DecisionTrace> msg);
  
  void handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_initialize_protocol(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeProtocol::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeProtocol::Response> response);
  void request_finish_session();
  void mark_protocol_complete();
  void publish_experiment_state(double current_time);
  
  /* Experiment logic */
  void update_experiment_state(double current_time);
  void advance_to_next_element();
  void start_rest(const experiment_coordinator::Rest& rest, double current_time);
  void end_rest();
  void start_stage(const experiment_coordinator::Stage& stage, double current_time);
  void reset_experiment_state();
  
  /* Healthcheck */
  void publish_heartbeat();
  
  /* Time utilities */
  double get_experiment_time(double sample_time);
};

#endif // EXPERIMENT_COORDINATOR_H

