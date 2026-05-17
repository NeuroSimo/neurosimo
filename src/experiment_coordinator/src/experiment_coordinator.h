#ifndef EXPERIMENT_COORDINATOR_H
#define EXPERIMENT_COORDINATOR_H

#include <array>
#include <string>
#include <memory>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "neurosimo_eeg_interfaces/msg/sample.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "neurosimo_project_interfaces/msg/filename_list.hpp"
#include "neurosimo_project_interfaces/srv/set_module.hpp"
#include "neurosimo_pipeline_interfaces/msg/experiment_state.hpp"
#include "neurosimo_pipeline_interfaces/msg/attempt_trace.hpp"
#include "neurosimo_pipeline_interfaces/msg/attempt_commit.hpp"
#include "neurosimo_pipeline_interfaces/msg/protocol_info.hpp"
#include "neurosimo_pipeline_interfaces/msg/task_start.hpp"
#include "neurosimo_pipeline_interfaces/msg/task_finished.hpp"
#include "neurosimo_pipeline_interfaces/srv/initialize_protocol.hpp"
#include "neurosimo_pipeline_interfaces/srv/finalize_protocol.hpp"
#include "neurosimo_pipeline_interfaces/srv/get_protocol_info.hpp"
#include "neurosimo_system_interfaces/msg/component_health.hpp"

#include "protocol.h"
#include "protocol_loader.h"

const std::string UNSET_STRING = "<unset>";

class ExperimentCoordinator : public rclcpp::Node {
public:
  ExperimentCoordinator();

private:
  /* ROS2 interfaces */
  // Subscribers
  rclcpp::Subscription<neurosimo_eeg_interfaces::msg::Sample>::SharedPtr raw_eeg_subscriber;
  rclcpp::Subscription<neurosimo_pipeline_interfaces::msg::AttemptTrace>::SharedPtr attempt_trace_final_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Subscription<neurosimo_pipeline_interfaces::msg::TaskFinished>::SharedPtr task_finished_subscriber;
  
  // Publishers
  rclcpp::Publisher<neurosimo_eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_publisher;
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::AttemptCommit>::SharedPtr attempt_commit_publisher;
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::TaskStart>::SharedPtr task_start_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<neurosimo_system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;
  rclcpp::Publisher<neurosimo_pipeline_interfaces::msg::ExperimentState>::SharedPtr experiment_state_publisher;

  // Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_simulator_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_bridge_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr finish_session_client;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::InitializeProtocol>::SharedPtr initialize_protocol_service;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::FinalizeProtocol>::SharedPtr finalize_protocol_service;
  rclcpp::Service<neurosimo_pipeline_interfaces::srv::GetProtocolInfo>::SharedPtr get_protocol_info_service;
    
  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer;
  rclcpp::TimerBase::SharedPtr experiment_state_timer;
  
  /* Protocol and state */
  std::optional<experiment_coordinator::Protocol> protocol;
  experiment_coordinator::ExperimentState state;
  experiment_coordinator::ProtocolLoader protocol_loader;
  bool is_protocol_initialized = false;
  std::array<uint8_t, 16> session_id = {};
    
  /* Logger */
  rclcpp::Logger logger;

  /* Callbacks */
  void handle_raw_sample(const std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample> msg);
  void handle_attempt_trace_final(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::AttemptTrace> msg);
  void handle_task_finished(const std::shared_ptr<neurosimo_pipeline_interfaces::msg::TaskFinished> msg);
  
  void handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_initialize_protocol(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeProtocol::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::InitializeProtocol::Response> response);
  void handle_finalize_protocol(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeProtocol::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::FinalizeProtocol::Response> response);
  void handle_get_protocol_info(
    const std::shared_ptr<neurosimo_pipeline_interfaces::srv::GetProtocolInfo::Request> request,
    std::shared_ptr<neurosimo_pipeline_interfaces::srv::GetProtocolInfo::Response> response);
  void request_finish_session();
  void mark_protocol_complete();
  void publish_experiment_state();
  
  /* Experiment logic */
  void publish_attempt_commit();
  void update_experiment_state(double current_time);
  void advance_to_next_element();
  void start_rest(const experiment_coordinator::Rest& rest, double current_time);
  void end_rest();
  void start_task(const experiment_coordinator::Task& task, double current_time);
  void end_task();
  void start_stage(const experiment_coordinator::Stage& stage, double current_time);
  void reset_experiment_state();
  
  /* Component health */
  void publish_heartbeat();
  void publish_health_status(uint8_t health_level, const std::string& message);
  
  /* Time utilities */
  double get_experiment_time(double sample_time);
};

#endif // EXPERIMENT_COORDINATOR_H

