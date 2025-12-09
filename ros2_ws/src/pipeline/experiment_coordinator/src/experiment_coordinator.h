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
#include "system_interfaces/srv/stop_session.hpp"
#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"
#include "project_interfaces/msg/protocol_list.hpp"
#include "project_interfaces/srv/set_protocol.hpp"
#include "pipeline_interfaces/msg/experiment_state.hpp"

#include "protocol.h"
#include "protocol_loader.h"

const std::string UNSET_STRING = "<unset>";

class ExperimentCoordinator : public rclcpp::Node {
public:
  ExperimentCoordinator();
  ~ExperimentCoordinator() = default;

private:
  /* ROS2 interfaces */
  // Subscribers
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr raw_eeg_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pulse_event_subscriber;
  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  
  // Publishers
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_publisher;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::ExperimentState>::SharedPtr experiment_state_publisher;
  rclcpp::Publisher<project_interfaces::msg::ProtocolList>::SharedPtr protocol_list_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr protocol_module_publisher;
  rclcpp::Client<system_interfaces::srv::StopSession>::SharedPtr stop_session_client;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service;
  rclcpp::Service<project_interfaces::srv::SetProtocol>::SharedPtr set_protocol_service;
  
  // Timers
  rclcpp::TimerBase::SharedPtr healthcheck_timer;
  rclcpp::TimerBase::SharedPtr inotify_timer;
  
  /* Protocol and state */
  std::optional<experiment_coordinator::Protocol> protocol;
  experiment_coordinator::ExperimentState state;
  experiment_coordinator::ProtocolLoader protocol_loader;
  
  /* Project management */
  std::string active_project = UNSET_STRING;
  std::string working_directory = UNSET_STRING;
  std::string protocol_name = UNSET_STRING;
  std::vector<std::string> available_protocols;
  bool is_working_directory_set = false;
  
  /* Session state */
  system_interfaces::msg::SessionState session_state;
  
  /* Inotify for file watching */
  int inotify_descriptor;
  int watch_descriptor;
  char inotify_buffer[1024];
  
  /* Logger */
  rclcpp::Logger logger;
  
  /* Healthcheck state */
  enum class CoordinatorState {
    WAITING_FOR_PROTOCOL,
    READY,
    PROTOCOL_ERROR
  };
  CoordinatorState coordinator_state = CoordinatorState::WAITING_FOR_PROTOCOL;
  
  /* Callbacks */
  void handle_raw_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_pulse_event(const std::shared_ptr<std_msgs::msg::Empty> msg);
  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);
  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  
  void handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void request_stop_session();
  void mark_protocol_complete();
  void publish_experiment_state(double current_time);
    
  void handle_set_protocol(
    const std::shared_ptr<project_interfaces::srv::SetProtocol::Request> request,
    std::shared_ptr<project_interfaces::srv::SetProtocol::Response> response);
  
  /* Protocol management */
  bool load_protocol(const std::string& protocol_name);
  void unset_protocol();
  bool set_protocol(const std::string& protocol_name);
  std::string get_protocol_name_with_fallback(const std::string protocol_name);
  
  /* Project management */
  bool change_working_directory(const std::string& path);
  std::vector<std::string> list_yaml_files_in_working_directory();
  void update_protocol_list();
  
  /* Inotify */
  void update_inotify_watch();
  void inotify_timer_callback();
  
  /* Experiment logic */
  void update_experiment_state(double current_time);
  void advance_to_next_element();
  void start_rest(const experiment_coordinator::Rest& rest, double current_time);
  void end_rest();
  void start_stage(const experiment_coordinator::Stage& stage, double current_time);
  void reset_experiment_state();
  
  /* Healthcheck */
  void publish_healthcheck();
  
  /* Time utilities */
  double get_experiment_time(double sample_time);
};

#endif // EXPERIMENT_COORDINATOR_H

