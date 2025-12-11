#ifndef EXPERIMENT_COORDINATOR_H
#define EXPERIMENT_COORDINATOR_H

#include <string>
#include <memory>
#include <vector>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "inotify_utils/inotify_watcher.h"
#include "module_utils/module_manager.h"
#include "eeg_interfaces/msg/sample.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"
#include "project_interfaces/msg/module_list.hpp"
#include "project_interfaces/srv/set_module.hpp"
#include "pipeline_interfaces/msg/experiment_state.hpp"

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
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr pulse_event_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  
  // Publishers
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_publisher;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;
  rclcpp::Publisher<pipeline_interfaces::msg::ExperimentState>::SharedPtr experiment_state_publisher;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_simulator_client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_bridge_client;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service;
  
  /* Module manager for handling protocol selection and project changes */
  std::unique_ptr<module_utils::ModuleManager> module_manager;
  
  // Timers
  rclcpp::TimerBase::SharedPtr healthcheck_timer;
  
  /* Protocol and state */
  std::optional<experiment_coordinator::Protocol> protocol;
  experiment_coordinator::ExperimentState state;
  experiment_coordinator::ProtocolLoader protocol_loader;
  
  /* Protocol name tracking (separate from module_manager since protocols need loading) */
  std::string protocol_name = UNSET_STRING;
  
  /* Logger */
  rclcpp::Logger logger;
  
  /* Healthcheck state */
  enum class CoordinatorState {
    WAITING_FOR_PROTOCOL,
    READY,
    PROTOCOL_ERROR
  };
  CoordinatorState coordinator_state = CoordinatorState::WAITING_FOR_PROTOCOL;
  
  /* Track whether we're in simulation mode (from session metadata) */
  bool is_simulation = false;
  
  /* Callbacks */
  void handle_raw_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_pulse_event(const std::shared_ptr<std_msgs::msg::Empty> msg);
  
  void handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void request_stop_session();
  void mark_protocol_complete();
  void publish_experiment_state(double current_time);
    
  /* Protocol management (called when module_manager selects a protocol) */
  bool load_protocol(const std::string& protocol_name);
  
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

