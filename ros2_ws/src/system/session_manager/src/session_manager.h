#ifndef SESSION_MANAGER_H
#define SESSION_MANAGER_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"

#include "system_interfaces/srv/start_session.hpp"
#include "system_interfaces/srv/stop_session.hpp"


class SessionManager : public rclcpp::Node {
public:
  SessionManager();

private:
  void handle_start_session(
    const std::shared_ptr<system_interfaces::srv::StartSession::Request> request,
    std::shared_ptr<system_interfaces::srv::StartSession::Response> response);

  void handle_stop_session(
    const std::shared_ptr<system_interfaces::srv::StopSession::Request> request,
    std::shared_ptr<system_interfaces::srv::StopSession::Response> response);

  void publish_session();

  rclcpp::Publisher<system_interfaces::msg::Session>::SharedPtr session_publisher;

  rclcpp::Service<system_interfaces::srv::StartSession>::SharedPtr start_session_service;
  rclcpp::Service<system_interfaces::srv::StopSession>::SharedPtr stop_session_service;

  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::TimerBase::SharedPtr timer;

  /* Internal session variables */
  uint8_t session_state = system_interfaces::msg::SessionState::STOPPED;
  std::chrono::steady_clock::time_point session_start_time;
};

#endif //SESSION_MANAGER_H
