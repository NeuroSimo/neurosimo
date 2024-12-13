#ifndef GATHER_EEG_SERVER_GATHER_EEG_SERVER_H
#define GATHER_EEG_SERVER_GATHER_EEG_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/gather_eeg_error.hpp"
#include "eeg_interfaces/action/gather_eeg.hpp"

using GatherEeg = eeg_interfaces::action::GatherEeg;
using GoalHandleGatherEeg = rclcpp_action::ServerGoalHandle<eeg_interfaces::action::GatherEeg>;

class GatherEegServer : public rclcpp::Node {
public:
  GatherEegServer();

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GatherEeg::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGatherEeg> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleGatherEeg> goal_handle);
  void execute(const std::shared_ptr<GoalHandleGatherEeg> goal_handle);

  std::vector<eeg_interfaces::msg::Sample> eeg_buffer;

  rclcpp_action::Server<eeg_interfaces::action::GatherEeg>::SharedPtr gather_eeg_action_server;
};

#endif //GATHER_EEG_SERVER_GATHER_EEG_SERVER_H
