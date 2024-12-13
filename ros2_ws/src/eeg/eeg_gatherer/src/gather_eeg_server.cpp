#include "rclcpp/rclcpp.hpp"

#include "headers/gather_eeg_server.h"
#include "headers/eeg_gatherer.h"

#include "headers/scheduling_utils.h"
#include "headers/memory_utils.h"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std::placeholders;

/* Use relatively long queue to prevent dropping samples. */
const uint16_t EEG_DATA_SUBSCRIBER_QUEUE_SIZE = 1024;

GatherEegServer::GatherEegServer() : Node("eeg_gatherer") {
  this->gather_eeg_action_server = rclcpp_action::create_server<eeg_interfaces::action::GatherEeg>(
    this,
    "/eeg/gather",
    std::bind(&GatherEegServer::handle_goal, this, _1, _2),
    std::bind(&GatherEegServer::handle_cancel, this, _1),
    std::bind(&GatherEegServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse GatherEegServer::handle_goal([[maybe_unused]] const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GatherEeg::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with start time = %.2f and end time = %.2f.", goal->time_window.start, goal->time_window.end);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GatherEegServer::handle_cancel([[maybe_unused]] const std::shared_ptr<GoalHandleGatherEeg> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GatherEegServer::handle_accepted(const std::shared_ptr<GoalHandleGatherEeg> goal_handle) {
  std::thread{std::bind(&GatherEegServer::execute, this, _1), goal_handle}.detach();
}

void GatherEegServer::execute(const std::shared_ptr<GoalHandleGatherEeg> goal_handle) {
  const auto goal = goal_handle->get_goal();

  auto start_time = goal->time_window.start;
  auto end_time = goal->time_window.end;

  /* TODO: Replace with proper UUID-based id. */
  std::string goal_id = "abcd";

  auto eeg_gatherer = EegGatherer(goal_id, start_time, end_time);

  auto eeg_subscription = this->create_subscription<eeg_interfaces::msg::Sample>(
    "/eeg/raw",
    EEG_DATA_SUBSCRIBER_QUEUE_SIZE,
    [this, &eeg_gatherer](const eeg_interfaces::msg::Sample::SharedPtr msg) {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("eeg_gatherer"),
                          *this->get_clock(),
                          1000,
                          "Received raw EEG datapoint with timestamp %.4f.",
                          msg->time);
      eeg_gatherer.handle_eeg_sample(msg);
    });

  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto result = std::make_shared<eeg_interfaces::action::GatherEeg::Result>();

  while (!(eeg_gatherer.is_finished() || goal_handle->is_canceling())) {
    rclcpp::sleep_for(std::chrono::milliseconds(5));
  }

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    return;
  }

  result->error = *eeg_gatherer.get_error();

  if (rclcpp::ok()) {
    if (eeg_gatherer.success()) {
      auto& eeg_buffer = eeg_gatherer.get_eeg_buffer();
      result->eeg_buffer = eeg_buffer;

      RCLCPP_INFO(this->get_logger(), "%s: # of samples received: %lu", goal_id.c_str(), eeg_buffer.size());
    } else {
      RCLCPP_INFO(this->get_logger(), "%s: Failed to gather data.", goal_id.c_str());
      RCLCPP_INFO(this->get_logger(), "%s: Returning error code: %d", goal_id.c_str(), result->error.value);
    }
    goal_handle->succeed(result);
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal failed.");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GatherEegServer>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("eeg_gatherer"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
