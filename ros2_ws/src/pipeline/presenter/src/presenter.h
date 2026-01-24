#ifndef EEG_PROCESSOR_PRESENTER_H
#define EEG_PROCESSOR_PRESENTER_H

#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "inotify_utils/inotify_watcher.h"

#include "presenter_wrapper.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"
#include "pipeline_interfaces/action/initialize_presenter.hpp"
#include "pipeline_interfaces/srv/finalize_presenter.hpp"

#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

const std::string bold_on = "\033[1m";
const std::string bold_off = "\033[0m";

const std::string UNSET_STRING = "";

struct StimulusCompare {
  bool operator()(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>& a,
                  const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>& b) const
  {
    // return true if a should come *after* b in the queue (i.e. a.time > b.time)
    return a->time > b->time;
  }
};

class PresenterWrapper;

class EegPresenter : public rclcpp::Node {
public:
  EegPresenter();

private:
  void publish_python_logs(double sample_time, bool is_initialization);

  rclcpp_action::GoalResponse handle_initialize_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pipeline_interfaces::action::InitializePresenter::Goal> goal);
  rclcpp_action::CancelResponse handle_initialize_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle);
  void handle_initialize_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle);
  void execute_initialize(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pipeline_interfaces::action::InitializePresenter>> goal_handle);
  void handle_finalize_presenter(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Response> response);

  void handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  void update_time(double_t time);
  void handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg);

  rclcpp::Logger logger;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_subscriber;

  rclcpp::Subscription<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_subscriber;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  /* Action server for initialization */
  rclcpp_action::Server<pipeline_interfaces::action::InitializePresenter>::SharedPtr initialize_action_server;

  /* Service server for finalization */
  rclcpp::Service<pipeline_interfaces::srv::FinalizePresenter>::SharedPtr finalize_service_server;

  /* Initialization state */
  bool is_initialized = false;
  bool is_enabled = false;
  std::string initialized_project_name;
  std::string initialized_module_filename;
  std::filesystem::path initialized_working_directory;

  /* State variables */
  bool error_occurred = false;

  std::priority_queue<
    std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>,
    std::vector<std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>>, StimulusCompare> sensory_stimuli;

  std::unique_ptr<PresenterWrapper> presenter_wrapper;

  /* Track session state internally based on session markers from EEG samples. */
  bool session_started = false;
};

#endif //EEG_PROCESSOR_PRESENTER_H
