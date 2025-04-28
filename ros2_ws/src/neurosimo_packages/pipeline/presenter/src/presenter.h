#ifndef EEG_PROCESSOR_PRESENTER_H
#define EEG_PROCESSOR_PRESENTER_H

#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"

#include "project_interfaces/msg/presenter_list.hpp"
#include "project_interfaces/srv/set_presenter_module.hpp"
#include "project_interfaces/srv/set_presenter_enabled.hpp"

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
  ~EegPresenter();

private:
  void reset_sensory_stimuli();
  void initialize_presenter_module();
  void unset_presenter_module();

  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);

  void set_presenter_module(const std::string module);
  void handle_set_presenter_module(
      const std::shared_ptr<project_interfaces::srv::SetPresenterModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPresenterModule::Response> response);

  void handle_set_presenter_enabled(
      const std::shared_ptr<project_interfaces::srv::SetPresenterEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPresenterEnabled::Response> response);

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  void update_presenter_list();

  void update_time(double_t time);
  void handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg);

  /* File-system related functions */
  bool change_working_directory(const std::string path);
  std::vector<std::string> list_python_modules_in_working_directory();

  void update_inotify_watch();
  void inotify_timer_callback();

  rclcpp::Logger logger;

  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;

  rclcpp::Subscription<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_subscriber;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Publisher<project_interfaces::msg::PresenterList>::SharedPtr presenter_list_publisher;

  rclcpp::Service<project_interfaces::srv::SetPresenterModule>::SharedPtr set_presenter_module_service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr presenter_module_publisher;

  rclcpp::Service<project_interfaces::srv::SetPresenterEnabled>::SharedPtr set_presenter_enabled_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr presenter_enabled_publisher;

  bool enabled;

  std::string active_project;

  std::string working_directory  = UNSET_STRING;
  bool is_working_directory_set = false;
  std::string module_name = UNSET_STRING;

  std::vector<std::string> modules;

  std::priority_queue<
    std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>,
    std::vector<std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus>>, StimulusCompare> sensory_stimuli;

  std::unique_ptr<PresenterWrapper> presenter_wrapper;

  /* Keep track of the session state so that the Python module can be re-initialized just once
     when the session is stopped. */
  system_interfaces::msg::SessionState session_state;

  /* Inotify variables */
  rclcpp::TimerBase::SharedPtr inotify_timer;
  int inotify_descriptor;
  int watch_descriptor;
  char inotify_buffer[1024];
};

#endif //EEG_PROCESSOR_PRESENTER_H
