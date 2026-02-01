#ifndef EEG_PROCESSOR_PRESENTER_H
#define EEG_PROCESSOR_PRESENTER_H

#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"

#include "presenter_wrapper.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"
#include "pipeline_interfaces/srv/initialize_presenter.hpp"
#include "pipeline_interfaces/srv/finalize_presenter.hpp"

#include "system_interfaces/msg/component_health.hpp"
#include "system_interfaces/srv/abort_session.hpp"

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
  void publish_python_logs(uint8_t phase, double sample_time);
  void _publish_heartbeat();
  void _publish_health_status(uint8_t health_level, const std::string& message);

  void abort_session();

  void handle_initialize_presenter(
    const std::shared_ptr<pipeline_interfaces::srv::InitializePresenter::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializePresenter::Response> response);
  void handle_finalize_presenter(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizePresenter::Response> response);

  void handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg);

  rclcpp::Logger logger;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_subscriber;

  rclcpp::Subscription<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_subscriber;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;
  rclcpp::Publisher<system_interfaces::msg::ComponentHealth>::SharedPtr health_publisher;

  rclcpp::TimerBase::SharedPtr heartbeat_timer;

  /* Service server for initialization */
  rclcpp::Service<pipeline_interfaces::srv::InitializePresenter>::SharedPtr initialize_service_server;

  /* Service server for finalization */
  rclcpp::Service<pipeline_interfaces::srv::FinalizePresenter>::SharedPtr finalize_service_server;

  /* Service client for session abort */
  rclcpp::Client<system_interfaces::srv::AbortSession>::SharedPtr abort_session_client;

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
};

#endif //EEG_PROCESSOR_PRESENTER_H
