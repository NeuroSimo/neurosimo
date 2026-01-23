#ifndef EEG_PROCESSOR_PRESENTER_H
#define EEG_PROCESSOR_PRESENTER_H

#include <cmath>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "inotify_utils/inotify_watcher.h"
#include "module_utils/module_manager.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"

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
  bool initialize_presenter_module();
  void publish_python_logs(double sample_time, bool is_initialization);
  void unset_presenter_module();

  void handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  void update_time(double_t time);
  void handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg);

  rclcpp::Logger logger;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_subscriber;

  rclcpp::Subscription<pipeline_interfaces::msg::SensoryStimulus>::SharedPtr sensory_stimulus_subscriber;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  /* Module manager for handling module selection and project changes */
  std::unique_ptr<module_utils::ModuleManager> module_manager;

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
