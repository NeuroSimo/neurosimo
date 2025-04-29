#include <chrono>
#include <filesystem>

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "presenter_wrapper.h"
#include "presenter.h"

#include "realtime_utils/utils.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono;
using namespace std::placeholders;

const std::string SENSORY_STIMULUS_TOPIC = "/pipeline/sensory_stimulus";

const std::string PROJECTS_DIRECTORY = "/app/projects";

const std::string DEFAULT_PRESENTER_NAME = "example";

/* XXX: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 5ms;


EegPresenter::EegPresenter() : Node("presenter"), logger(rclcpp::get_logger("presenter")) {
  /* Subscriber for session. */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.deadline_callback = [this]([[maybe_unused]] rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Session not received within deadline.");
  };

  this->session_subscriber = this->create_subscription<system_interfaces::msg::Session>(
    "/system/session",
    qos_session,
    std::bind(&EegPresenter::handle_session, this, _1),
    subscription_options);

  /* Subscriber for sensory stimuli. */

  // Messages can be sent in bursts so keep all messages in the queue.
  auto qos_keep_all = rclcpp::QoS(rclcpp::KeepAll())
  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  this->sensory_stimulus_subscriber = create_subscription<pipeline_interfaces::msg::SensoryStimulus>(
    SENSORY_STIMULUS_TOPIC,
    qos_keep_all,
    std::bind(&EegPresenter::handle_sensory_stimulus, this, _1));

  RCLCPP_INFO(this->get_logger(), "Listening to sensory stimuli on topic %s.", SENSORY_STIMULUS_TOPIC.c_str());

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegPresenter::handle_set_active_project, this, _1));

  /* Publisher for listing presenters. */
  this->presenter_list_publisher = this->create_publisher<project_interfaces::msg::PresenterList>(
    "/pipeline/presenter/list",
    qos_persist_latest);

  /* Service for changing presenter module. */
  this->set_presenter_module_service = this->create_service<project_interfaces::srv::SetPresenterModule>(
    "/pipeline/presenter/module/set",
    std::bind(&EegPresenter::handle_set_presenter_module, this, _1, _2));

  /* Publisher for presenter module. */
  this->presenter_module_publisher = this->create_publisher<std_msgs::msg::String>(
    "/pipeline/presenter/module",
    qos_persist_latest);

  /* Service for enabling and disabling presenter. */
  this->set_presenter_enabled_service = this->create_service<project_interfaces::srv::SetPresenterEnabled>(
    "/pipeline/presenter/enabled/set",
    std::bind(&EegPresenter::handle_set_presenter_enabled, this, _1, _2));

  /* Publisher for presenter enabled message. */
  this->presenter_enabled_publisher = this->create_publisher<std_msgs::msg::Bool>(
    "/pipeline/presenter/enabled",
    qos_persist_latest);

  /* Initialize variables. */
  this->presenter_wrapper = std::make_unique<PresenterWrapper>(logger);

  /* Initialize inotify. */
  this->inotify_descriptor = inotify_init();
  if (this->inotify_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error initializing inotify");
      exit(1);
  }

  /* Set the inotify descriptor to non-blocking. */
  int flags = fcntl(inotify_descriptor, F_GETFL, 0);
  fcntl(inotify_descriptor, F_SETFL, flags | O_NONBLOCK);

  /* Create a timer callback to poll inotify. */
  this->inotify_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&EegPresenter::inotify_timer_callback, this));
}

EegPresenter::~EegPresenter() {
  inotify_rm_watch(inotify_descriptor, watch_descriptor);
  close(inotify_descriptor);
}

void EegPresenter::reset_sensory_stimuli() {
  while (!this->sensory_stimuli.empty()) {
    this->sensory_stimuli.pop();
  }
}

/* Functions to re-initialize the presenter state. */
void EegPresenter::initialize_presenter_module() {
  reset_sensory_stimuli();

  if (!this->enabled ||
      this->working_directory == UNSET_STRING ||
      this->module_name == UNSET_STRING) {

    return;
  }
  this->presenter_wrapper->initialize_module(
    this->working_directory,
    this->module_name);
}

/* Session handler. */
void EegPresenter::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  bool state_changed = this->session_state.value != msg->state.value;
  this->session_state = msg->state;

  if (state_changed && this->session_state.value == system_interfaces::msg::SessionState::STOPPED) {
    this->initialize_presenter_module();
  }

  update_time(msg->time);
}

/* Listing and setting EEG presenters. */

void EegPresenter::handle_set_presenter_enabled(
      const std::shared_ptr<project_interfaces::srv::SetPresenterEnabled::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPresenterEnabled::Response> response) {

  /* Update local state variable. */
  this->enabled = request->enabled;

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = enabled;

  this->presenter_enabled_publisher->publish(msg);

  RCLCPP_INFO(this->get_logger(), "%s presenter.", this->enabled ? "Enabling" : "Disabling");

  /* Initialize the presenter module if it was enabled, otherwise reset it. */
  if (this->enabled) {
    this->initialize_presenter_module();

  } else {
    /* Reset the state of the existing module so that any windows etc. created by the Python module are closed,
       but do not unset the module. */
    this->presenter_wrapper->reset_module_state();
  }

  response->success = true;
}

void EegPresenter::unset_presenter_module() {
  this->module_name = UNSET_STRING;

  RCLCPP_INFO(this->get_logger(), "Presenter module unset.");

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->presenter_module_publisher->publish(msg);

  /* Reset the state of the existing module. */
  this->presenter_wrapper->reset_module_state();
}

void EegPresenter::set_presenter_module(const std::string module) {
  this->module_name = module;

  RCLCPP_INFO(this->get_logger(), "Presenter set to: %s.", this->module_name.c_str());

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = this->module_name;

  this->presenter_module_publisher->publish(msg);

  /* Initialize the wrapper to use the changed presenter module. */
  initialize_presenter_module();
}

void EegPresenter::handle_set_presenter_module(
      const std::shared_ptr<project_interfaces::srv::SetPresenterModule::Request> request,
      std::shared_ptr<project_interfaces::srv::SetPresenterModule::Response> response) {

  set_presenter_module(request->module);
  response->success = true;
}

void EegPresenter::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;

  RCLCPP_INFO(this->get_logger(), "Active project set to: %s.", this->active_project.c_str());

  this->is_working_directory_set = change_working_directory(PROJECTS_DIRECTORY + "/" + this->active_project + "/presenter");
  update_presenter_list();

  if (this->modules.size() > 0) {
    /* Set presenter module to the default if available, otherwise use the first listed module. */
    if (std::find(this->modules.begin(), this->modules.end(), DEFAULT_PRESENTER_NAME) != this->modules.end()) {
      this->set_presenter_module(DEFAULT_PRESENTER_NAME);
    } else {
      this->set_presenter_module(this->modules[0]);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No presenters found in project: %s.", this->active_project.c_str());
    this->unset_presenter_module();
  }

  update_inotify_watch();
}

/* File-system related functions */

bool EegPresenter::change_working_directory(const std::string path) {
  this->working_directory = path;

  /* Check that the directory exists. */
  if (!std::filesystem::exists(this->working_directory) || !std::filesystem::is_directory(this->working_directory)) {
    RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s.", path.c_str());
    return false;
  }

  /* Change the working directory to the project directory. */
  if (chdir(this->working_directory.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s.", this->working_directory.c_str());
    return false;
  }

  return true;
}

std::vector<std::string> EegPresenter::list_python_modules_in_working_directory() {
  std::vector<std::string> modules;

  /* List all .py files in the current working directory. */
  auto path = std::filesystem::current_path();
  for (const auto &entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".py") {
      modules.push_back(entry.path().stem().string());
    }
  }
  return modules;
}

void EegPresenter::update_inotify_watch() {
  /* Remove the old watch. */
  inotify_rm_watch(inotify_descriptor, watch_descriptor);

  /* Add a new watch. */
  watch_descriptor = inotify_add_watch(inotify_descriptor, this->working_directory.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE | IN_MOVE);
  if (watch_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error adding watch for: %s", this->working_directory.c_str());
      return;
  }
}

void EegPresenter::inotify_timer_callback() {
  int length = read(inotify_descriptor, inotify_buffer, 1024);

  if (length < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      /* No events, return early. */
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error reading inotify");
      return;
    }
  }

  int i = 0;
  while (i < length) {
    struct inotify_event *event = (struct inotify_event *)&inotify_buffer[i];
    if (event->len) {
      std::string event_name = event->name;
      if ((event->mask & IN_MODIFY) &&
          (event_name == this->module_name + ".py")) {

        RCLCPP_INFO(this->get_logger(), "The current module '%s' was modified, re-initializing.", this->module_name.c_str());
        this->initialize_presenter_module();
      }
      if (event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) {
        RCLCPP_INFO(this->get_logger(), "File '%s' created, deleted, or moved, updating presenter list.", event_name.c_str());
        this->update_presenter_list();
      }
    }
    i += sizeof(struct inotify_event) + event->len;
  }
}

void EegPresenter::update_presenter_list() {
  if (this->is_working_directory_set) {
    this->modules = this->list_python_modules_in_working_directory();
  } else {
    this->modules.clear();
  }
  auto msg = project_interfaces::msg::PresenterList();
  msg.scripts = this->modules;

  this->presenter_list_publisher->publish(msg);
}

void EegPresenter::update_time(double_t time) {
  // Return early if no stimuli are queued
  if (this->sensory_stimuli.empty()) {
    return;
  }

  auto stimulus = this->sensory_stimuli.top();
  double_t stimulus_time = stimulus->time;

  // If the stimulus time is in the future, return early
  if (time <= stimulus_time) {
    return;
  }

  this->sensory_stimuli.pop();

  // Log the stimulus information
  RCLCPP_INFO(this->get_logger(), "Presenting stimulus:");
  RCLCPP_INFO(this->get_logger(), "  - Time: %.3f\n", stimulus_time);
  RCLCPP_INFO(this->get_logger(), "  - Type: %s\n", stimulus->type.c_str());

  if (!stimulus->parameters.empty()) {
    RCLCPP_INFO(this->get_logger(), "  - Parameters:");
    for (const auto &kv : stimulus->parameters) {
      RCLCPP_INFO(this->get_logger(), "    - %s: %s", kv.key.c_str(), kv.value.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "");  // blank line
  }

  // Process the stimulus using the presenter wrapper
  bool success = this->presenter_wrapper->process(*stimulus);

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Error presenting stimulus");
  }
}


void EegPresenter::handle_sensory_stimulus(const std::shared_ptr<pipeline_interfaces::msg::SensoryStimulus> msg) {
  RCLCPP_INFO(this->get_logger(), "Received sensory stimulus (type: %s, time: %.3f)", msg->type.c_str(), msg->time);

  if (!this->enabled) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Presenter not enabled, ignoring stimulus.");
    return;
  }
  if (this->module_name == UNSET_STRING) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Presenter enabled but not selected, ignoring stimulus.");
    return;
  }
  if (!this->presenter_wrapper->is_initialized()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "Presenter enabled and selected but not initialized, ignoring stimulus.");
    return;
  }
  if (this->presenter_wrapper->error_occurred()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         1000,
                         "An error occurred in presenter module, please re-initialize. Ignoring stimulus.");
    return;
  }

  this->sensory_stimuli.push(msg);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("presenter");

  realtime_utils::MemoryConfig mem_config;
  mem_config.enable_memory_optimization = true;
  mem_config.preallocate_size = 10 * 1024 * 1024; // 10 MB

  realtime_utils::SchedulingConfig sched_config;
  sched_config.enable_scheduling_optimization = true;
  sched_config.scheduling_policy = SCHED_RR;
  sched_config.priority_level = realtime_utils::PriorityLevel::REALTIME;

  try {
    realtime_utils::initialize_scheduling(sched_config, logger);
    realtime_utils::initialize_memory(mem_config, logger);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Initialization failed: %s", e.what());
    return -1;
  }

  auto node = std::make_shared<EegPresenter>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
