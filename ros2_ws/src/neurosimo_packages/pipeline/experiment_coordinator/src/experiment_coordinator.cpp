#include "experiment_coordinator.h"
#include <chrono>
#include <algorithm>
#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

using namespace std::chrono;
using namespace std::placeholders;
using namespace experiment_coordinator;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string PULSE_EVENT_TOPIC = "/experiment/pulse_events";
const std::string HEALTHCHECK_TOPIC = "/experiment/coordinator/healthcheck";
const std::string PROJECTS_DIRECTORY = "/app/projects";
const uint16_t EEG_QUEUE_LENGTH = 65535;

/* XXX: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 5ms;

ExperimentCoordinator::ExperimentCoordinator() 
  : Node("experiment_coordinator"), 
    logger(rclcpp::get_logger("experiment_coordinator")),
    protocol_loader(rclcpp::get_logger("protocol_loader")) {
  
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Initializing Experiment Coordinator");
  
  /* Publisher for healthcheck. */
  this->healthcheck_publisher = this->create_publisher<system_interfaces::msg::Healthcheck>(
    HEALTHCHECK_TOPIC, 10);
  
  /* Subscriber for session. */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);
  
  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);
  
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.deadline_callback = 
    [this]([[maybe_unused]] rclcpp::QOSDeadlineRequestedInfo & event) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        "Session not received within deadline.");
    };
  
  this->session_subscriber = this->create_subscription<system_interfaces::msg::Session>(
    "/system/session",
    qos_session,
    std::bind(&ExperimentCoordinator::handle_session, this, _1),
    subscription_options);
  
  /* Publisher for enriched EEG data. */
  this->enriched_eeg_publisher = this->create_publisher<eeg_msgs::msg::Sample>(
    EEG_ENRICHED_TOPIC, EEG_QUEUE_LENGTH);
  
  /* Subscriber for raw EEG data. */
  this->raw_eeg_subscriber = this->create_subscription<eeg_msgs::msg::Sample>(
    EEG_RAW_TOPIC,
    EEG_QUEUE_LENGTH,
    std::bind(&ExperimentCoordinator::handle_raw_sample, this, _1));
  
  /* Subscriber for pulse events. */
  this->pulse_event_subscriber = this->create_subscription<std_msgs::msg::Empty>(
    PULSE_EVENT_TOPIC,
    100,
    std::bind(&ExperimentCoordinator::handle_pulse_event, this, _1));
  
  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  
  this->active_project_subscriber = this->create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&ExperimentCoordinator::handle_set_active_project, this, _1));
  
  /* Publisher for listing protocols. */
  this->protocol_list_publisher = this->create_publisher<project_interfaces::msg::ProtocolList>(
    "/experiment/protocol/list",
    qos_persist_latest);
  
  /* Service for changing protocol. */
  this->set_protocol_service = this->create_service<project_interfaces::srv::SetProtocol>(
    "/experiment/protocol/set",
    std::bind(&ExperimentCoordinator::handle_set_protocol, this, _1, _2));
  
  /* Publisher for protocol module. */
  this->protocol_module_publisher = this->create_publisher<std_msgs::msg::String>(
    "/experiment/protocol",
    qos_persist_latest);
  
  /* Services for pause/resume. */
  this->pause_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/pause",
    std::bind(&ExperimentCoordinator::handle_pause, this, _1, _2));
  
  this->resume_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/resume",
    std::bind(&ExperimentCoordinator::handle_resume, this, _1, _2));
  
  /* Initialize inotify. */
  this->inotify_descriptor = inotify_init();
  if (this->inotify_descriptor == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error initializing inotify");
    exit(1);
  }
  
  /* Set the inotify descriptor to non-blocking. */
  int flags = fcntl(inotify_descriptor, F_GETFL, 0);
  fcntl(inotify_descriptor, F_SETFL, flags | O_NONBLOCK);
  
  /* Create timers. */
  this->inotify_timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ExperimentCoordinator::inotify_timer_callback, this));
  
  this->healthcheck_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ExperimentCoordinator::publish_healthcheck, this));
  
  RCLCPP_INFO(this->get_logger(), "Experiment Coordinator initialized");
}

void ExperimentCoordinator::publish_healthcheck() {
  auto healthcheck = system_interfaces::msg::Healthcheck();
  
  switch (this->coordinator_state) {
    case CoordinatorState::WAITING_FOR_PROTOCOL:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
      healthcheck.status_message = "No protocol loaded";
      healthcheck.actionable_message = "Please select a protocol.";
      break;
    
    case CoordinatorState::READY:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
      healthcheck.status_message = "Ready";
      healthcheck.actionable_message = "Ready";
      break;
    
    case CoordinatorState::PROTOCOL_ERROR:
      healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::ERROR;
      healthcheck.status_message = "Protocol error";
      healthcheck.actionable_message = "Protocol has an error. Please check logs.";
      break;
  }
  
  this->healthcheck_publisher->publish(healthcheck);
}

void ExperimentCoordinator::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  bool state_changed = this->session_state.value != msg->state.value;
  this->session_state = msg->state;
  
  if (state_changed) {
    if (this->session_state.value == system_interfaces::msg::SessionState::STARTED) {
      RCLCPP_INFO(this->get_logger(), "Session started, resetting experiment state");
      reset_experiment_state();
      state.session_started = true;
    } else if (this->session_state.value == system_interfaces::msg::SessionState::STOPPED) {
      RCLCPP_INFO(this->get_logger(), "Session stopped");
      state.session_started = false;
    }
  }
}

void ExperimentCoordinator::handle_raw_sample(const std::shared_ptr<eeg_msgs::msg::Sample> msg) {
  /* Only process samples during an active session. */
  if (this->session_state.value != system_interfaces::msg::SessionState::STARTED) {
    return;
  }
  
  if (!this->protocol.has_value()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "No protocol loaded, passing through sample without enrichment");
    this->enriched_eeg_publisher->publish(*msg);
    return;
  }
  
  double sample_time = msg->time;
  
  /* Track the most recent sample time. */
  state.last_sample_time = sample_time;
  
  /* Update experiment state based on sample time. */
  update_experiment_state(sample_time);
  
  /* Create enriched sample. */
  auto enriched = *msg;
  
  /* Add experiment state fields. */
  enriched.in_rest = state.in_rest;
  enriched.paused = state.paused;
  enriched.experiment_time = get_experiment_time(sample_time);
  enriched.current_trial = state.current_trial;
  enriched.pulse_count = state.total_pulses;
  
  /* Add stage information. */
  if (!state.in_rest && state.current_element_index < protocol->elements.size()) {
    const auto& element = protocol->elements[state.current_element_index];
    if (element.type == ProtocolElement::Type::STAGE) {
      enriched.current_stage = element.stage->name;
    }
  }
  
  /* Mark as passed experiment coordinator. */
  enriched.metadata.passed_experiment_coordinator = true;
  
  /* Publish enriched sample. */
  this->enriched_eeg_publisher->publish(enriched);
}

void ExperimentCoordinator::handle_pulse_event(const std::shared_ptr<std_msgs::msg::Empty> msg) {
  (void)msg;  // Unused
  
  if (!this->protocol.has_value() || !state.session_started) {
    return;
  }
  
  if (state.paused) {
    RCLCPP_WARN(this->get_logger(), "Pulse received while paused, ignoring");
    return;
  }
  
  if (state.in_rest) {
    RCLCPP_WARN(this->get_logger(), "Pulse received during rest, ignoring");
    return;
  }
  
  state.total_pulses++;
  
  /* Check if we're in a stage. */
  if (state.current_element_index >= protocol->elements.size()) {
    RCLCPP_WARN(this->get_logger(), "Pulse received but protocol is complete");
    return;
  }
  
  const auto& element = protocol->elements[state.current_element_index];
  
  if (element.type != ProtocolElement::Type::STAGE) {
    RCLCPP_WARN(this->get_logger(), "Pulse received but not in a stage");
    return;
  }
  
  const auto& stage = element.stage.value();
  state.current_trial++;
  
  RCLCPP_INFO(this->get_logger(), "Pulse %u: Stage '%s' trial %u/%u",
    state.total_pulses, stage.name.c_str(), state.current_trial, stage.trials);
  
  /* Check if stage is complete. */
  if (state.current_trial >= stage.trials) {
    RCLCPP_INFO(this->get_logger(), "Stage '%s' complete (%u trials)",
      stage.name.c_str(), stage.trials);
    
    /* Advance to next element. */
    advance_to_next_element();
  }
}

void ExperimentCoordinator::handle_pause(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;  // Unused
  
  if (state.paused) {
    response->success = false;
    response->message = "Already paused";
    return;
  }
  
  state.paused = true;
  state.pause_start_time = state.last_sample_time;
  
  RCLCPP_INFO(this->get_logger(), "Experiment paused at %.2f s", 
    get_experiment_time(state.last_sample_time));
  
  response->success = true;
  response->message = "Experiment paused";
}

void ExperimentCoordinator::handle_resume(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;  // Unused
  
  if (!state.paused) {
    response->success = false;
    response->message = "Not paused";
    return;
  }
  
  double pause_duration = state.last_sample_time - state.pause_start_time;
  state.total_pause_duration += pause_duration;
  state.paused = false;
  
  RCLCPP_INFO(this->get_logger(), "Experiment resumed at %.2f s (paused for %.2f s)",
    get_experiment_time(state.last_sample_time), pause_duration);
  
  response->success = true;
  response->message = "Experiment resumed";
}

void ExperimentCoordinator::update_experiment_state(double current_time) {
  if (state.paused || !protocol.has_value()) {
    return;
  }
  
  /* Check if we're in a rest period. */
  if (state.in_rest && state.rest_target_time.has_value()) {
    /* Check if rest period is complete. */
    if (current_time >= state.rest_target_time.value()) {
      end_rest();
      
      /* Advance to next element if there is one. */
      if (state.current_element_index + 1 < protocol->elements.size()) {
        advance_to_next_element();
      } else {
        state.protocol_complete = true;
        RCLCPP_INFO(this->get_logger(), "Protocol complete!");
      }
    }
  }
}

void ExperimentCoordinator::advance_to_next_element() {
  if (!protocol.has_value()) {
    return;
  }
  
  state.current_element_index++;
  
  if (state.current_element_index >= protocol->elements.size()) {
    state.protocol_complete = true;
    RCLCPP_INFO(this->get_logger(), "Protocol complete!");
    return;
  }
  
  const auto& element = protocol->elements[state.current_element_index];
  double current_time = state.last_sample_time;
  
  if (element.type == ProtocolElement::Type::STAGE) {
    start_stage(element.stage.value(), current_time);
  } else if (element.type == ProtocolElement::Type::REST) {
    start_rest(element.rest.value(), current_time);
  }
}

void ExperimentCoordinator::start_rest(const Rest& rest, double current_time) {
  state.in_rest = true;
  state.rest_start_time = current_time;
  
  /* Calculate target time for rest. */
  if (rest.duration.has_value()) {
    state.rest_target_time = current_time + rest.duration.value();
    RCLCPP_INFO(this->get_logger(), "Rest started (duration: %.1f s)", rest.duration.value());
  } else if (rest.wait_until.has_value()) {
    const auto& wu = rest.wait_until.value();
    
    /* Look up anchor stage start time. */
    if (state.stage_start_times.count(wu.anchor) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot find start time for anchor stage: %s", 
        wu.anchor.c_str());
      state.rest_target_time = current_time;  // End rest immediately
      return;
    }
    
    double anchor_time = state.stage_start_times[wu.anchor];
    state.rest_target_time = anchor_time + wu.offset;
    
    RCLCPP_INFO(this->get_logger(), "Rest started (wait_until: %s + %.1f s, target: %.1f s)",
      wu.anchor.c_str(), wu.offset, state.rest_target_time.value());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Rest has neither duration nor wait_until");
    state.rest_target_time = current_time;  // End rest immediately
  }
}

void ExperimentCoordinator::end_rest() {
  double rest_duration = state.last_sample_time - state.rest_start_time;
  RCLCPP_INFO(this->get_logger(), "Rest ended (duration: %.1f s)", rest_duration);
  
  state.in_rest = false;
  state.rest_target_time.reset();
}

void ExperimentCoordinator::start_stage(const Stage& stage, double current_time) {
  state.current_stage_name = stage.name;
  state.current_trial = 0;
  state.stage_start_times[stage.name] = current_time;
  
  RCLCPP_INFO(this->get_logger(), "Stage '%s' started (%u trials)", 
    stage.name.c_str(), stage.trials);
}

void ExperimentCoordinator::reset_experiment_state() {
  state = ExperimentState();
  
  if (protocol.has_value() && !protocol->elements.empty()) {
    /* Start with first element at time 0.0 (will be updated when first sample arrives). */
    const auto& first_element = protocol->elements[0];
    double initial_time = 0.0;
    
    if (first_element.type == ProtocolElement::Type::STAGE) {
      start_stage(first_element.stage.value(), initial_time);
    } else if (first_element.type == ProtocolElement::Type::REST) {
      start_rest(first_element.rest.value(), initial_time);
    }
  }
}

/* Protocol management */

std::string ExperimentCoordinator::get_protocol_name_with_fallback(const std::string protocol_name) {
  if (std::find(this->available_protocols.begin(), this->available_protocols.end(), protocol_name) 
      != this->available_protocols.end()) {
    return protocol_name;
  }
  
  if (!this->available_protocols.empty()) {
    RCLCPP_WARN(this->get_logger(), "Protocol %s not found, setting to first protocol: %s",
      protocol_name.c_str(), this->available_protocols[0].c_str());
    return this->available_protocols[0];
  }
  
  RCLCPP_WARN(this->get_logger(), "No protocols found in project: %s", this->active_project.c_str());
  return UNSET_STRING;
}

void ExperimentCoordinator::unset_protocol() {
  this->protocol_name = UNSET_STRING;
  this->protocol.reset();
  this->coordinator_state = CoordinatorState::WAITING_FOR_PROTOCOL;
  
  RCLCPP_INFO(this->get_logger(), "Protocol unset");
  
  auto msg = std_msgs::msg::String();
  msg.data = this->protocol_name;
  this->protocol_module_publisher->publish(msg);
}

bool ExperimentCoordinator::load_protocol(const std::string& protocol_name) {
  std::string filepath = this->working_directory + "/" + protocol_name + ".yaml";
  
  RCLCPP_INFO(this->get_logger(), "Loading protocol from: %s", filepath.c_str());
  
  LoadResult result = this->protocol_loader.load_from_file(filepath);
  
  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load protocol: %s", 
      result.error_message.c_str());
    this->coordinator_state = CoordinatorState::PROTOCOL_ERROR;
    return false;
  }
  
  this->protocol = result.protocol;
  this->coordinator_state = CoordinatorState::READY;
  
  RCLCPP_INFO(this->get_logger(), "Protocol loaded successfully: %s", 
    this->protocol->name.c_str());
  
  return true;
}

bool ExperimentCoordinator::set_protocol(const std::string& protocol_name) {
  this->protocol_name = get_protocol_name_with_fallback(protocol_name);
  
  if (this->protocol_name == UNSET_STRING) {
    RCLCPP_ERROR(this->get_logger(), "No protocol set");
    unset_protocol();
    return false;
  }
  
  if (!load_protocol(this->protocol_name)) {
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), "Protocol set to: %s", this->protocol_name.c_str());
  
  auto msg = std_msgs::msg::String();
  msg.data = this->protocol_name;
  this->protocol_module_publisher->publish(msg);
  
  return true;
}

void ExperimentCoordinator::handle_set_protocol(
    const std::shared_ptr<project_interfaces::srv::SetProtocol::Request> request,
    std::shared_ptr<project_interfaces::srv::SetProtocol::Response> response) {
  response->success = set_protocol(request->protocol);
}

void ExperimentCoordinator::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;
  
  RCLCPP_INFO(this->get_logger(), "Project set to: %s", this->active_project.c_str());
  
  this->is_working_directory_set = change_working_directory(
    PROJECTS_DIRECTORY + "/" + this->active_project + "/protocols");
  
  update_protocol_list();
  update_inotify_watch();
}

/* File-system related functions */

bool ExperimentCoordinator::change_working_directory(const std::string& path) {
  this->working_directory = path;
  
  std::error_code ec;
  if (!std::filesystem::exists(this->working_directory, ec) || 
      !std::filesystem::is_directory(this->working_directory, ec)) {
    RCLCPP_ERROR(this->get_logger(), "Directory does not exist: %s", path.c_str());
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", ec.message().c_str());
    }
    return false;
  }
  
  if (std::filesystem::is_symlink(this->working_directory, ec)) {
    auto resolved_path = std::filesystem::canonical(this->working_directory, ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Failed to resolve symlink %s: %s", 
        path.c_str(), ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Resolved symlink %s to %s", 
      path.c_str(), resolved_path.c_str());
    
    if (!std::filesystem::is_directory(resolved_path, ec)) {
      RCLCPP_ERROR(this->get_logger(), "Symlink target is not a directory: %s -> %s",
        path.c_str(), resolved_path.c_str());
      return false;
    }
  }
  
  if (chdir(this->working_directory.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to change working directory to: %s", 
      this->working_directory.c_str());
    return false;
  }
  
  return true;
}

std::vector<std::string> ExperimentCoordinator::list_yaml_files_in_working_directory() {
  std::vector<std::string> files;
  
  std::error_code ec;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(this->working_directory, ec)) {
      if (ec) {
        RCLCPP_WARN(this->get_logger(), "Error accessing directory %s: %s",
          this->working_directory.c_str(), ec.message().c_str());
        return files;
      }
      
      std::error_code entry_ec;
      if (entry.is_regular_file(entry_ec) && !entry_ec && 
          (entry.path().extension() == ".yaml" || entry.path().extension() == ".yml")) {
        files.push_back(entry.path().stem().string());
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(this->get_logger(), "Filesystem error while listing protocols in %s: %s",
      this->working_directory.c_str(), e.what());
    return files;
  }
  
  std::sort(files.begin(), files.end());
  return files;
}

void ExperimentCoordinator::update_protocol_list() {
  if (this->is_working_directory_set) {
    this->available_protocols = this->list_yaml_files_in_working_directory();
  } else {
    this->available_protocols.clear();
  }
  
  auto msg = project_interfaces::msg::ProtocolList();
  msg.protocols = this->available_protocols;
  
  this->protocol_list_publisher->publish(msg);
}

void ExperimentCoordinator::update_inotify_watch() {
  inotify_rm_watch(inotify_descriptor, watch_descriptor);
  
  std::error_code ec;
  if (!std::filesystem::exists(this->working_directory, ec) || 
      !std::filesystem::is_directory(this->working_directory, ec)) {
    RCLCPP_ERROR(this->get_logger(), "Working directory does not exist or is not a directory: %s",
      this->working_directory.c_str());
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", ec.message().c_str());
    }
    return;
  }
  
  watch_descriptor = inotify_add_watch(inotify_descriptor, this->working_directory.c_str(),
    IN_MODIFY | IN_CREATE | IN_DELETE | IN_MOVE);
  
  if (watch_descriptor == -1) {
    RCLCPP_ERROR(this->get_logger(), "Error adding watch for: %s", this->working_directory.c_str());
    return;
  }
}

void ExperimentCoordinator::inotify_timer_callback() {
  int length = read(inotify_descriptor, inotify_buffer, 1024);
  
  if (length < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
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
          (event_name == this->protocol_name + ".yaml" || event_name == this->protocol_name + ".yml")) {
        RCLCPP_INFO(this->get_logger(), "Current protocol '%s' modified, reloading",
          this->protocol_name.c_str());
        load_protocol(this->protocol_name);
      }
      
      if (event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) {
        RCLCPP_INFO(this->get_logger(), "File '%s' created, deleted, or moved, updating protocol list",
          event_name.c_str());
        update_protocol_list();
      }
    }
    i += sizeof(struct inotify_event) + event->len;
  }
}

/* Time utilities */

double ExperimentCoordinator::get_experiment_time(double sample_time) {
  /* Calculate experiment time as sample time minus total pause duration. */
  double experiment_time = sample_time - state.total_pause_duration;
  
  /* If currently paused, also subtract the ongoing pause. */
  if (state.paused) {
    experiment_time -= (sample_time - state.pause_start_time);
  }
  
  return experiment_time;
}

