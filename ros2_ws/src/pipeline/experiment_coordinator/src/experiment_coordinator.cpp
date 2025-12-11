#include "experiment_coordinator.h"
#include <chrono>
#include <algorithm>

using namespace std::chrono;
using namespace std::placeholders;
using namespace experiment_coordinator;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string PULSE_EVENT_TOPIC = "/pipeline/pulse_events";
const std::string HEALTHCHECK_TOPIC = "/experiment/coordinator/healthcheck";
const std::string PROJECTS_DIRECTORY = "/app/projects";
const uint16_t EEG_QUEUE_LENGTH = 65535;

const std::string DEFAULT_PROTOCOL_NAME = "example";

ExperimentCoordinator::ExperimentCoordinator()
  : Node("experiment_coordinator"),
    protocol_loader(rclcpp::get_logger("protocol_loader")),
    logger(rclcpp::get_logger("experiment_coordinator")) {

  /* Publisher for healthcheck. */
  this->healthcheck_publisher = this->create_publisher<system_interfaces::msg::Healthcheck>(
    HEALTHCHECK_TOPIC, 10);

  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  
  /* Publisher for enriched EEG data. */
  this->enriched_eeg_publisher = this->create_publisher<eeg_interfaces::msg::Sample>(
    EEG_ENRICHED_TOPIC, EEG_QUEUE_LENGTH);
  
  /* Publisher for experiment UI state. */
  this->experiment_state_publisher = this->create_publisher<pipeline_interfaces::msg::ExperimentState>(
    "/pipeline/experiment_state", qos_persist_latest);
  
  /* Subscriber for raw EEG data. */
  this->raw_eeg_subscriber = this->create_subscription<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    EEG_QUEUE_LENGTH,
    std::bind(&ExperimentCoordinator::handle_raw_sample, this, _1));
  
  /* Subscriber for pulse events. */
  this->pulse_event_subscriber = this->create_subscription<std_msgs::msg::Empty>(
    PULSE_EVENT_TOPIC,
    100,
    std::bind(&ExperimentCoordinator::handle_pulse_event, this, _1));
  
  /* Subscriber for active project. */
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
  
  /* Clients for stopping EEG simulator or bridge when protocol completes. */
  this->stop_simulator_client = this->create_client<std_srvs::srv::Trigger>(
    "/eeg_simulator/stop");
  
  this->stop_bridge_client = this->create_client<std_srvs::srv::Trigger>(
    "/eeg_bridge/stop");
  
  /* Services for pause/resume. */
  this->pause_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/pause",
    std::bind(&ExperimentCoordinator::handle_pause, this, _1, _2));
  
  this->resume_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/resume",
    std::bind(&ExperimentCoordinator::handle_resume, this, _1, _2));
  
  /* Initialize inotify watcher. */
  this->inotify_watcher = std::make_unique<inotify_utils::InotifyWatcher>(
    this, this->get_logger());
  this->inotify_watcher->set_change_callback(
    std::bind(&ExperimentCoordinator::update_protocol_list, this));
  
  /* Create timers. */
  this->healthcheck_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ExperimentCoordinator::publish_healthcheck, this));
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

void ExperimentCoordinator::handle_raw_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Handle session start marker from EEG bridge/simulator. */
  if (msg->is_session_start) {
    RCLCPP_INFO(this->get_logger(), "Session started, resetting experiment state");
    reset_experiment_state();
    state.session_started = true;
    this->is_simulation = msg->session.is_simulation;
    publish_experiment_state(0.0);
  }

  /* Only process samples during an active session. */
  if (!state.session_started) {
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
  enriched.trial = state.trial;
  enriched.pulse_count = state.total_pulses;

  /* Add stage information. */
  if (!state.in_rest && state.current_element_index < protocol->elements.size()) {
    const auto& element = protocol->elements[state.current_element_index];
    if (element.type == ProtocolElement::Type::STAGE) {
      enriched.stage_name = element.stage->name;
    }
  }

  /* Publish enriched sample. */
  this->enriched_eeg_publisher->publish(enriched);

  publish_experiment_state(sample_time);

  /* Handle session end marker from EEG bridge/simulator (after publishing enriched sample). */
  if (msg->is_session_end) {
    RCLCPP_INFO(this->get_logger(), "Session stopped");
    state.session_started = false;
    publish_experiment_state(state.last_sample_time);
  }
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
  state.trial++;
  
  RCLCPP_INFO(this->get_logger(), "Pulse %u: Stage '%s' trial %u/%u",
    state.total_pulses, stage.name.c_str(), state.trial, stage.trials);
  
  /* Check if stage is complete. */
  if (state.trial >= stage.trials) {
    RCLCPP_INFO(this->get_logger(), "Stage '%s' complete (%u trials)",
      stage.name.c_str(), stage.trials);
    
    /* Advance to next element. */
    advance_to_next_element();
  }
  
  publish_experiment_state(state.last_sample_time);
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
  publish_experiment_state(state.last_sample_time);
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
  publish_experiment_state(state.last_sample_time);
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
        mark_protocol_complete();
      }
    }
  }
  
  publish_experiment_state(current_time);
}

void ExperimentCoordinator::advance_to_next_element() {
  if (!protocol.has_value()) {
    return;
  }
  
  state.current_element_index++;
  
  if (state.current_element_index >= protocol->elements.size()) {
    mark_protocol_complete();
    return;
  }
  
  const auto& element = protocol->elements[state.current_element_index];
  double current_time = state.last_sample_time;
  
  if (element.type == ProtocolElement::Type::STAGE) {
    start_stage(element.stage.value(), current_time);
  } else if (element.type == ProtocolElement::Type::REST) {
    start_rest(element.rest.value(), current_time);
  }
  
  publish_experiment_state(current_time);
}

void ExperimentCoordinator::mark_protocol_complete() {
  if (state.protocol_complete) {
    return;
  }
  
  state.protocol_complete = true;
  RCLCPP_INFO(this->get_logger(), "Protocol complete! Requesting session stop.");
  request_stop_session();
  publish_experiment_state(state.last_sample_time);
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
  state.stage_name = stage.name;
  state.trial = 0;
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
  
  publish_experiment_state(0.0);
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
  
  if (this->is_working_directory_set) {
    this->inotify_watcher->update_watch(this->working_directory);
  }
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

void ExperimentCoordinator::request_stop_session() {
  /* Choose the appropriate stop service based on whether we're in simulation mode. */
  auto stop_client = this->is_simulation ? this->stop_simulator_client : this->stop_bridge_client;
  const char* service_name = this->is_simulation ? "EEG simulator" : "EEG bridge";
  
  if (!stop_client) {
    RCLCPP_ERROR(this->get_logger(), "%s stop client not initialized.", service_name);
    return;
  }
  
  if (!stop_client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "%s stop service not available yet.", service_name);
  }
  
  RCLCPP_INFO(this->get_logger(), "Requesting %s to stop streaming...", service_name);
  
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  stop_client->async_send_request(
    request,
    [this, service_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(this->get_logger(), "%s stop service responded with failure: %s", 
            service_name, response->message.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "%s stop requested successfully: %s",
            service_name, response->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error calling %s stop service: %s", 
          service_name, e.what());
      }
    });
}

void ExperimentCoordinator::publish_experiment_state(double current_time) {
  if (!this->experiment_state_publisher) {
    return;
  }
  
  pipeline_interfaces::msg::ExperimentState msg;
  
  const bool has_protocol = protocol.has_value();
  const bool ongoing = state.session_started && !state.protocol_complete;
  const auto experiment_time = get_experiment_time(current_time);
  
  msg.ongoing = ongoing;
  
  /* If not ongoing, publish a clean, cleared state for UI consumers. */
  if (!ongoing) {
    msg.in_rest = false;
    msg.paused = false;
    msg.experiment_time = 0.0;
    msg.stage_name = "";
    msg.stage_index = 0;
    msg.total_stages = 0;
    msg.trial = 0;
    msg.total_trials_in_stage = 0;
    msg.stage_start_time = 0.0;
    msg.stage_elapsed_time = 0.0;
    msg.rest_duration = 0.0;
    msg.rest_elapsed = 0.0;
    msg.rest_remaining = 0.0;
    msg.next_stage_name = "";
    msg.next_is_rest = false;
    this->experiment_state_publisher->publish(msg);
    return;
  }
  
  msg.in_rest = state.in_rest;
  msg.paused = state.paused;
  msg.experiment_time = experiment_time;
  
  /* Stage info */
  size_t total_stages = 0;
  size_t stage_index = 0;
  uint32_t total_trials_in_stage = 0;
  
  if (has_protocol) {
    for (size_t i = 0; i < protocol->elements.size(); ++i) {
      if (protocol->elements[i].type == ProtocolElement::Type::STAGE) {
        if (i <= state.current_element_index) {
          stage_index = total_stages;
        }
        total_stages++;
      }
    }
    
    if (state.current_element_index < protocol->elements.size()) {
      const auto& element = protocol->elements[state.current_element_index];
      if (element.type == ProtocolElement::Type::STAGE && element.stage.has_value()) {
        msg.stage_name = element.stage->name;
        total_trials_in_stage = element.stage->trials;
      }
    }
  }
  
  msg.stage_index = static_cast<uint32_t>(stage_index);
  msg.total_stages = static_cast<uint32_t>(total_stages);
  msg.trial = state.trial;
  msg.total_trials_in_stage = total_trials_in_stage;
  
  /* Stage timing */
  double stage_start_experiment_time = 0.0;
  if (!msg.stage_name.empty() && state.stage_start_times.count(msg.stage_name)) {
    double stage_start_sample_time = state.stage_start_times[msg.stage_name];
    stage_start_experiment_time = get_experiment_time(stage_start_sample_time);
  }
  msg.stage_start_time = stage_start_experiment_time;
  msg.stage_elapsed_time = experiment_time - stage_start_experiment_time;
  if (msg.stage_elapsed_time < 0.0) {
    msg.stage_elapsed_time = 0.0;
  }
  
  /* Rest info */
  if (state.in_rest) {
    double rest_duration = 0.0;
    double rest_remaining = 0.0;
    
    if (state.rest_target_time.has_value()) {
      rest_duration = state.rest_target_time.value() - state.rest_start_time;
      rest_remaining = state.rest_target_time.value() - current_time;
    }
    
    msg.rest_duration = rest_duration;
    msg.rest_elapsed = current_time - state.rest_start_time;
    msg.rest_remaining = std::max(0.0, rest_remaining);
  } else {
    msg.rest_duration = 0.0;
    msg.rest_elapsed = 0.0;
    msg.rest_remaining = 0.0;
  }
  
  /* Next stage preview */
  msg.next_is_rest = false;
  msg.next_stage_name = "";
  if (has_protocol && state.current_element_index + 1 < protocol->elements.size()) {
    for (size_t idx = state.current_element_index + 1; idx < protocol->elements.size(); ++idx) {
      const auto& next_elem = protocol->elements[idx];
      if (next_elem.type == ProtocolElement::Type::STAGE && next_elem.stage.has_value()) {
        msg.next_stage_name = next_elem.stage->name;
        break;
      } else if (next_elem.type == ProtocolElement::Type::REST) {
        msg.next_is_rest = true;
        break;
      }
    }
  }
  
  this->experiment_state_publisher->publish(msg);
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

