#include "experiment_coordinator.h"
#include "filesystem_utils/filesystem_utils.h"
#include <chrono>
#include <algorithm>

using namespace std::chrono;
using namespace std::placeholders;
using namespace experiment_coordinator;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string DECISION_TRACE_FINAL_TOPIC = "/pipeline/decision_trace/final";
const std::string HEARTBEAT_TOPIC = "/experiment_coordinator/heartbeat";
const std::string PROJECTS_DIRECTORY = "/app/projects";
const uint16_t EEG_QUEUE_LENGTH = 65535;

ExperimentCoordinator::ExperimentCoordinator()
  : Node("experiment_coordinator"),
    protocol_loader(rclcpp::get_logger("protocol_loader")),
    logger(rclcpp::get_logger("experiment_coordinator")) {

  /* Publisher for heartbeat. */
  this->heartbeat_publisher = this->create_publisher<std_msgs::msg::Empty>(
    HEARTBEAT_TOPIC, 10);

  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  
  /* Publisher for health. */
  this->health_publisher = this->create_publisher<system_interfaces::msg::ComponentHealth>(
    "/experiment_coordinator/health", qos_persist_latest);
  
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
  
  /* Subscriber for decision trace final events. */
  this->decision_trace_final_subscriber = this->create_subscription<pipeline_interfaces::msg::DecisionTrace>(
    DECISION_TRACE_FINAL_TOPIC,
    100,
    std::bind(&ExperimentCoordinator::handle_decision_trace_final, this, _1));

  /* Client for finishing the session. */
  this->finish_session_client = this->create_client<std_srvs::srv::Trigger>(
    "/session/finish");

  // Wait for service client to be available
  while (!this->finish_session_client->wait_for_service(std::chrono::duration<double>(1.0))) {
    RCLCPP_INFO(this->get_logger(), "Service /session/finish not available, waiting...");
  }

  /* Services for pause/resume. */
  this->pause_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/pause",
    std::bind(&ExperimentCoordinator::handle_pause, this, _1, _2));

  this->resume_service = this->create_service<std_srvs::srv::Trigger>(
    "/experiment/resume",
    std::bind(&ExperimentCoordinator::handle_resume, this, _1, _2));

  /* Service for protocol initialization. */
  this->initialize_protocol_service = this->create_service<pipeline_interfaces::srv::InitializeProtocol>(
    "/pipeline/protocol/initialize",
    std::bind(&ExperimentCoordinator::handle_initialize_protocol, this, _1, _2));
  
  /* Service for protocol info. */
  this->get_protocol_info_service = this->create_service<pipeline_interfaces::srv::GetProtocolInfo>(
    "/experiment_coordinator/protocol/get_info",
    std::bind(&ExperimentCoordinator::handle_get_protocol_info, this, _1, _2));
  
  /* Create timers. */
  this->heartbeat_timer = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&ExperimentCoordinator::publish_heartbeat, this));
}

void ExperimentCoordinator::publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void ExperimentCoordinator::publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

void ExperimentCoordinator::handle_raw_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  /* Handle session start marker from EEG bridge/simulator. */
  if (msg->is_session_start) {
    RCLCPP_INFO(this->get_logger(), "Session started, resetting experiment state");

    /* Reset experiment state. */
    reset_experiment_state();

    /* Mark session as ongoing after resetting state (which would set it to false). */
    state.is_session_ongoing = true;

    publish_experiment_state(0.0);
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
    RCLCPP_INFO(this->get_logger(), "Session aborted");
    state.is_session_ongoing = false;
    publish_experiment_state(state.last_sample_time);
  }
}

void ExperimentCoordinator::handle_decision_trace_final(const std::shared_ptr<pipeline_interfaces::msg::DecisionTrace> msg) {
  /* Only process confirmed pulses. */
  if (!msg->pulse_confirmed) {
    return;
  }

  if (!state.is_session_ongoing) {
    RCLCPP_WARN(this->get_logger(), "Session not ongoing, ignoring pulse event");
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

void ExperimentCoordinator::handle_initialize_protocol(
    const std::shared_ptr<pipeline_interfaces::srv::InitializeProtocol::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializeProtocol::Response> response) {

  this->is_protocol_initialized = false;

  RCLCPP_INFO(this->get_logger(), "Initializing protocol: project='%s', protocol='%s'",
    request->project_name.c_str(), request->protocol_filename.c_str());

  LoadResult result = this->protocol_loader.load_from_project(
    PROJECTS_DIRECTORY, request->project_name, request->protocol_filename);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load protocol: %s",
      result.error_message.c_str());

    this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR, 
      "Protocol loading error: " + result.error_message);
    response->success = false;

    return;
  }
  RCLCPP_INFO(this->get_logger(), "Protocol loaded successfully: %s", request->protocol_filename.c_str());

  this->protocol = result.protocol;
  this->is_protocol_initialized = true;

  /* Reset experiment state. */
  reset_experiment_state();

  this->publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");
  response->success = true;
}

void ExperimentCoordinator::handle_get_protocol_info(
    const std::shared_ptr<pipeline_interfaces::srv::GetProtocolInfo::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::GetProtocolInfo::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Getting protocol info for: project='%s', protocol='%s'",
    request->project_name.c_str(), request->filename.c_str());

  experiment_coordinator::ProtocolInfoResult result = 
    this->protocol_loader.get_protocol_info(
      PROJECTS_DIRECTORY, request->project_name, request->filename);

  if (!result.success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get protocol info: %s",
      result.error_message.c_str());
    response->success = false;
    return;
  }

  /* Set response with the ROS message directly. */
  response->protocol_info = result.protocol_info.value();
  response->success = true;
  
  RCLCPP_INFO(this->get_logger(), "Protocol info retrieved: %s (%zu elements)",
    response->protocol_info.name.c_str(), response->protocol_info.elements.size());
}

void ExperimentCoordinator::update_experiment_state(double current_time) {
  if (state.paused || !this->is_protocol_initialized) {
    return;
  }
  
  /* Check if we're in a rest period. */
  if (state.in_rest && state.rest_target_time.has_value()) {
    /* Check if rest period is complete. */
    if (get_experiment_time(current_time) >= state.rest_target_time.value()) {
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
  if (!this->is_protocol_initialized) {
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
  request_finish_session();
  publish_experiment_state(state.last_sample_time);
}

void ExperimentCoordinator::start_rest(const Rest& rest, double current_time) {
  state.in_rest = true;
  state.rest_start_time = get_experiment_time(current_time);
  
  /* Calculate target time for rest. */
  if (rest.duration.has_value()) {
    state.rest_target_time = state.rest_start_time + rest.duration.value();
    RCLCPP_INFO(this->get_logger(), "Rest started (duration: %.1f s)", rest.duration.value());
  } else if (rest.wait_until.has_value()) {
    const auto& wu = rest.wait_until.value();
    
    /* Look up anchor stage start time. */
    if (state.stage_start_experiment_times.count(wu.anchor) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot find start time for anchor stage: %s", 
        wu.anchor.c_str());
      state.rest_target_time = state.rest_start_time;  // End rest immediately
      return;
    }
    
    double anchor_experiment_time = state.stage_start_experiment_times[wu.anchor];
    state.rest_target_time = anchor_experiment_time + wu.offset;
    
    RCLCPP_INFO(this->get_logger(), "Rest started (wait_until: %s + %.1f s, target: %.1f s)",
      wu.anchor.c_str(), wu.offset, state.rest_target_time.value());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Rest has neither duration nor wait_until");
    state.rest_target_time = get_experiment_time(current_time);  // End rest immediately
  }
}

void ExperimentCoordinator::end_rest() {
  double rest_duration = get_experiment_time(state.last_sample_time) - state.rest_start_time;
  RCLCPP_INFO(this->get_logger(), "Rest ended (duration: %.1f s)", rest_duration);
  
  state.in_rest = false;
  state.rest_target_time.reset();
}

void ExperimentCoordinator::start_stage(const Stage& stage, double current_time) {
  state.stage_name = stage.name;
  state.trial = 0;
  state.stage_start_times[stage.name] = current_time;
  state.stage_start_experiment_times[stage.name] = get_experiment_time(current_time);
  
  RCLCPP_INFO(this->get_logger(), "Stage '%s' started (%u trials)", 
    stage.name.c_str(), stage.trials);
}

void ExperimentCoordinator::reset_experiment_state() {
  state = ExperimentState();
  
  if (this->is_protocol_initialized && this->protocol && !this->protocol->elements.empty()) {
    /* Start with first element at time 0.0. */
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

void ExperimentCoordinator::request_finish_session() {
  if (!this->finish_session_client) {
    RCLCPP_ERROR(this->get_logger(), "Finish session client not initialized.");
    return;
  }
  
  if (!this->finish_session_client->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "Finish session client not available yet.");
  }
  
  RCLCPP_INFO(this->get_logger(), "Requesting session to finish...");
  
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  this->finish_session_client->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(this->get_logger(), "Session finish service responded with failure: %s", 
            response->message.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "Session finished successfully: %s",
            response->message.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error calling session finish service: %s", 
          e.what());
      }
    });
}

void ExperimentCoordinator::publish_experiment_state(double current_time) {
  if (!this->experiment_state_publisher) {
    return;
  }
  
  pipeline_interfaces::msg::ExperimentState msg;
  
  const bool has_protocol = this->is_protocol_initialized;
  const bool ongoing = state.is_session_ongoing && !state.protocol_complete;
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
  
  if (this->is_protocol_initialized) {
    for (size_t i = 0; i < this->protocol->elements.size(); ++i) {
      if (this->protocol->elements[i].type == ProtocolElement::Type::STAGE) {
        if (i <= state.current_element_index) {
          stage_index = total_stages;
        }
        total_stages++;
      }
    }
    
    if (state.current_element_index < this->protocol->elements.size()) {
      const auto& element = this->protocol->elements[state.current_element_index];
      if (element.type == ProtocolElement::Type::STAGE && element.stage.has_value()) {
        msg.stage_name = element.stage->name;
        total_trials_in_stage = element.stage->trials;
      }
    }
  }
  
  msg.stage_index = static_cast<uint32_t>(stage_index);
  msg.total_stages = static_cast<uint32_t>(total_stages);
  msg.trial = state.in_rest ? 0 : state.trial;
  msg.total_trials_in_stage = total_trials_in_stage;
  
  /* Stage timing */
  double stage_start_experiment_time = 0.0;
  if (!msg.stage_name.empty() && state.stage_start_experiment_times.count(msg.stage_name)) {
    stage_start_experiment_time = state.stage_start_experiment_times[msg.stage_name];
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
      rest_remaining = state.rest_target_time.value() - experiment_time;
    }
    
    msg.rest_duration = rest_duration;
    msg.rest_elapsed = experiment_time - state.rest_start_time;
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
  
  /* If currently paused, also subtract the ongoing pause, but only if the 
     sample_time is after the pause started. */
  if (state.paused && sample_time > state.pause_start_time) {
    experiment_time -= (sample_time - state.pause_start_time);
  }
  
  return experiment_time;
}

