#include "headers/trial_performer.h"

using namespace std::chrono_literals;

TrialPerformerNode::TrialPerformerNode(const rclcpp::NodeOptions &options)
    : Node("trial_performer_node", options),
      callback_group(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
      reentrant_callback_group(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
      id_counter(0) {

  initialize_actions();
  initialize_service_clients();
  initialize_subscribers();
  initialize_publishers();
}

void TrialPerformerNode::initialize_actions() {
  /* Action server for performing trial. */
  action_server = rclcpp_action::create_server<experiment_interfaces::action::PerformTrial>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "/trial/perform",
      std::bind(&TrialPerformerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TrialPerformerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&TrialPerformerNode::handle_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(),
      callback_group);

  /* Action client for setting voltages. */
  set_voltages_client = rclcpp_action::create_client<mtms_device_interfaces::action::SetVoltages>(
      this, "/mtms_device/set_voltages");

  while (!set_voltages_client->wait_for_action_server(2s)) {
    RCLCPP_INFO(get_logger(), "Action /mtms_device/set_voltages not available, waiting...");
  }

  /* Action client for analyzing MEP. */
  analyze_mep_client = rclcpp_action::create_client<mep_interfaces::action::AnalyzeMep>(
      this, "/mep/analyze");

  while (!analyze_mep_client->wait_for_action_server(2s)) {
    RCLCPP_INFO(get_logger(), "Action /mep/analyze not available, waiting...");
  }
}

void TrialPerformerNode::initialize_service_clients() {
  targeting_client = this->create_client<targeting_interfaces::srv::GetTargetVoltages>("/targeting/get_target_voltages");

  reverse_polarity_client = this->create_client<targeting_interfaces::srv::ReversePolarity>("/waveforms/reverse_polarity");
  while (!reverse_polarity_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /waveforms/reverse_polarity not available, waiting...");
  }

  get_default_waveform_client = this->create_client<targeting_interfaces::srv::GetDefaultWaveform>(
      "/waveforms/get_default",
      rclcpp::QoS(rclcpp::ServicesQoS()),
      reentrant_callback_group);
  while (!get_default_waveform_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /waveforms/get_default not available, waiting...");
  }

  get_multipulse_waveforms_client = this->create_client<targeting_interfaces::srv::GetMultipulseWaveforms>("/waveforms/get_multipulse_waveforms");
  while (!get_multipulse_waveforms_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /waveforms/get_multipulse_waveforms not available, waiting...");
  }

  request_events_client = this->create_client<mtms_device_interfaces::srv::RequestEvents>("/mtms_device/request_events");
  while (!request_events_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /mtms_device/request_events not available, waiting...");
  }
}

void TrialPerformerNode::initialize_subscribers() {
  system_statesubscriber = this->create_subscription<mtms_device_interfaces::msg::SystemState>(
      "/mtms_device/system_state",
      1,
      std::bind(&TrialPerformerNode::handle_system_state, this, std::placeholders::_1));

  session_subscriber = this->create_subscription<system_interfaces::msg::Session>(
      "/system/session",
      1,
      std::bind(&TrialPerformerNode::handle_session, this, std::placeholders::_1));

  pulse_feedback_subscriber = this->create_subscription<event_interfaces::msg::PulseFeedback>(
      "/event/pulse_feedback", 10,
      std::bind(&TrialPerformerNode::update_pulse_feedback, this, std::placeholders::_1));

  trigger_out_feedback_subscriber = this->create_subscription<event_interfaces::msg::TriggerOutFeedback>(
      "/event/trigger_out_feedback", 10,
      std::bind(&TrialPerformerNode::update_trigger_out_feedback, this, std::placeholders::_1));
}

void TrialPerformerNode::initialize_publishers() {
  create_marker_publisher = this->create_publisher<neuronavigation_interfaces::msg::CreateMarker>("/neuronavigation/create_marker", 10);
}

/* Subscriber callbacks */

void TrialPerformerNode::handle_system_state(const mtms_device_interfaces::msg::SystemState::SharedPtr msg) {
  system_state = msg;
}

void TrialPerformerNode::handle_session(const system_interfaces::msg::Session::SharedPtr msg) {
  session = msg;
}

void TrialPerformerNode::update_pulse_feedback(const event_interfaces::msg::PulseFeedback::SharedPtr msg) {
  pulse_feedback[msg->id] = msg;

  auto error_code = msg->error.value;
  auto execution_time = msg->execution_time;

  /* If the event fails, the execution time will be 0; to avoid confusion, do not log it in that case. */
  if (error_code == 0) {
    RCLCPP_INFO(get_logger(), "Pulse event %d finished with error code %d at %.3f s", msg->id, msg->error.value, msg->execution_time);
  } else {
    RCLCPP_WARN(get_logger(), "Pulse event %d finished with error code %d", msg->id, msg->error.value);
  }
}

void TrialPerformerNode::update_trigger_out_feedback(const event_interfaces::msg::TriggerOutFeedback::SharedPtr msg) {
  trigger_out_feedback[msg->id] = msg;

  auto error_code = msg->error.value;
  auto execution_time = msg->execution_time;

  /* If the event fails, the execution time will be 0; to avoid confusion, do not log it in that case. */
  if (error_code == 0) {
    RCLCPP_INFO(get_logger(), "Trigger out event %d finished with error code %d at %.3f s", msg->id, msg->error.value, msg->execution_time);
  } else {
    RCLCPP_WARN(get_logger(), "Trigger out event %d finished with error code %d", msg->id, msg->error.value);
  }
}

/* Using publishers */

void TrialPerformerNode::create_marker() {
  RCLCPP_INFO(this->get_logger(), "Creating marker...");

  auto msg = std::make_shared<neuronavigation_interfaces::msg::CreateMarker>();
  create_marker_publisher->publish(*msg);
}

/* Helpers */

bool TrialPerformerNode::check_trial_feasible() {
  if (!is_device_started()) {
    RCLCPP_WARN(this->get_logger(), "Device not started.");
    return false;
  }
  if (!is_session_started()) {
    RCLCPP_WARN(this->get_logger(), "Session not started.");
    return false;
  }
  return true;
}

bool TrialPerformerNode::is_device_started() const {
  return system_state && system_state->device_state.value == mtms_device_interfaces::msg::DeviceState::OPERATIONAL;
}

bool TrialPerformerNode::is_session_started() const {
  return session && session->state.value == system_interfaces::msg::SessionState::STARTED;
}

double TrialPerformerNode::get_current_time() const {
  if (session) {
    return session->time;
  }
  return 0.0;
}

int TrialPerformerNode::get_next_id() {
  return ++id_counter;
}

std::vector<uint16_t> TrialPerformerNode::get_actual_voltages() const {
  std::vector<uint16_t> voltages;
  for (uint8_t i = 0; i < NUM_OF_CHANNELS; ++i) {
    voltages.push_back(system_state->channel_states[i].voltage);
  }
  return voltages;
}

/* ROS message creation */

std::pair<std::vector<event_interfaces::msg::Pulse>, std::vector<uint16_t>> TrialPerformerNode::create_pulses(const std::vector<event_interfaces::msg::WaveformsForCoilSet> &waveforms, const experiment_interfaces::msg::Trial &trial, double start_time) {
  std::vector<uint16_t> pulse_ids;
  std::vector<event_interfaces::msg::Pulse> pulses;

  for (uint8_t i = 0; i < trial.targets.size(); ++i) {
    auto waveforms_for_coil_set = waveforms[i];
    for (uint8_t channel = 0; channel < NUM_OF_CHANNELS; ++channel) {
      uint16_t id = get_next_id();
      auto waveform = waveforms_for_coil_set.waveforms[channel];
      auto pulse = create_pulse(id, channel, waveform, start_time + trial.pulse_times_since_trial_start[i], event_interfaces::msg::ExecutionCondition::TIMED);
      pulse_ids.push_back(id);
      pulses.push_back(pulse);
    }
  }

  return {pulses, pulse_ids};
}

event_interfaces::msg::Pulse TrialPerformerNode::create_pulse(uint16_t id, uint8_t channel, const event_interfaces::msg::Waveform &waveform, double time, uint8_t execution_condition) {
  event_interfaces::msg::EventInfo event_info;
  event_info.id = id;
  event_info.execution_condition.value = execution_condition;
  event_info.execution_time = time;

  event_interfaces::msg::Pulse pulse;
  pulse.event_info = event_info;
  pulse.channel = channel;
  pulse.waveform = waveform;

  return pulse;
}

std::pair<std::vector<event_interfaces::msg::TriggerOut>, std::vector<uint16_t>> TrialPerformerNode::create_trigger_outs(const std::vector<experiment_interfaces::msg::TriggerConfig> &triggers, double pulse_time) {
  std::vector<uint16_t> trigger_out_ids;
  std::vector<event_interfaces::msg::TriggerOut> trigger_outs;

  for (uint8_t i = 0; i < triggers.size(); ++i) {
    if (triggers[i].enabled) {
      int id = get_next_id();
      auto trigger_out = create_trigger_out(id, pulse_time + triggers[i].delay, event_interfaces::msg::ExecutionCondition::TIMED, i + 1);
      trigger_out_ids.push_back(id);
      trigger_outs.push_back(trigger_out);
    }
  }

  return {trigger_outs, trigger_out_ids};
}

event_interfaces::msg::TriggerOut TrialPerformerNode::create_trigger_out(uint16_t id, double time, uint8_t execution_condition, uint8_t port) {
  event_interfaces::msg::EventInfo event_info;
  event_info.id = id;
  event_info.execution_condition.value = execution_condition;
  event_info.execution_time = time;

  event_interfaces::msg::TriggerOut trigger_out;
  trigger_out.event_info = event_info;
  trigger_out.port = port;
  trigger_out.duration_us = TRIGGER_DURATION_US;

  return trigger_out;
}

/* Events */

bool TrialPerformerNode::wait_for_events_to_finish(const std::vector<uint16_t> &pulse_ids, const std::vector<uint16_t> &trigger_out_ids) {
  while (true) {
    bool all_pulses_finished = std::all_of(pulse_ids.begin(), pulse_ids.end(), [this](uint16_t id) {
      return pulse_feedback.find(id) != pulse_feedback.end();
    });

    bool all_triggers_finished = std::all_of(trigger_out_ids.begin(), trigger_out_ids.end(), [this](uint16_t id) {
      return trigger_out_feedback.find(id) != trigger_out_feedback.end();
    });

    if (all_pulses_finished && all_triggers_finished) {
      break;
    }

    std::this_thread::sleep_for(1ms);
  }

  /* Check that all error codes are zero. */
  bool all_error_codes_zero = true;
  for (const auto &entry : pulse_feedback) {
    if (entry.second->error.value != 0) {
      all_error_codes_zero = false;
      break;
    }
  }
  for (const auto &entry : trigger_out_feedback) {
    if (entry.second->error.value != 0) {
      all_error_codes_zero = false;
      break;
    }
  }

  /* Clear feedback. */
  pulse_feedback.clear();
  trigger_out_feedback.clear();

  return all_error_codes_zero;
}

/* Logging */

void TrialPerformerNode::log_trial(const experiment_interfaces::msg::Trial &trial) {
  auto timing = trial.timing;
  RCLCPP_INFO(this->get_logger(), "Trial:");
  if (trial.config.dry_run) {
    RCLCPP_INFO(this->get_logger(), "  Dry run");
  }
  RCLCPP_INFO(this->get_logger(), "  Number of targets: %zu", trial.targets.size());
  if (!trial.config.dry_run) {
    RCLCPP_INFO(this->get_logger(), "  Timing:");
    RCLCPP_INFO(this->get_logger(), "    Desired start time: %.3f s", timing.desired_start_time);
    RCLCPP_INFO(this->get_logger(), "    Allow trial to be late: %s", timing.allow_late ? "true" : "false");
  }
}

void TrialPerformerNode::log_voltages(const std::vector<uint16_t> &voltages, const std::string &prefix) {
  RCLCPP_INFO(this->get_logger(), "%s:", prefix.c_str());
  for (uint8_t i = 0; i < voltages.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  Channel %d: %d V", i, voltages[i]);
  }
}

void TrialPerformerNode::tic() {
  start_time = std::chrono::high_resolution_clock::now();
}

void TrialPerformerNode::toc(const std::string &prefix) {
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  double duration_ms = duration.count() / 1000.0;

  RCLCPP_INFO(this->get_logger(), "%s: %.1f ms", prefix.c_str(), duration_ms);
}

/* Service requests */

std::pair<std::vector<uint16_t>, std::vector<event_interfaces::msg::WaveformsForCoilSet>> TrialPerformerNode::get_approximated_waveforms(
    const std::vector<targeting_interfaces::msg::ElectricTarget> &targets,
    const std::vector<event_interfaces::msg::WaveformsForCoilSet> &target_waveforms) {

  auto request = std::make_shared<targeting_interfaces::srv::GetMultipulseWaveforms::Request>();
  request->targets = targets;
  request->target_waveforms = target_waveforms;

  /* Use promise and future to handle the async response. */
  std::promise<targeting_interfaces::srv::GetMultipulseWaveforms::Response::SharedPtr> promise;
  auto future = promise.get_future();

  auto response_callback = [&promise](rclcpp::Client<targeting_interfaces::srv::GetMultipulseWaveforms>::SharedFuture future_response) {
    try {
      auto response = future_response.get();
      promise.set_value(response);
    } catch (const std::exception &e) {
      promise.set_exception(std::make_exception_ptr(e));
    }
  };

  get_multipulse_waveforms_client->async_send_request(request, response_callback);

  /* Wait for the response. */
  auto future_status = future.wait_for(60s);
  if (future_status != std::future_status::ready) {
    throw std::runtime_error("Call to GetMultipulseWaveforms service timed out");
  }
  auto response = future.get();

  if (!response->success) {
    throw std::runtime_error("Call to GetMultipulseWaveforms service failed");
  }

  std::vector<event_interfaces::msg::WaveformsForCoilSet> approximated_waveforms(
      response->approximated_waveforms.begin(), response->approximated_waveforms.end());

  return {response->initial_voltages, approximated_waveforms};
}

std::pair<std::vector<double_t>, std::vector<bool>> TrialPerformerNode::get_target_voltages(
    const targeting_interfaces::msg::ElectricTarget &target) {

  auto request = std::make_shared<targeting_interfaces::srv::GetTargetVoltages::Request>();
  request->target = target;

  /* Use promise and future to handle the async response. */
  std::promise<targeting_interfaces::srv::GetTargetVoltages::Response::SharedPtr> promise;
  auto future = promise.get_future();

  auto response_callback = [&promise](rclcpp::Client<targeting_interfaces::srv::GetTargetVoltages>::SharedFuture future_response) {
    try {
      auto response = future_response.get();
      promise.set_value(response);
    } catch (const std::exception &e) {
      promise.set_exception(std::make_exception_ptr(e));
    }
  };

  targeting_client->async_send_request(request, response_callback);

  /* Wait for the response. */
  auto future_status = future.wait_for(10s);
  if (future_status != std::future_status::ready) {
    throw std::runtime_error("Call to GetTargetVoltages service timed out");
  }
  auto response = future.get();

  if (!response->success) {
    throw std::runtime_error("Call to GetTargetVoltages service failed");
  }

  return {response->voltages, response->reversed_polarities};
}

event_interfaces::msg::Waveform TrialPerformerNode::get_default_waveform(uint8_t channel) {
  auto request = std::make_shared<targeting_interfaces::srv::GetDefaultWaveform::Request>();
  request->channel = channel;

  /* Use promise and future to handle the async response. */
  std::promise<targeting_interfaces::srv::GetDefaultWaveform::Response::SharedPtr> promise;
  auto future = promise.get_future();

  auto response_callback = [&promise](rclcpp::Client<targeting_interfaces::srv::GetDefaultWaveform>::SharedFuture future_response) {
    try {
      auto response = future_response.get();
      promise.set_value(response);
    } catch (const std::exception &e) {
      promise.set_exception(std::make_exception_ptr(e));
    }
  };

  get_default_waveform_client->async_send_request(request, response_callback);

  /* Wait for the response. */
  auto future_status = future.wait_for(10s);
  if (future_status != std::future_status::ready) {
    throw std::runtime_error("Call to GetDefaultWaveform service timed out");
  }
  auto response = future.get();

  auto waveform = response->waveform;
  return waveform;
}

event_interfaces::msg::Waveform TrialPerformerNode::reverse_polarity(const event_interfaces::msg::Waveform &waveform) {
  auto request = std::make_shared<targeting_interfaces::srv::ReversePolarity::Request>();
  request->waveform = waveform;

  /* Use promise and future to handle the async response. */
  std::promise<targeting_interfaces::srv::ReversePolarity::Response::SharedPtr> promise;
  auto future = promise.get_future();

  auto response_callback = [&promise](rclcpp::Client<targeting_interfaces::srv::ReversePolarity>::SharedFuture future_response) {
    try {
      auto response = future_response.get();
      promise.set_value(response);
    } catch (const std::exception &e) {
      promise.set_exception(std::make_exception_ptr(e));
    }
  };

  reverse_polarity_client->async_send_request(request, response_callback);

  /* Wait for the response. */
  auto future_status = future.wait_for(10s);
  if (future_status != std::future_status::ready) {
    throw std::runtime_error("Call to ReversePolarity service timed out");
  }
  auto response = future.get();

  if (!response->success) {
    throw std::runtime_error("Call to ReversePolarity service failed");
  }

  return response->waveform;
}

void TrialPerformerNode::request_events(const std::vector<event_interfaces::msg::Pulse> &pulses, const std::vector<event_interfaces::msg::TriggerOut> &trigger_outs) {
  auto request = std::make_shared<mtms_device_interfaces::srv::RequestEvents::Request>();
  request->pulses = pulses;
  request->trigger_outs = trigger_outs;

  /* Use promise and future to handle the async response. */
  std::promise<mtms_device_interfaces::srv::RequestEvents::Response::SharedPtr> promise;
  auto future = promise.get_future();

  auto response_callback = [&promise](rclcpp::Client<mtms_device_interfaces::srv::RequestEvents>::SharedFuture future_response) {
    try {
      auto response = future_response.get();
      promise.set_value(response);
    } catch (const std::exception &e) {
      promise.set_exception(std::make_exception_ptr(e));
    }
  };

  request_events_client->async_send_request(request, response_callback);

  /* Wait for the response. */
  auto future_status = future.wait_for(10s);
  if (future_status != std::future_status::ready) {
    throw std::runtime_error("Call to RequestEvents service timed out");
  }
  auto response = future.get();

  if (!response->success) {
    throw std::runtime_error("Call to RequestEvents service failed");
  }
}

/* Action clients */

bool TrialPerformerNode::set_voltages(const std::vector<uint16_t> &voltages) {
  auto goal = mtms_device_interfaces::action::SetVoltages::Goal();
  goal.voltages = voltages;

  std::promise<bool> promise;
  auto future = promise.get_future();

  auto send_goal_options = rclcpp_action::Client<mtms_device_interfaces::action::SetVoltages>::SendGoalOptions();
  send_goal_options.result_callback =
      [&promise](const rclcpp_action::ClientGoalHandle<mtms_device_interfaces::action::SetVoltages>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          promise.set_value(result.result->success);
        } else {
          promise.set_value(false);
        }
      };

  auto future_goal_handle = set_voltages_client->async_send_goal(goal, send_goal_options);

  /* Wait for the goal handle. */
  if (future_goal_handle.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send 'set voltages' goal");
    return false;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "'Set voltages' goal was rejected by server");
    return false;
  }

  auto future_result = set_voltages_client->async_get_result(goal_handle);
  if (future_result.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get 'set voltages' result");
    return false;
  }

  bool success = future.get();
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set voltages");
  }

  return success;
}

std::shared_ptr<mep_interfaces::action::AnalyzeMep::Result> TrialPerformerNode::analyze_mep(const mep_interfaces::msg::MepConfiguration &mep_config, double time) {
  auto goal = mep_interfaces::action::AnalyzeMep::Goal();
  goal.mep_configuration = mep_config;
  goal.time = time;

  std::promise<std::shared_ptr<mep_interfaces::action::AnalyzeMep::Result>> promise;
  auto future = promise.get_future();

  auto send_goal_options = rclcpp_action::Client<mep_interfaces::action::AnalyzeMep>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, &promise](const rclcpp_action::ClientGoalHandle<mep_interfaces::action::AnalyzeMep>::WrappedResult &wrapped_result) {
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          promise.set_value(wrapped_result.result);
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to analyze MEP.");
          promise.set_value(nullptr);
        }
      };

  auto future_goal_handle = analyze_mep_client->async_send_goal(goal, send_goal_options);

  /* Wait for the goal handle. */
  if (future_goal_handle.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send MEP goal");
    return nullptr;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "MEP goal was rejected by server");
    return nullptr;
  }

  auto future_result = analyze_mep_client->async_get_result(goal_handle);
  if (future_result.wait_for(10s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get MEP result");
    return nullptr;
  }

  return future.get();
}

/* Goal handling */

rclcpp_action::GoalResponse TrialPerformerNode::handle_goal(
    [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
    [[maybe_unused]] std::shared_ptr<const experiment_interfaces::action::PerformTrial::Goal> goal) {
  RCLCPP_INFO(get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrialPerformerNode::handle_cancel(
    [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<experiment_interfaces::action::PerformTrial>> goal_handle) {
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrialPerformerNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<experiment_interfaces::action::PerformTrial>> goal_handle) {
  std::thread{std::bind(&TrialPerformerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void TrialPerformerNode::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<experiment_interfaces::action::PerformTrial>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal...");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<experiment_interfaces::action::PerformTrial::Result>();

  /* Log trial details. */
  log_trial(goal->trial);

  /* Check feasibility. */
  if (!check_trial_feasible()) {
    RCLCPP_WARN(this->get_logger(), "Trial not feasible.");
    result->success = false;
    goal_handle->succeed(result);
    return;
  }

  /* Perform the trial. */
  auto [success, trial_result] = perform_trial(goal->trial);

  /* Log trial success. */
  RCLCPP_INFO(this->get_logger(), "%s completed %s.",
    goal->trial.config.dry_run ? "Dry run" : "Trial",
    success ? "successfully" : "with errors");

  /* Set result and succeed the goal. */
  result->trial_result = trial_result;
  result->success = success;

  goal_handle->succeed(result);
}

std::pair<bool, experiment_interfaces::msg::TrialResult> TrialPerformerNode::perform_trial(const experiment_interfaces::msg::Trial &trial) {
  tic();

  bool success = true;
  experiment_interfaces::msg::TrialResult trial_result;
  double_t start_time;

  auto timing = trial.timing;
  auto config = trial.config;

  /* Always get initial voltages and waveforms to warm up the cache. */
  auto [desired_voltages, waveforms] = get_desired_voltages_and_waveforms(trial.targets, config.use_pulse_width_modulation_approximation);

  log_voltages(desired_voltages, "Desired voltages");

  /* Actually set voltages and perform pulses if not in dry run mode. */
  if (!config.dry_run) {
    success = success && set_voltages_if_needed(desired_voltages, config.voltage_tolerance_proportion_for_precharging);
    if (!success) {
      return {false, trial_result};
    }

    /* Determine the start time of the trial. */
    if (timing.allow_late) {
      /* If trial is allowed to be late, set the start time to the earliest possible time. The marginal
         is added because it takes a while to request the events from the mTMS device. */
      double_t earliest_start_time = get_current_time() + TRIAL_TIME_MARGINAL_S;
      start_time = std::max(earliest_start_time, timing.desired_start_time);

    } else {
      /* Otherwise attempt to start the trial at the desired start time. */
      start_time = timing.desired_start_time;
    }

    /* Create and send pulses and trigger outs. */
    auto [pulses, pulse_ids] = create_pulses(waveforms, trial, start_time);
    auto [trigger_outs, trigger_out_ids] = create_trigger_outs(trial.triggers, start_time);

    /* Request events. */
    request_events(pulses, trigger_outs);

    /* For troubleshooting purposes, log the time passed after requesting events. */
    toc("Time passed after requesting events");

    /* Fill in the earliest start time of the trial as the time right after requesting events. */
    trial_result.earliest_start_time = this->get_current_time();

    /* Analyze MEP if needed. */
    if (trial.analyze_mep) {
      auto mep_result = analyze_mep(trial.mep_config, start_time);
      if (mep_result == nullptr) {
        return {false, trial_result};
      }
      trial_result.mep = mep_result->mep;
      success = success && mep_result->success;

      if (!mep_result->success) {
        RCLCPP_WARN(this->get_logger(), "MEP analysis failed.");
      }
    }

    /* Wait for events to finish. */
    success = success && wait_for_events_to_finish(pulse_ids, trigger_out_ids);
  }

  /* Fill trial result. */
  trial_result.actual_start_time = start_time;

  auto voltages_after_trial = get_actual_voltages();
  log_voltages(voltages_after_trial, "Voltages after trial");

  if (config.recharge_after_trial) {
    RCLCPP_INFO(this->get_logger(), "Recharging...");
    success = success && set_voltages(desired_voltages);

    auto voltages_after_recharging = get_actual_voltages();
    log_voltages(voltages_after_recharging, "Voltages after recharging");
  }

  /* If trial was successful, create a marker in neuronavigation. */
  if (success) {
    create_marker();
  }

  return {success, trial_result};
}

std::pair<std::vector<uint16_t>, std::vector<event_interfaces::msg::WaveformsForCoilSet>> TrialPerformerNode::get_desired_voltages_and_waveforms(const std::vector<targeting_interfaces::msg::ElectricTarget> &targets, const bool use_pwm_approximation) {
  std::vector<event_interfaces::msg::WaveformsForCoilSet> target_waveforms;
  for (uint8_t i = 0; i < targets.size(); ++i) {
    event_interfaces::msg::WaveformsForCoilSet waveforms;
    for (uint8_t channel = 0; channel < NUM_OF_CHANNELS; ++channel) {
      waveforms.waveforms.push_back(get_default_waveform(channel));
    }
    target_waveforms.push_back(waveforms);
  }

  if (use_pwm_approximation) {
    return get_approximated_waveforms(targets, target_waveforms);
  } else {
    if (targets.size() != 1) {
      throw std::runtime_error("Non-approximated waveforms are only supported for one target.");
    }
    /* TODO: This should follow the same codepath as the approximated waveforms; most likely
         a good solution would be to use the same ROS service for both approximated and
        non-approximated waveforms. */
    return get_non_approximated_waveforms(targets[0], target_waveforms[0]);
  }
}

std::pair<std::vector<uint16_t>, std::vector<event_interfaces::msg::WaveformsForCoilSet>> TrialPerformerNode::get_non_approximated_waveforms(const targeting_interfaces::msg::ElectricTarget &target, const event_interfaces::msg::WaveformsForCoilSet &target_waveforms) {
  auto [desired_voltages_float, reversed_polarities] = get_target_voltages(target);

  /* Convert initial voltages from float to integer.

     TODO: Probably they should be integers in the first place. */
  std::vector<uint16_t> desired_voltages;
  for (auto voltage : desired_voltages_float) {
    desired_voltages.push_back(static_cast<uint16_t>(voltage));
  }

  /* Copy the target waveforms and reverse polarities if necessary. */
  auto target_waveforms_reversed = target_waveforms.waveforms;
  for (uint8_t channel = 0; channel < target_waveforms_reversed.size(); ++channel) {
    if (reversed_polarities[channel]) {
      target_waveforms_reversed[channel] = reverse_polarity(target_waveforms.waveforms[channel]);
    }
  }

  /* Create the approximated waveforms. */
  std::vector<event_interfaces::msg::WaveformsForCoilSet> approximated_waveforms(1);
  approximated_waveforms[0].waveforms = target_waveforms_reversed;

  return {desired_voltages, approximated_waveforms};
}

bool TrialPerformerNode::set_voltages_if_needed(const std::vector<uint16_t> &desired_voltages, float voltage_tolerance_proportion_for_precharging) {
  bool success = true;

  auto actual_voltages = get_actual_voltages();
  log_voltages(actual_voltages, "Actual voltages");

  bool precharge_needed = false;
  for (uint8_t i = 0; i < actual_voltages.size(); ++i) {
    auto relative_error = std::abs(actual_voltages[i] - desired_voltages[i]) / static_cast<float>(actual_voltages[i]);
    if (relative_error > voltage_tolerance_proportion_for_precharging &&
        std::abs(actual_voltages[i] - desired_voltages[i]) > ABSOLUTE_VOLTAGE_ERROR_THRESHOLD_FOR_PRECHARGING) {

      RCLCPP_INFO(this->get_logger(), "Voltage tolerance exceeded on channel %d (relative error: %.0f%%, absolute error: %d V). Precharging...",
        i,
        100 * relative_error,
        std::abs(actual_voltages[i] - desired_voltages[i]));

      precharge_needed = true;
      break;
    }
  }

  if (precharge_needed) {
    success = set_voltages(desired_voltages);
  }
  return success;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto trial_performer_node = std::make_shared<TrialPerformerNode>();

  /* Note: Use SingleThreadedExecutor here for several reasons:

    - Seems to offer better performance than MultiThreadedExecutor; the latter
      seems to have occasional hiccup-like delays, giving rise to spikes of ~10 ms
      in time between receiving the goal and requesting the trial-related events from
      the mTMS device.
    - Thread-safety won't need to be taken into account in the code.
    - When using MultiThreadedExecutor, an action call (e.g., SetVoltages) within the action server
      seems to occasionally crash with the following error:

      [trial_performer-1] terminate called after throwing an instance of 'std::runtime_error'
      [trial_performer-1]   what():  'data' is empty

      This error doesn't seem to occur with SingleThreadedExecutor. It might be a bug in the
      rclcpp_action library, but it's hard to say for sure. Here's a potentially relevant GitHub
      issue: https://github.com/ros2/rclcpp/issues/2423
    - For some reason, when using SingleThreadedExecutor in C++, service calls work and terminate
      properly when run within an action server - this is not the case when using Python, or when
      service calls are run within another service call or a subscriber callback in C++. Take advantage
      of this behavior for now.
  */
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(trial_performer_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
