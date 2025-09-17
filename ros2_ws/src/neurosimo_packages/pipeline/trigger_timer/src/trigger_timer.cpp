#include <chrono>
#include <thread>

#include <LabJackM.h>
#include "LJM_Utilities.h"

#include "trigger_timer.h"

#include "realtime_utils/utils.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string TIMED_TRIGGER_SERVICE = "/pipeline/timed_trigger";
const std::string EEG_RAW_TOPIC = "/eeg/raw";

const double_t latency_measurement_interval = 0.1;

/* Note: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 5ms;

const char* tms_trigger_fio = "FIO5";
const char* latency_measurement_trigger_fio = "FIO4";

TriggerTimer::TriggerTimer() : Node("trigger_timer"), logger(rclcpp::get_logger("trigger_timer")) {
  /* Read ROS parameter: Maximum triggering error */
  auto triggering_tolerance_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  triggering_tolerance_descriptor.description = "The maximum triggering error (in seconds)";
  this->declare_parameter("triggering-tolerance", 0.0, triggering_tolerance_descriptor);
  this->get_parameter("triggering-tolerance", this->triggering_tolerance);

  /* Check that the triggering tolerance is non-negative. */
  if (this->triggering_tolerance < 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Triggering tolerance must be non-negative.");
    rclcpp::shutdown();
    return;
  }

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Triggering tolerance (ms): %.1f", 1000 * this->triggering_tolerance);
  RCLCPP_INFO(this->get_logger(), " ");

  /* Subscriber for mTMS device healthcheck. */
  this->mtms_device_healthcheck_subscriber = create_subscription<system_interfaces::msg::Healthcheck>(
    "/mtms_device/healthcheck",
    10,
    std::bind(&TriggerTimer::handle_mtms_device_healthcheck, this, _1));

  /* Subscriber for EEG raw data. */
  this->eeg_raw_subscriber = create_subscription<eeg_msgs::msg::Sample>(
    EEG_RAW_TOPIC,
    10,
    std::bind(&TriggerTimer::handle_eeg_raw, this, _1));

  /* Subscriber for timing error. */
  this->timing_error_subscriber = this->create_subscription<pipeline_interfaces::msg::TimingError>(
    "/pipeline/timing/error",
    10,
    std::bind(&TriggerTimer::handle_timing_error, this, _1));

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
    std::bind(&TriggerTimer::handle_session, this, _1),
    subscription_options);

  this->latency_measurement_trigger_subscriber = create_subscription<pipeline_interfaces::msg::TimedTrigger>(
    "/pipeline/latency_measurement_trigger",
    10,
    std::bind(&TriggerTimer::handle_latency_measurement_trigger, this, _1));

  /* Service for trigger request. */
  this->trigger_request_service = create_service<pipeline_interfaces::srv::RequestTimedTrigger>(
    TIMED_TRIGGER_SERVICE,
    std::bind(&TriggerTimer::handle_request_timed_trigger, this, _1, _2));

  /* Publisher for trigger info. */
  this->trigger_info_publisher = this->create_publisher<pipeline_interfaces::msg::TriggerInfo>(
    "/pipeline/trigger_info",
    10);

  /* Publisher for timing latency. */
  this->timing_latency_publisher = this->create_publisher<pipeline_interfaces::msg::TimingLatency>(
    "/pipeline/timing/latency",
    10);

  /* Publisher for decision info. */
  this->decision_info_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionInfo>(
    "/pipeline/decision_info",
    10);

  /* Attempt initial connection to LabJack. */
  attempt_labjack_connection();

  /* Set up a timer to attempt reconnection every second. */
  timer = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TriggerTimer::attempt_labjack_connection, this));
}

TriggerTimer::~TriggerTimer() {
  if (labjack_handle != -1) {
    CloseOrDie(labjack_handle);
  }
}

void TriggerTimer::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  bool state_changed = this->session_state.value != msg->state.value;
  this->session_state = msg->state;

  if (state_changed) {
    if (this->session_state.value == system_interfaces::msg::SessionState::STOPPED) {
      RCLCPP_INFO(this->get_logger(), "Session stopped, resetting.");
      this->current_latency = 0.0;
      this->latest_latency_measurement_time = 0.0;
    }
  }
}

void TriggerTimer::handle_latency_measurement_trigger(const std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> msg) {
  double_t trigger_time = msg->time;

  current_latency = trigger_time - latest_latency_measurement_time;

  /* Publish latency ROS message. */
  auto msg_ = pipeline_interfaces::msg::TimingLatency();
  msg_.latency = current_latency;

  this->timing_latency_publisher->publish(msg_);
}

void TriggerTimer::handle_timing_error(const std::shared_ptr<pipeline_interfaces::msg::TimingError> msg) {
  this->latest_timing_error = msg->error;
}

void TriggerTimer::attempt_labjack_connection() {
  if (labjack_handle == -1) {
    /* Attempt to open a connection to the LabJack device. */
    int err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &labjack_handle);
    if (err != LJME_NOERROR) {
      /* Ensure that handle is marked as invalid. */
      labjack_handle = -1;
      RCLCPP_WARN(logger, "Failed to connect to LabJack. Error code: %d", err);
    } else {
      PrintDeviceInfoFromHandle(labjack_handle);
      RCLCPP_INFO(logger, "Successfully connected to LabJack.");
    }
  }
}

bool TriggerTimer::safe_error_check(int err, const char* action) {
  if (err != LJME_NOERROR) {
    RCLCPP_ERROR(logger, "%s failed with error code: %d", action, err);
    if (err == LJME_RECONNECT_FAILED) {
      /* Mark as disconnected. */
      labjack_handle = -1;
      RCLCPP_WARN(logger, "LabJack connection lost. Will attempt to reconnect.");
    }
    return false;
  }
  return true;
}

void TriggerTimer::handle_mtms_device_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg) {
  this->mtms_device_available = msg->status.value == system_interfaces::msg::HealthcheckStatus::READY;
}

void TriggerTimer::handle_eeg_raw(const std::shared_ptr<eeg_msgs::msg::Sample> msg) {
  /* Ignore EEG sample if session has not started. */
  if (this->session_state.value != system_interfaces::msg::SessionState::STARTED) {
    return;
  }

  double_t current_time = msg->time;
  this->current_latency_corrected_time = current_time + this->current_latency;

  std::lock_guard<std::mutex> lock(queue_mutex);

  /* Trigger all events that are due. */
  while (!trigger_queue.empty() && trigger_queue.top() <= this->current_latency_corrected_time) {
    double_t scheduled_time = trigger_queue.top();
    double_t error = this->current_latency_corrected_time - scheduled_time;

    RCLCPP_INFO(logger, "Triggering at time: %.4f (current time: %.4f, error: %.4f)",
                scheduled_time, this->current_latency_corrected_time, error);
    bool success = trigger_labjack(tms_trigger_fio);
    if (!success) {
      RCLCPP_ERROR(logger, "Failed to trigger TMS trigger.");
      continue;
    }

    trigger_queue.pop();

    /* Publish trigger info. */
    auto trigger_info_msg = pipeline_interfaces::msg::TriggerInfo();
    trigger_info_msg.success = true;
    trigger_info_msg.scheduled_time = scheduled_time;
    trigger_info_msg.current_latency = this->current_latency;
    trigger_info_msg.latency_corrected_actual_time = this->current_latency_corrected_time;

    this->trigger_info_publisher->publish(trigger_info_msg);
  }

  /* Trigger latency measurement event at specific intervals. */
  if (current_time - latest_latency_measurement_time >= latency_measurement_interval) {
    bool success = trigger_labjack(latency_measurement_trigger_fio);
    if (!success) {
      RCLCPP_ERROR(logger, "Failed to trigger latency measurement trigger.");
      return;
    }
    latest_latency_measurement_time = current_time;
  }
}

void TriggerTimer::handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response) {

  double_t trigger_time = request->timed_trigger.time;

  /* Trigger is feasible if LabJack is connected and requested trigger time is less than current time - tolerance. */
  bool feasible = labjack_handle != -1 && trigger_time > this->current_latency_corrected_time - this->triggering_tolerance;

  /* Create and publish decision info. */
  auto msg = pipeline_interfaces::msg::DecisionInfo();
  msg.stimulate = true;
  msg.feasible = feasible;
  msg.decision_time = request->decision_time;
  msg.decider_latency = request->decider_latency;
  msg.preprocessor_latency = request->preprocessor_latency;

  /* In case of a positive stimulation decision, the total latency can only be calculated by Trigger Timer -
     it cannot be calculated by Decider due to the additional component (Trigger Timer) on the pathway. */
  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Time sample_time_rcl(request->system_time_for_sample);
  double_t total_latency = now.seconds() - sample_time_rcl.seconds();

  msg.total_latency = total_latency;
  this->decision_info_publisher->publish(msg);

  /* If not within acceptable range, log and return. */
  if (!feasible) {
    response->success = false;
    if (labjack_handle == -1) {
      RCLCPP_WARN(logger, "LabJack is not connected. Not scheduling a trigger.");
    } else {
      RCLCPP_WARN(logger,
        "Requested trigger time %.4f (s) is too late (current time: %.4f s, tolerance: %.1f ms). Not scheduling.",
        trigger_time, this->current_latency_corrected_time, 1000 * this->triggering_tolerance);
    }
    return;
  }

  /* Within acceptable range and LabJack is connected, schedule the trigger. */
  std::lock_guard<std::mutex> lock(queue_mutex);
  trigger_queue.push(trigger_time);

  RCLCPP_INFO(logger, "Scheduled trigger at time: %.4f (request accepted)", trigger_time);
  response->success = true;
}

bool TriggerTimer::trigger_labjack(const char* name) {
  if (labjack_handle == -1) {
    return false;
  }

  /* Set output port state to high. */
  int err = LJM_eWriteName(labjack_handle, name, 1);
  if (!safe_error_check(err, "Setting digital output on LabJack")) {
    return false;
  }

  /* Wait for one millisecond. */
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  /* Set output port state to low. */
  err = LJM_eWriteName(labjack_handle, name, 0);
  if (!safe_error_check(err, "Setting digital output on LabJack")) {
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("trigger_timer");

  realtime_utils::MemoryConfig mem_config;
  mem_config.enable_memory_optimization = true;
  mem_config.preallocate_size = 10 * 1024 * 1024; // 10 MB

  realtime_utils::SchedulingConfig sched_config;
  sched_config.enable_scheduling_optimization = true;
  sched_config.scheduling_policy = SCHED_RR;
  sched_config.priority_level = realtime_utils::PriorityLevel::HIGHEST_REALTIME;

  try {
    realtime_utils::initialize_scheduling(sched_config, logger);
    realtime_utils::initialize_memory(mem_config, logger);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Initialization failed: %s", e.what());
    return -1;
  }

  auto node = std::make_shared<TriggerTimer>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
