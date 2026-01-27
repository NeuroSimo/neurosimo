#include <chrono>
#include <thread>

#include "trigger_timer.h"

#include "realtime_utils/utils.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string TIMED_TRIGGER_SERVICE = "/pipeline/timed_trigger";
const std::string EEG_RAW_TOPIC = "/eeg/raw";

const double_t latency_measurement_interval = 0.1;

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

  /* Read ROS parameter: LabJack simulation mode */
  auto simulate_labjack_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  simulate_labjack_descriptor.description = "Simulate LabJack device when hardware is not available";
  simulate_labjack_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  this->declare_parameter("simulate-labjack", false, simulate_labjack_descriptor);
  this->get_parameter("simulate-labjack", this->simulate_labjack);

  /* Log the configuration. */
  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Configuration:");
  RCLCPP_INFO(this->get_logger(), "  Triggering tolerance (ms): %.1f", 1000 * this->triggering_tolerance);
  RCLCPP_INFO(this->get_logger(), "  LabJack simulation: %s", this->simulate_labjack ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), " ");

  /* Subscriber for EEG raw data. */
  this->eeg_raw_subscriber = create_subscription<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    10,
    std::bind(&TriggerTimer::handle_eeg_raw, this, _1));

  /* Subscriber for timing error. */
  this->timing_error_subscriber = this->create_subscription<pipeline_interfaces::msg::TimingError>(
    "/pipeline/timing/error",
    10,
    std::bind(&TriggerTimer::handle_timing_error, this, _1));

  /* Service for trigger request. */
  this->trigger_request_service = create_service<pipeline_interfaces::srv::RequestTimedTrigger>(
    TIMED_TRIGGER_SERVICE,
    std::bind(&TriggerTimer::handle_request_timed_trigger, this, _1, _2));

  /* Publisher for decision trace. */
  this->decision_trace_publisher = this->create_publisher<pipeline_interfaces::msg::DecisionTrace>(
    "/pipeline/decision_trace",
    10);

  /* Publisher for timing latency. */
  this->pipeline_latency_publisher = this->create_publisher<pipeline_interfaces::msg::PipelineLatency>(
    "/pipeline/latency",
    10);

  /* Publisher for pulse events. */
  this->pulse_event_publisher = this->create_publisher<std_msgs::msg::Empty>(
    "/pipeline/pulse_events",
    100);

  /* Initialize LabJack manager. */
  labjack_manager = std::make_unique<LabJackManager>(this->get_logger(), this->simulate_labjack);
  labjack_manager->start();

  /* Set up a timer to signal connection attempts every second. */
  timer = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&TriggerTimer::attempt_labjack_connection, this));
}

TriggerTimer::~TriggerTimer() {
  if (labjack_manager) {
    labjack_manager->stop();
  }
}

void TriggerTimer::handle_timing_error(const std::shared_ptr<pipeline_interfaces::msg::TimingError> msg) {
  this->latest_timing_error = msg->error;
}

void TriggerTimer::attempt_labjack_connection() {
  if (labjack_manager) {
    labjack_manager->request_connection_attempt();
  }
}

void TriggerTimer::trigger_pulses_until_time(double_t sample_time) {
  double_t latency_corrected_time = sample_time + this->current_latency;

  /* Trigger all events until the given time. */
  while (!trigger_queue.empty() && trigger_queue.top()->timed_trigger.time <= latency_corrected_time) {
    auto request = trigger_queue.top();
    double_t scheduled_time = request->timed_trigger.time;
    double_t error = latency_corrected_time - scheduled_time;

    RCLCPP_INFO(logger, "Triggering at scheduled time: %.4f (current time: %.4f, error: %.4f)",
                scheduled_time, latency_corrected_time, error);

    /* Capture timing before hardware trigger. */
    auto firing_time = std::chrono::high_resolution_clock::now();
    uint64_t system_time_hardware_fired = std::chrono::duration_cast<std::chrono::nanoseconds>(
      firing_time.time_since_epoch()).count();

    if (!labjack_manager || !labjack_manager->trigger_output(tms_trigger_fio)) {
      RCLCPP_ERROR(logger, "Failed to trigger TMS trigger.");
    }

    trigger_queue.pop();

    /* Publish second DecisionTrace (fired). */
    auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
    decision_trace.session_id = request->session_id;
    decision_trace.decision_id = request->decision_id;
    decision_trace.status = pipeline_interfaces::msg::DecisionTrace::STATUS_FIRED;
    decision_trace.system_time_hardware_fired = system_time_hardware_fired;
    decision_trace.sample_time_at_firing = sample_time;
    decision_trace.latency_corrected_time_at_firing = latency_corrected_time;
    decision_trace.pipeline_latency_at_firing = this->current_latency;

    this->decision_trace_publisher->publish(decision_trace);

    /* Publish pulse event for experiment coordinator. */
    auto pulse_event_msg = std_msgs::msg::Empty();
    this->pulse_event_publisher->publish(pulse_event_msg);
  }
}

void TriggerTimer::measure_latency(bool latency_trigger, double_t sample_time) {
  /* Update current latency if latency trigger is present. */
  if (latency_trigger) {
    this->current_latency = sample_time - this->last_latency_measurement_time;

    /* Publish pipeline latency ROS message. */
    auto msg = pipeline_interfaces::msg::PipelineLatency();
    msg.latency = this->current_latency;
  
    this->pipeline_latency_publisher->publish(msg);  
  }

  /* Trigger latency measurement at specific intervals. */
  double_t time_since_last_latency_measurement = sample_time - this->last_latency_measurement_time;
  if (time_since_last_latency_measurement < latency_measurement_interval) {
    return;
  }

  this->last_latency_measurement_time = sample_time;

  if (!labjack_manager || !labjack_manager->trigger_output(latency_measurement_trigger_fio)) {
    RCLCPP_ERROR(logger, "Failed to trigger latency measurement trigger.");
    return;
  }
}

void TriggerTimer::handle_eeg_raw(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  if (msg->is_session_start) {
    RCLCPP_INFO(this->get_logger(), "Session started.");
    this->current_latency = 0.0;
    this->last_latency_measurement_time = 0.0;
  }
    
  double_t sample_time = msg->time;

  std::lock_guard<std::mutex> lock(queue_mutex);
    
  trigger_pulses_until_time(sample_time);
  measure_latency(msg->latency_trigger, sample_time);
}

void TriggerTimer::handle_request_timed_trigger(
    const std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::RequestTimedTrigger::Response> response) {

  /* Capture timing at request receipt. */
  auto start_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_received = std::chrono::duration_cast<std::chrono::nanoseconds>(
    start_time.time_since_epoch()).count();

  double_t trigger_time = request->timed_trigger.time;

  bool is_labjack_connected = labjack_manager && labjack_manager->is_connected();
  bool accepted = is_labjack_connected;

  if (!accepted) {
    RCLCPP_WARN(logger, "LabJack is not connected, rejecting trigger at time: %.4f (s).", trigger_time);
    response->success = false;
  } else {
    std::lock_guard<std::mutex> lock(queue_mutex);
    trigger_queue.push(request);

    RCLCPP_INFO(logger, "Scheduled trigger at time: %.4f (s).", trigger_time);
    response->success = true;
  }

  /* Capture timing at request finish. */
  auto end_time = std::chrono::high_resolution_clock::now();
  uint64_t system_time_trigger_timer_finished = std::chrono::duration_cast<std::chrono::nanoseconds>(
    end_time.time_since_epoch()).count();

  /* Publish first DecisionTrace (scheduled or rejected). */
  auto decision_trace = pipeline_interfaces::msg::DecisionTrace();
  decision_trace.session_id = request->session_id;
  decision_trace.decision_id = request->decision_id;
  decision_trace.status = accepted ? pipeline_interfaces::msg::DecisionTrace::STATUS_SCHEDULED
                                   : pipeline_interfaces::msg::DecisionTrace::STATUS_REJECTED;
  decision_trace.system_time_trigger_timer_received = system_time_trigger_timer_received;
  decision_trace.system_time_trigger_timer_finished = system_time_trigger_timer_finished;

  this->decision_trace_publisher->publish(decision_trace);
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
