#include <chrono>
#include <string>

#include "session_manager.h"

#include "realtime_utils/utils.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const milliseconds SESSION_PUBLISHING_INTERVAL = 1ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 2ms;


SessionManager::SessionManager() : Node("session_manager") {
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /* QOS for session */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  /* Publisher for session. */
  this->session_publisher = create_publisher<system_interfaces::msg::Session>(
    "/system/session",
    qos_session);

  /* Service for starting session. */
  this->start_session_service = this->create_service<system_interfaces::srv::StartSession>(
    "/system/session/start",
    std::bind(&SessionManager::handle_start_session, this, _1, _2),
    rclcpp::QoS(10),
    callback_group);

  /* Service for stopping session. */
  this->stop_session_service = this->create_service<system_interfaces::srv::StopSession>(
    "/system/session/stop",
    std::bind(&SessionManager::handle_stop_session, this, _1, _2),
    rclcpp::QoS(10),
    callback_group);

  /* Timer for keeping track of internal time and publishing session. */
  timer = this->create_wall_timer(SESSION_PUBLISHING_INTERVAL, std::bind(&SessionManager::publish_session, this));
}

void SessionManager::handle_start_session(
      [[maybe_unused]] const std::shared_ptr<system_interfaces::srv::StartSession::Request> request,
      std::shared_ptr<system_interfaces::srv::StartSession::Response> response) {

  this->session_state = system_interfaces::msg::SessionState::STARTED;
  this->session_start_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(this->get_logger(), "Session started.");

  response->success = true;
}

void SessionManager::handle_stop_session(
      [[maybe_unused]] const std::shared_ptr<system_interfaces::srv::StopSession::Request> request,
      std::shared_ptr<system_interfaces::srv::StopSession::Response> response) {

  this->session_state = system_interfaces::msg::SessionState::STOPPED;
  RCLCPP_INFO(this->get_logger(), "Session stopped.");

  response->success = true;
}

void SessionManager::publish_session() {
  /* Update time. */
  double_t time = 0.0;

  if (session_state == system_interfaces::msg::SessionState::STARTED) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - session_start_time);
    time = elapsed.count() / 1000.0;
  }

  /* Update session state. */
  auto msg = system_interfaces::msg::Session();
  msg.state.value = session_state;
  msg.time = time;
  this->session_publisher->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("session_manager");

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

  auto node = std::make_shared<SessionManager>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
