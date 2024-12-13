#include <chrono>
#include <string>

#include "session_manager.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const milliseconds SESSION_PUBLISHING_INTERVAL = 1ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 2ms;


SessionManager::SessionManager() : Node("session_manager") {
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /* Subscriber for mTMS device healthcheck. */
  this->mtms_device_healthcheck_subscriber = create_subscription<system_interfaces::msg::Healthcheck>(
    "/mtms_device/healthcheck",
    10,
    std::bind(&SessionManager::handle_mtms_device_healthcheck, this, _1));

  /* QOS for session */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  /* Subscriber for mTMS device session. */
  this->session_subscriber = create_subscription<system_interfaces::msg::Session>(
    "/mtms_device/session",
    qos_session,
    std::bind(&SessionManager::handle_mtms_device_session, this, _1));

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

  /* Service client for starting mTMS device session. */
  start_mtms_session_client = this->create_client<system_interfaces::srv::StartSession>(
    "/mtms_device/session/start",
    rclcpp::QoS(10),
    callback_group);

  /* Service client for stopping mTMS device session. */
  stop_mtms_session_client = this->create_client<system_interfaces::srv::StopSession>(
    "/mtms_device/session/stop",
    rclcpp::QoS(10),
    callback_group);

  /* Timer for keeping track of internal time and publishing session. */
  timer = this->create_wall_timer(SESSION_PUBLISHING_INTERVAL, std::bind(&SessionManager::publish_session, this));
}

void SessionManager::handle_mtms_device_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg) {
  this->mtms_device_available = msg->status.value == system_interfaces::msg::HealthcheckStatus::READY;
}

void SessionManager::handle_mtms_device_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  if (!this->mtms_device_available) {
    return;
  }
  system_interfaces::msg::Session msg_ = *msg;
  this->session_publisher->publish(msg_);
}

void SessionManager::handle_start_session(
      [[maybe_unused]] const std::shared_ptr<system_interfaces::srv::StartSession::Request> request,
      std::shared_ptr<system_interfaces::srv::StartSession::Response> response) {

  bool success = false;
  if (this->mtms_device_available) {
    success = start_mtms_session();
  } else {
    success = start_session();
  }

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Session started.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start session.");
  }
  response->success = success;
}

void SessionManager::handle_stop_session(
      [[maybe_unused]] const std::shared_ptr<system_interfaces::srv::StopSession::Request> request,
      std::shared_ptr<system_interfaces::srv::StopSession::Response> response) {

  bool success = false;
  if (this->mtms_device_available) {
    success = stop_mtms_session();
  } else {
    success = stop_session();
  }

  if (success) {
    RCLCPP_INFO(this->get_logger(), "Session stopped.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to stop session.");
  }
  response->success = success;
}

/* mTMS device session */

bool SessionManager::start_mtms_session() {
  auto request = std::make_shared<system_interfaces::srv::StartSession::Request>();

  auto future_result = start_mtms_session_client->async_send_request(request);

  if (future_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    return future_result.get()->success;

  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call /mtms_device/session/start or response timed out");
    return false;
  }
}

bool SessionManager::stop_mtms_session() {
  auto request = std::make_shared<system_interfaces::srv::StopSession::Request>();

  auto future_result = stop_mtms_session_client->async_send_request(request);

  if (future_result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    return future_result.get()->success;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call /mtms_device/session/stop or response timed out");
    return false;
  }
}

/* Internal session */

bool SessionManager::start_session() {
  session_state = system_interfaces::msg::SessionState::STARTED;
  session_start_time = std::chrono::steady_clock::now();

  return true;
}

bool SessionManager::stop_session() {
  session_state = system_interfaces::msg::SessionState::STOPPED;
  return true;
}

void SessionManager::publish_session() {
  if (this->mtms_device_available) {
    return;
  }

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

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("session_manager"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_REALTIME_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<SessionManager>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("session_manager"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
