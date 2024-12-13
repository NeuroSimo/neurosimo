#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "system_interfaces/msg/session_state.hpp"
#include "system_interfaces/msg/session.hpp"

#include "fpga.h"
#include "NiFpga_mTMS.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

using namespace std::chrono;
using namespace std::chrono_literals;

const uint32_t CLOCK_FREQUENCY_HZ = 4e7;

const milliseconds SESSION_PUBLISHING_INTERVAL = 1ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 2ms;

NiFpga_mTMS_IndicatorU8 session_state_indicator = NiFpga_mTMS_IndicatorU8_Sessionstate;
NiFpga_mTMS_IndicatorU64 time_indicator = NiFpga_mTMS_IndicatorU64_Time;

class SessionBridge : public rclcpp::Node {
public:
  SessionBridge() : Node("session_bridge") {

    const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        .deadline(DEADLINE_NS)
        .lifespan(DEADLINE_NS);

    session_publisher = this->create_publisher<system_interfaces::msg::Session>(
      "/mtms_device/session",
      qos);

    timer = this->create_wall_timer(SESSION_PUBLISHING_INTERVAL, std::bind(&SessionBridge::publish_session, this));
  }

private:
  void publish_session() {
    if (!is_fpga_ok()) {
      RCLCPP_WARN(rclcpp::get_logger("session_bridge"), "FPGA not in OK state while attempting to read session");
      return;
    }

    auto session_msg = system_interfaces::msg::Session();

    NiFpga_MergeStatus(&status,
                        NiFpga_ReadU8(
                            session,
                            session_state_indicator,
                            &session_msg.state.value
                        ));

    uint64_t time;
    NiFpga_MergeStatus(&status,
                        NiFpga_ReadU64(
                            session,
                            time_indicator,
                            &time
                        ));

    session_msg.time = (double)time / CLOCK_FREQUENCY_HZ;

    session_publisher->publish(session_msg);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<system_interfaces::msg::Session>::SharedPtr session_publisher;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("session_bridge"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<SessionBridge>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("session_bridge"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("session_bridge"), "Session bridge ready.");

  init_fpga();

  auto timer = node->create_wall_timer(
      std::chrono::milliseconds(FPGA_OK_CHECK_INTERVAL_MS),
      [&]() {
          if (!is_fpga_ok()) {
              close_fpga();
              init_fpga();
          }
      }
  );
  rclcpp::spin(node);

  close_fpga();
  rclcpp::shutdown();
}
