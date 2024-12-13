#include "rclcpp/rclcpp.hpp"

#include "system_interfaces/srv/stop_session.hpp"

#include "NiFpga_mTMS.h"
#include "fpga.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

void stop_session([[maybe_unused]] const std::shared_ptr<system_interfaces::srv::StopSession::Request> request,
                     std::shared_ptr<system_interfaces::srv::StopSession::Response> response) {
  if (!is_fpga_ok()) {
    RCLCPP_WARN(rclcpp::get_logger("stop_session_handler"), "FPGA not in OK state during service call");
    response->success = false;
    return;
  }

  NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_mTMS_ControlBool_Stopsession, true));

  response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("stop_session_handler"), "Stopped session");
}

class StopSession : public rclcpp::Node {
public:
  StopSession()
      : Node("stop_session") {
    stop_session_service_ = this->create_service<system_interfaces::srv::StopSession>("/mtms_device/session/stop",
                                                                                          stop_session);
  }

private:
  rclcpp::Service<system_interfaces::srv::StopSession>::SharedPtr stop_session_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("stop_session_handler"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<StopSession>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("stop_session_handler"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("stop_session_handler"), "Stop session handler ready.");

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
