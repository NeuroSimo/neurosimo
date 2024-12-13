#include "rclcpp/rclcpp.hpp"
#include "system_interfaces/srv/request_trigger.hpp"

#include "NiFpga_mTMS.h"
#include "fpga.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

void handle_request(
    [[maybe_unused]] const std::shared_ptr<system_interfaces::srv::RequestTrigger::Request> request,
    std::shared_ptr<system_interfaces::srv::RequestTrigger::Response> response) {

  if (!is_fpga_ok()) {
    RCLCPP_WARN(rclcpp::get_logger("request_trigger_handler"), "FPGA not in OK state while attempting to trigger events.");
    response->success = false;
    return;
  }

  NiFpga_MergeStatus(&status,
                     NiFpga_WriteBool(session,
                                      NiFpga_mTMS_ControlBool_Eventtrigger,
                                      true));

  RCLCPP_INFO(rclcpp::get_logger("request_trigger_handler"), "Events triggered.");
  response->success = true;
}

class RequestTriggerHandler : public rclcpp::Node {
public:
  RequestTriggerHandler() : Node("request_trigger_handler") {
    trigger_service_ = this->create_service<system_interfaces::srv::RequestTrigger>(
        "/mtms_device/trigger", &handle_request);
  }

private:
  rclcpp::Service<system_interfaces::srv::RequestTrigger>::SharedPtr trigger_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("request_trigger_handler"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<RequestTriggerHandler>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("request_trigger_handler"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("request_trigger_handler"), "Request trigger handler ready.");

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
