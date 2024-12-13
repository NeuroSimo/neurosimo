#include "rclcpp/rclcpp.hpp"

#include "mtms_device_interfaces/srv/start_device.hpp"

#include "NiFpga_mTMS.h"
#include "fpga.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

void start_device([[maybe_unused]] const std::shared_ptr<mtms_device_interfaces::srv::StartDevice::Request> request,
                  std::shared_ptr<mtms_device_interfaces::srv::StartDevice::Response> response) {
  if (!is_fpga_ok()) {
    RCLCPP_WARN(rclcpp::get_logger("start_device_handler"), "FPGA not in OK state during service call");
    response->success = false;
    return;
  }

  NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_mTMS_ControlBool_Startdevice, true));

  response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("start_device_handler"), "Started device");
}

class StartDevice : public rclcpp::Node {
public:
  StartDevice()
      : Node("start_device") {
    start_device_service_ = this->create_service<mtms_device_interfaces::srv::StartDevice>("/mtms_device/start_device", start_device);
  }

private:
  rclcpp::Service<mtms_device_interfaces::srv::StartDevice>::SharedPtr start_device_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("start_device_handler"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<StartDevice>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("start_device_handler"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("start_device_handler"), "Start device handler ready.");

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
