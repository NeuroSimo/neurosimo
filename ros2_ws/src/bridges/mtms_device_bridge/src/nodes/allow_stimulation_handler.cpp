#include "rclcpp/rclcpp.hpp"

#include "mtms_device_interfaces/srv/allow_stimulation.hpp"

#include "NiFpga_mTMS.h"
#include "fpga.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

void allow_stimulation([[maybe_unused]] const std::shared_ptr<mtms_device_interfaces::srv::AllowStimulation::Request> request,
                        std::shared_ptr<mtms_device_interfaces::srv::AllowStimulation::Response> response) {
  if (!is_fpga_ok()) {
    RCLCPP_WARN(rclcpp::get_logger("allow_stimulation"), "FPGA not in OK state during service call");
    response->success = false;
    return;
  }

  bool allow_stimulation = request->allow_stimulation;

  NiFpga_MergeStatus(&status, NiFpga_WriteBool(session, NiFpga_mTMS_ControlBool_Allowstimulation, allow_stimulation));

  response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("allow_stimulation"), "%s stimulation", allow_stimulation ? "Allowing" : "Disallowing");
}

class AllowStimulation : public rclcpp::Node {
public:
  AllowStimulation() : Node("allow_stimulation") {
    allow_stimulation_service_ = this->create_service<mtms_device_interfaces::srv::AllowStimulation>("/mtms_device/allow_stimulation", allow_stimulation);
  }

private:
  rclcpp::Service<mtms_device_interfaces::srv::AllowStimulation>::SharedPtr allow_stimulation_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("allow_stimulation"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<AllowStimulation>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("allow_stimulation"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("allow_stimulation"), "Allow stimulation handler ready.");

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
