#include "rclcpp/rclcpp.hpp"

#include "mtms_device_interfaces/srv/send_settings.hpp"

#include "NiFpga_mTMS.h"
#include "fpga.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

const uint32_t CLOCK_FREQUENCY_HZ = 4e7;

void send_settings(const std::shared_ptr<mtms_device_interfaces::srv::SendSettings::Request> request,
                   std::shared_ptr<mtms_device_interfaces::srv::SendSettings::Response> response) {
  /* Log new settings. */
  auto settings = request->settings;

  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "Received new settings:");
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "  maximum number of pulse pieces: %d", settings.maximum_number_of_pulse_pieces);
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "  maximum rising-falling difference (ticks): %d", settings.maximum_rising_falling_difference_ticks);
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "  maximum pulse duration (ticks): %d", settings.maximum_pulse_duration_ticks);
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "  maximum pulses per unit time, pulses: %d", settings.pulses_in_maximum_pulses_per_time);
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "  maximum pulses per unit time, unit time: %d", settings.time_in_maximum_pulses_per_time_ms);

  /* Check if FPGA is OK. */
  if (!is_fpga_ok()) {
    RCLCPP_WARN(rclcpp::get_logger("settings_handler"), "Tried to change settings but FPGA is not in OK state");
    return;
  }

  /* Write settings to registers. */
  NiFpga_MergeStatus(&status,
                     NiFpga_WriteU8(session,
                                    NiFpga_mTMS_ControlU8_Maximumnumberofpulsepieces,
                                    settings.maximum_number_of_pulse_pieces));

  NiFpga_MergeStatus(&status,
                     NiFpga_WriteU16(session,
                                     NiFpga_mTMS_ControlU16_Maximumrisingfallingdifferenceticks,
                                     settings.maximum_rising_falling_difference_ticks));

  NiFpga_MergeStatus(&status,
                     NiFpga_WriteU16(session,
                                     NiFpga_mTMS_ControlU16_Maximumpulsedurationticks,
                                     settings.maximum_pulse_duration_ticks));

  uint16_t time_ms = settings.time_in_maximum_pulses_per_time_ms;
  uint32_t time_ticks = CLOCK_FREQUENCY_HZ / 1000 * (uint32_t)time_ms;

  NiFpga_MergeStatus(&status,
                     NiFpga_WriteU32(session,
                                     NiFpga_mTMS_ControlU32_Maximumpulsespertimetimeticks,
                                     time_ticks));

  NiFpga_MergeStatus(&status,
                     NiFpga_WriteU8(session,
                                    NiFpga_mTMS_ControlU8_Maximumpulsespertimepulses,
                                    settings.pulses_in_maximum_pulses_per_time));

  response->success = true;
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "Settings changed successfully.");
}

class SettingsHandler : public rclcpp::Node {
public:
  SettingsHandler()
      : Node("settings_handler") {
    send_settings_service_ = this->create_service<mtms_device_interfaces::srv::SendSettings>("/mtms_device/send_settings",
                                                                                      send_settings);
  }

private:
  rclcpp::Service<mtms_device_interfaces::srv::SendSettings>::SharedPtr send_settings_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<SettingsHandler>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("settings_handler"), "Settings handler ready.");

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
