#include <unordered_map>

#include <boost/uuid/detail/md5.hpp>
#include <boost/algorithm/hex.hpp>

#include "rclcpp/serialization.hpp" // Make sure this include is at the top of your file
#include "rclcpp/serialized_message.hpp"

#include "rclcpp/rclcpp.hpp"

#include "targeting_interfaces/msg/electric_target.hpp"
#include "targeting_interfaces/srv/get_multipulse_waveforms.hpp"
#include "targeting_interfaces/srv/get_target_voltages.hpp"
#include "targeting_interfaces/srv/approximate_waveform.hpp"
#include "targeting_interfaces/srv/estimate_voltage_after_pulse.hpp"
#include "targeting_interfaces/srv/reverse_polarity.hpp"

#include "event_interfaces/msg/waveform_phase.hpp"
#include "event_interfaces/msg/waveform_piece.hpp"
#include "event_interfaces/msg/waveforms_for_coil_set.hpp"

#include "scheduling_utils.h"
#include "memory_utils.h"

using namespace std::placeholders;
using boost::uuids::detail::md5;


class GetMultipulseWaveforms : public rclcpp::Node {
public:
  GetMultipulseWaveforms();

private:
  void handle_get_multipulse_waveforms(
    const std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Request> request,
    std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Response> response);

  void handle_get_multipulse_waveforms_no_cache(
    const std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Request>& request,
    std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Response>& response);

  const std::shared_ptr<targeting_interfaces::srv::ApproximateWaveform::Response> approximate_waveform(
    const uint16_t actual_voltage,
    const uint16_t target_voltage,
    const event_interfaces::msg::Waveform& target_waveform,
    const uint8_t coil_number);

  const std::shared_ptr<targeting_interfaces::srv::ReversePolarity::Response> reverse_polarity(
    const event_interfaces::msg::Waveform& waveform);

  const std::shared_ptr<targeting_interfaces::srv::EstimateVoltageAfterPulse::Response> estimate_voltage_after_pulse(
    const uint16_t voltage_before,
    const event_interfaces::msg::Waveform& waveform,
    const uint8_t coil_number);

  const std::shared_ptr<targeting_interfaces::srv::GetTargetVoltages::Response> get_target_voltages(
    const targeting_interfaces::msg::ElectricTarget& target);

  rclcpp::Logger logger;
  rclcpp::CallbackGroup::SharedPtr callback_group;

  rclcpp::Service<targeting_interfaces::srv::GetMultipulseWaveforms>::SharedPtr get_multipulse_waveforms_service;

  rclcpp::Client<targeting_interfaces::srv::GetTargetVoltages>::SharedPtr get_target_voltages_client;
  rclcpp::Client<targeting_interfaces::srv::ApproximateWaveform>::SharedPtr approximate_waveform_client;
  rclcpp::Client<targeting_interfaces::srv::EstimateVoltageAfterPulse>::SharedPtr estimate_voltage_after_pulse_client;
  rclcpp::Client<targeting_interfaces::srv::ReversePolarity>::SharedPtr reverse_polarity_client;

  std::unordered_map<std::string, std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Response>> cache;
};
