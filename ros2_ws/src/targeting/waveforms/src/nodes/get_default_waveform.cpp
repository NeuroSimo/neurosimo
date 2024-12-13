#include "scheduling_utils.h"
#include "memory_utils.h"

#include "rclcpp/rclcpp.hpp"
#include "targeting_interfaces/srv/get_default_waveform.hpp"
#include "event_interfaces/msg/waveform_phase.hpp"
#include "event_interfaces/msg/waveform_piece.hpp"

using namespace std;

const uint8_t N_CHANNELS = 5;

const uint16_t DEFAULT_WAVEFORM[][2] = {
  {event_interfaces::msg::WaveformPhase::RISING, 2400},
  {event_interfaces::msg::WaveformPhase::HOLD, 1200},
  {event_interfaces::msg::WaveformPhase::FALLING, 0}
};

const uint16_t LAST_WAVEFORM_PHASE_DURATION[N_CHANNELS] = {1480, 1480, 1564, 1564, 1776};

class GetDefaultWaveform : public rclcpp::Node {

public:
  GetDefaultWaveform() : Node("get_default_waveform") {

    auto service_callback = [this](
        const std::shared_ptr<targeting_interfaces::srv::GetDefaultWaveform::Request> request,
        std::shared_ptr<targeting_interfaces::srv::GetDefaultWaveform::Response> response) -> void {

      int8_t channel = request->channel;

      RCLCPP_INFO(rclcpp::get_logger("get_default_waveform"), "Request received: Default waveform for channel %d", channel);

      if (channel >= N_CHANNELS) {
        RCLCPP_WARN(rclcpp::get_logger("get_default_waveform"), "Invalid channel: %d.", channel);

        response->success = false;
        return;
      }

      event_interfaces::msg::WaveformPiece piece;
      for (uint8_t i = 0; i < std::size(DEFAULT_WAVEFORM); i++) {
        piece.waveform_phase.value = DEFAULT_WAVEFORM[i][0];

        if (i == std::size(DEFAULT_WAVEFORM) - 1) {
          piece.duration_in_ticks = LAST_WAVEFORM_PHASE_DURATION[channel];
        } else {
          piece.duration_in_ticks = DEFAULT_WAVEFORM[i][1];
        }

        response->waveform.pieces.push_back(piece);
      }

      response->success = true;

      RCLCPP_INFO(rclcpp::get_logger("get_default_waveform"), "Responded to request.");
    };

    get_default_waveform_service = this->create_service<targeting_interfaces::srv::GetDefaultWaveform>(
        "/waveforms/get_default", service_callback);
  }

private:
  rclcpp::Service<targeting_interfaces::srv::GetDefaultWaveform>::SharedPtr get_default_waveform_service;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("get_default_waveform"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_REALTIME_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<GetDefaultWaveform>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("get_default_waveform"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
