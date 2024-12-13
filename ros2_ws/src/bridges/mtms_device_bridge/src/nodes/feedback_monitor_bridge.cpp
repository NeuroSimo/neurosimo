#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "event_interfaces/msg/pulse_feedback.hpp"
#include "event_interfaces/msg/charge_feedback.hpp"
#include "event_interfaces/msg/discharge_feedback.hpp"
#include "event_interfaces/msg/trigger_out_feedback.hpp"

#include "fpga.h"
#include "NiFpga_mTMS.h"
#include "memory_utils.h"
#include "scheduling_utils.h"

const uint8_t CHANNEL_COUNT = 5;
const uint32_t CLOCK_FREQUENCY_HZ = 4e7;
const uint8_t FEEDBACK_MESSAGE_LENGTH = 11;
const uint8_t CHARGE_FEEDBACK_MESSAGE_LENGTH = 13;

using namespace std::chrono_literals;

NiFpga_mTMS_TargetToHostFifoU8 pulse_feedback_fifo = NiFpga_mTMS_TargetToHostFifoU8_TargettoHostPulsefeedbackFIFO;
NiFpga_mTMS_TargetToHostFifoU8 charge_feedback_fifo = NiFpga_mTMS_TargetToHostFifoU8_TargettoHostChargefeedbackFIFO;
NiFpga_mTMS_TargetToHostFifoU8 discharge_feedback_fifo = NiFpga_mTMS_TargetToHostFifoU8_TargettoHostDischargefeedbackFIFO;
NiFpga_mTMS_TargetToHostFifoU8 trigger_out_feedback_fifo = NiFpga_mTMS_TargetToHostFifoU8_TargettoHostTriggerOutfeedbackFIFO;

class FeedbackMonitorBridge : public rclcpp::Node {
public:
  FeedbackMonitorBridge()
      : Node("feedback_monitor_bridge") {
    pulse_feedback_publisher_ = this->create_publisher<event_interfaces::msg::PulseFeedback>(
        "/event/pulse_feedback", 10);
    charge_feedback_publisher_ = this->create_publisher<event_interfaces::msg::ChargeFeedback>(
        "/event/charge_feedback", 10);
    discharge_feedback_publisher_ = this->create_publisher<event_interfaces::msg::DischargeFeedback>(
        "/event/discharge_feedback", 10);
    trigger_out_feedback_publisher_ = this->create_publisher<event_interfaces::msg::TriggerOutFeedback>(
        "/event/trigger_out_feedback", 10);

    timer_ = this->create_wall_timer(20ms, std::bind(&FeedbackMonitorBridge::update_feedback_topics, this));
  }

private:
  /* HACK: There are essentially two ways to pass event feedback from FPGA, one is handled by read_fifo_and_publish
           function below (used by pulse, discharge, and trigger out events), and the other is handled by
           read_non_multiplexed_fifo_and_publish.

           They should be unified on the FPGA. The ideal way to do that might be drop the channel information and
           send feedback for all event types in the way it is done with charging at the moment. However, to
           confirm that, more SW components needs to be written that use the event feedback data. (Note that
           channel is already disregarded so that we are able to use a single ROS message type for all event
           feedback.) */

  void read_fifo_and_publish(std::string event_type,
                             NiFpga_mTMS_TargetToHostFifoU8 fifo,
                             std::map<uint8_t, std::vector<uint8_t>> map) {
    size_t elements_remaining = 0;
    NiFpga_Status read_status;
    std::vector<uint8_t> data(2);

    // Start by checking if there is enough data in the FIFO.
    read_status = NiFpga_ReadFifoU8(session, fifo, data.data(), 0, NiFpga_InfiniteTimeout,
                                    &elements_remaining);
    if (NiFpga_IsError(read_status)) {
      RCLCPP_ERROR(rclcpp::get_logger("feedback_monitor_bridge"), "Error reading data from fifo %d", read_status);
      return;
    }

    while (elements_remaining >= 2) {
      read_status = NiFpga_ReadFifoU8(session, fifo, data.data(), 2, NiFpga_InfiniteTimeout,
                                      &elements_remaining);

      if (NiFpga_IsError(read_status)) {
        RCLCPP_ERROR(rclcpp::get_logger("feedback_monitor_bridge"), "Error reading data from fifo %d",
                     read_status);
        return;
      }

      uint8_t channel = data[0];

      // If a key does not exist in map yet, add the key.
      if (map.find(channel) == map.end()) {
        std::vector<uint8_t> message_data;
        map.insert(std::make_pair(channel, message_data));
      }

      map[channel].push_back(data[1]);

      /* A whole message has been read. Unpack the message.

         The structure:

         Byte
         0     Event ID, lower byte
         1     Event ID, higher byte
         2     Error code
         3-10  Execution time in ticks */
      if (map[channel].size() >= FEEDBACK_MESSAGE_LENGTH) {
        uint16_t id = static_cast<uint16_t>(map[channel][0]) << 8 | map[channel][1];
        uint8_t error = map[channel][2];
        uint64_t execution_time_ticks =
            static_cast<uint64_t>(map[channel][3]) << 56 |
            static_cast<uint64_t>(map[channel][4]) << 48 |
            static_cast<uint64_t>(map[channel][5]) << 40 |
            static_cast<uint64_t>(map[channel][6]) << 32 |
            static_cast<uint64_t>(map[channel][7]) << 24 |
            static_cast<uint64_t>(map[channel][8]) << 16 |
            static_cast<uint64_t>(map[channel][9]) << 8 |
            static_cast<uint64_t>(map[channel][10]);

        double_t execution_time = (double)execution_time_ticks / CLOCK_FREQUENCY_HZ;

        map[channel].clear();

        publish_feedback(event_type, id, error, execution_time);
      }
    }
  }

  void read_and_publish_charge_feedback(NiFpga_mTMS_TargetToHostFifoU8 fifo) {

    size_t elements_remaining = 0;
    NiFpga_Status read_status;
    std::vector<uint8_t> data(CHARGE_FEEDBACK_MESSAGE_LENGTH);

    // Start by checking if there is enough data in the FIFO.
    read_status = NiFpga_ReadFifoU8(session, fifo, data.data(), 0, NiFpga_InfiniteTimeout,
                                    &elements_remaining);
    if (NiFpga_IsError(read_status)) {
      RCLCPP_ERROR(rclcpp::get_logger("feedback_monitor_bridge"), "Error reading data from fifo %d", read_status);
      return;
    }

    while (elements_remaining >= CHARGE_FEEDBACK_MESSAGE_LENGTH) {
      read_status = NiFpga_ReadFifoU8(session, fifo, data.data(), CHARGE_FEEDBACK_MESSAGE_LENGTH, NiFpga_InfiniteTimeout,
                                      &elements_remaining);

      if (NiFpga_IsError(read_status)) {
        RCLCPP_ERROR(rclcpp::get_logger("feedback_monitor_bridge"), "Error reading data from fifo %d",
                     read_status);
        return;
      }

      /* Unpack the feedback message.

         The structure:

         Byte
         0     Event ID, lower byte
         1     Event ID, higher byte
         2     Error code
         3     Charging time, lower byte
         4     Charging time, higher byte
         5-12  Execution time in ticks */

      uint16_t id = static_cast<uint16_t>(data[0]) << 8 | data[1];
      uint8_t error = data[2];
      uint64_t execution_time_ticks =
          static_cast<uint64_t>(data[3]) << 56 |
          static_cast<uint64_t>(data[4]) << 48 |
          static_cast<uint64_t>(data[5]) << 40 |
          static_cast<uint64_t>(data[6]) << 32 |
          static_cast<uint64_t>(data[7]) << 24 |
          static_cast<uint64_t>(data[8]) << 16 |
          static_cast<uint64_t>(data[9]) << 8 |
          static_cast<uint64_t>(data[10]);

      double_t execution_time = (double)execution_time_ticks / CLOCK_FREQUENCY_HZ;

      uint16_t charging_time_ms = static_cast<uint16_t>(data[11]) << 8 | data[12];
      double_t charging_time = (double_t)charging_time_ms / 1000;

      /* Create and publish charge feedback message. */

      event_interfaces::msg::ChargeFeedback feedback;
      feedback.id = id;
      feedback.error.value = error;
      feedback.charging_time = charging_time;
      feedback.execution_time = execution_time;

      charge_feedback_publisher_->publish(feedback);

      RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"),
                  "Publishing charge feedback: {id: %d, error: %d, charging time (ms): %d, execution time (s): %.1f}",
                  id,
                  error,
                  charging_time_ms,
                  execution_time);
    }
  }

  void publish_feedback(std::string event_type,
                        uint16_t id,
                        uint8_t error,
                        double_t execution_time) {

    /* HACK: Not that clean way to implement genericity for this function with regard to event types.
         For a better solution, reading FIFOs and publishing feedbacks would probably need to be decoupled
         first. */
    if (event_type == "Pulse") {
      event_interfaces::msg::PulseFeedback feedback;
      feedback.id = id;
      feedback.error.value = error;
      feedback.execution_time = execution_time;
      pulse_feedback_publisher_->publish(feedback);

    } else if (event_type == "Discharge") {
      event_interfaces::msg::DischargeFeedback feedback;
      feedback.id = id;
      feedback.error.value = error;
      feedback.execution_time = execution_time;
      discharge_feedback_publisher_->publish(feedback);

    } else if (event_type == "Trigger out") {
      event_interfaces::msg::TriggerOutFeedback feedback;
      feedback.id = id;
      feedback.error.value = error;
      feedback.execution_time = execution_time;
      trigger_out_feedback_publisher_->publish(feedback);
    }

    RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"),
                "Publishing data to %s feedback: {id: %d, error: %d, execution time: %.1f}",
                event_type.data(),
                id,
                error,
                execution_time);
  }

  void update_feedback_topics() {
    if (!is_fpga_ok()) {
      RCLCPP_WARN(rclcpp::get_logger("feedback_monitor_bridge"), "FPGA not in OK state while attempting to read event feedback");
      return;
    }
    read_fifo_and_publish("Pulse", pulse_feedback_fifo, pulse_data);
    read_fifo_and_publish("Discharge", discharge_feedback_fifo, discharge_data);
    read_fifo_and_publish("Trigger out", trigger_out_feedback_fifo, trigger_out_data);

    read_and_publish_charge_feedback(charge_feedback_fifo);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<event_interfaces::msg::PulseFeedback>::SharedPtr pulse_feedback_publisher_;
  rclcpp::Publisher<event_interfaces::msg::TriggerOutFeedback>::SharedPtr trigger_out_feedback_publisher_;
  rclcpp::Publisher<event_interfaces::msg::ChargeFeedback>::SharedPtr charge_feedback_publisher_;
  rclcpp::Publisher<event_interfaces::msg::DischargeFeedback>::SharedPtr discharge_feedback_publisher_;

  //channel index, data
  std::map<uint8_t, std::vector<uint8_t>> pulse_data = {};
  std::map<uint8_t, std::vector<uint8_t>> discharge_data = {};
  std::map<uint8_t, std::vector<uint8_t>> trigger_out_data = {};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_NORMAL_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<FeedbackMonitorBridge>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  RCLCPP_INFO(rclcpp::get_logger("feedback_monitor_bridge"), "Feedback monitor bridge ready.");

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
