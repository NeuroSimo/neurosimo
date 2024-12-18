#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "headers/scheduling_utils.h"
#include "headers/eeg_batcher.h"
#include "headers/memory_utils.h"

using namespace std::chrono_literals;
using namespace std::chrono;

EegBatcher::EegBatcher() : Node("eeg_batcher") {
  this->declare_parameter<int>("batch_size", 100);
  this->get_parameter("batch_size", batch_size);

  this->declare_parameter<int>("downsample_ratio", 10);
  this->get_parameter("downsample_ratio", downsample_ratio);

  auto eeg_data_subscription_callback = [this](
      const std::shared_ptr<eeg_interfaces::msg::Sample> message) -> void {
    if (send_counter % downsample_ratio == 0) {
      batch[batch_index++] = *message;
    }

    send_counter++;

    RCLCPP_INFO(rclcpp::get_logger("eeg_batcher"), "Received message index %d / %d", batch_index, batch_size);

    if (batch_index == batch_size) {
      auto batch_message = eeg_interfaces::msg::EegBatch();
      batch_message.batch = batch;
      batch_publisher->publish(batch_message);
      batch_index = 0;
      send_counter = 0;
    }
  };

  batch = std::vector<eeg_interfaces::msg::Sample>(batch_size);
  RCLCPP_INFO(this->get_logger(), "Batch size: %lu", batch.size());

  eeg_subscription = this->create_subscription<eeg_interfaces::msg::Sample>("/eeg/raw",
                                                                                   10,
                                                                                   eeg_data_subscription_callback);
  batch_publisher = this->create_publisher<eeg_interfaces::msg::EegBatch>("/eeg/batch_data", 10);
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);


#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("eeg_batcher"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<EegBatcher>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("eeg_batcher"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
