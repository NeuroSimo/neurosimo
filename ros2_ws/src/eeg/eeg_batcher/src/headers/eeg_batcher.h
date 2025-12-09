//
// Created by alqio on 9/12/22.
//

#ifndef DATA_PROCESSOR_DATA_PROCESSOR_H
#define DATA_PROCESSOR_DATA_PROCESSOR_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/eeg_batch.hpp"

#include <string>
#include <fstream>

class EegBatcher : public rclcpp::Node {
public:
  EegBatcher();

private:
  std::vector<eeg_msgs::msg::Sample> batch;
  unsigned int batch_index;
  unsigned int batch_size;
  unsigned int downsample_ratio;
  unsigned int send_counter;

  rclcpp::Subscription<eeg_msgs::msg::Sample>::SharedPtr eeg_subscription;
  rclcpp::Publisher<eeg_msgs::msg::EegBatch>::SharedPtr batch_publisher;
};

#endif //DATA_PROCESSOR_DATA_PROCESSOR_H
