//
// Created by alqio on 5.9.2022.
//

#include "headers/scheduling_utils.h"
#include "rclcpp/rclcpp.hpp"

void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority) {
  struct sched_param param;
  param.sched_priority = sched_priority;
  auto ret = pthread_setschedparam(thread, policy, &param);
  if (ret > 0) {
    RCLCPP_WARN(rclcpp::get_logger("data_batcher"), "Couldn't set scheduling priority and policy. Error code: %s",
                std::string(strerror(errno)).c_str());
  }
}
