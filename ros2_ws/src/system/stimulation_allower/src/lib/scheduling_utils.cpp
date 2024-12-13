//
// Created by alqio on 23.8.2022.
// Adapted from https://github.com/ros-realtime/ros2-realtime-examples/blob/rolling/minimal_scheduling
// Apache 2.0 License. No major changes
//

#include "scheduling_utils.h"
#include "rclcpp/rclcpp.hpp"

void set_thread_scheduling(std::thread::native_handle_type thread, int policy, int sched_priority) {
  struct sched_param param;
  param.sched_priority = sched_priority;
  auto ret = pthread_setschedparam(thread, policy, &param);
  if (ret > 0) {
    RCLCPP_WARN(rclcpp::get_logger("stimulation_allower"), "Couldn't set scheduling priority and policy. Error code: %s",
                std::string(strerror(errno)).c_str());
  }
}
