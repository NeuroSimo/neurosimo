#ifndef REALTIME_UTILS_UTILS_H
#define REALTIME_UTILS_UTILS_H

#include <sys/types.h>
#include <sys/mman.h>
#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <memory>
#include <thread>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"

namespace realtime_utils {

enum class PriorityLevel {
  NORMAL = 0,           // Non-realtime
  REALTIME = 98,        // Real-time priority
  HIGHEST_REALTIME = 99 // Highest real-time priority
};

struct MemoryConfig {
  bool enable_memory_optimization = true;
  size_t preallocate_size = 10 * 1024 * 1024; // 10 MB
};

struct SchedulingConfig {
  bool enable_scheduling_optimization = false;
  int scheduling_policy = SCHED_FIFO;
  PriorityLevel priority_level = PriorityLevel::NORMAL;
};

void initialize_memory(const MemoryConfig& config, rclcpp::Logger logger);
void initialize_scheduling(const SchedulingConfig& config, rclcpp::Logger logger);

} // namespace realtime_utils

#endif // REALTIME_UTILS_UTILS_H
