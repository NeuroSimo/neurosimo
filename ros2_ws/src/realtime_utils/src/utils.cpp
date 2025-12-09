#include "realtime_utils/utils.h"

namespace realtime_utils {

void initialize_memory(const MemoryConfig& config, rclcpp::Logger logger) {
  if (!config.enable_memory_optimization) return;

  // Lock all current and future pages
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    throw std::runtime_error("mlockall failed. Error: " + std::string(strerror(errno)));
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    munlockall();
    throw std::runtime_error("mallopt for trim threshold failed. Error: " + std::string(strerror(errno)));
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    throw std::runtime_error("mallopt for mmap failed. Error: " + std::string(strerror(errno)));
  }

  // Preallocate memory
  void* buf = nullptr;
  const size_t pg_sz = sysconf(_SC_PAGESIZE);
  int res = posix_memalign(&buf, static_cast<size_t>(pg_sz), config.preallocate_size);
  if (res != 0) {
    throw std::runtime_error("Failed to reserve memory. Error: " + std::string(strerror(errno)));
  }
  free(buf);
}

void initialize_scheduling(const SchedulingConfig& config, rclcpp::Logger logger) {
  if (!config.enable_scheduling_optimization) return;

  pthread_t thread = pthread_self();
  struct sched_param param;

  // Map PriorityLevel to actual priority values
  switch (config.priority_level) {
    case PriorityLevel::NORMAL:
      return; // Skip scheduling as it's not real-time
    case PriorityLevel::REALTIME:
      RCLCPP_INFO(logger, "Setting real-time priority");
      param.sched_priority = static_cast<int>(PriorityLevel::REALTIME);
      break;
    case PriorityLevel::HIGHEST_REALTIME:
      RCLCPP_INFO(logger, "Setting highest real-time priority");
      param.sched_priority = static_cast<int>(PriorityLevel::HIGHEST_REALTIME);
      break;
  }

  int ret = pthread_setschedparam(thread, config.scheduling_policy, &param);
  if (ret != 0) {
    throw std::runtime_error("Couldn't set scheduling priority and policy. Error: " + std::string(strerror(errno)));
  }
}

} // namespace realtime_utils
