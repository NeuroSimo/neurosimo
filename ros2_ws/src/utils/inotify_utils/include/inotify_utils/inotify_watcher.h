#ifndef INOTIFY_UTILS_INOTIFY_WATCHER_H
#define INOTIFY_UTILS_INOTIFY_WATCHER_H

#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace inotify_utils {

/**
 * @brief A class for watching a directory for file system changes using inotify.
 * 
 * This class provides a simplified interface to inotify functionality,
 * specifically designed for monitoring Python module directories in ROS2 nodes.
 * It watches for file creation, deletion, and moves (not modifications).
 */
class InotifyWatcher {
public:
  /**
   * @brief Construct a new Inotify Watcher object.
   * 
   * @param node Pointer to the ROS2 node for timer creation
   * @param logger ROS2 logger for error/info messages
   * @param poll_interval_ms Polling interval in milliseconds (default: 100ms)
   */
  InotifyWatcher(rclcpp::Node* node, rclcpp::Logger logger, int poll_interval_ms = 100);
  
  /**
   * @brief Destroy the Inotify Watcher object and clean up resources.
   */
  ~InotifyWatcher();

  /**
   * @brief Set the callback function to be called when directory changes are detected.
   * 
   * @param callback Function to call when files are created, deleted, or moved
   */
  void set_change_callback(std::function<void()> callback);

  /**
   * @brief Update the directory being watched.
   * 
   * @param directory_path Path to the directory to watch
   * @return true if watch was successfully updated
   * @return false if watch update failed
   */
  bool update_watch(const std::string& directory_path);

  /**
   * @brief Remove the current watch (if any).
   */
  void remove_watch();

private:
  /**
   * @brief Timer callback that polls inotify for events.
   */
  void timer_callback();

  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  int inotify_descriptor_;
  int watch_descriptor_;
  char inotify_buffer_[1024];
  
  std::function<void()> change_callback_;
  std::string watched_directory_;
};

} // namespace inotify_utils

#endif // INOTIFY_UTILS_INOTIFY_WATCHER_H

