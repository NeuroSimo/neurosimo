#include "inotify_utils/inotify_watcher.h"

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <filesystem>

namespace inotify_utils {

InotifyWatcher::InotifyWatcher(rclcpp::Node* node, rclcpp::Logger logger, int poll_interval_ms)
  : node_(node), logger_(logger), watch_descriptor_(-1) {
  
  /* Initialize inotify. */
  inotify_descriptor_ = inotify_init();
  if (inotify_descriptor_ == -1) {
    RCLCPP_ERROR(logger_, "Error initializing inotify: %s", strerror(errno));
    throw std::runtime_error("Failed to initialize inotify");
  }

  /* Set the inotify descriptor to non-blocking. */
  int flags = fcntl(inotify_descriptor_, F_GETFL, 0);
  if (flags == -1) {
    close(inotify_descriptor_);
    RCLCPP_ERROR(logger_, "Error getting inotify flags: %s", strerror(errno));
    throw std::runtime_error("Failed to get inotify flags");
  }
  
  if (fcntl(inotify_descriptor_, F_SETFL, flags | O_NONBLOCK) == -1) {
    close(inotify_descriptor_);
    RCLCPP_ERROR(logger_, "Error setting inotify to non-blocking: %s", strerror(errno));
    throw std::runtime_error("Failed to set inotify to non-blocking");
  }

  /* Create a timer callback to poll inotify. */
  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms),
    std::bind(&InotifyWatcher::timer_callback, this));
}

InotifyWatcher::~InotifyWatcher() {
  remove_watch();
  close(inotify_descriptor_);
}

void InotifyWatcher::set_change_callback(std::function<void()> callback) {
  change_callback_ = callback;
}

bool InotifyWatcher::update_watch(const std::string& directory_path) {
  /* Remove the old watch if it exists. */
  remove_watch();

  /* Check if directory exists and is accessible. */
  std::error_code ec;
  if (!std::filesystem::exists(directory_path, ec) || 
      !std::filesystem::is_directory(directory_path, ec)) {
    RCLCPP_ERROR(logger_, "Directory does not exist or is not a directory: %s", directory_path.c_str());
    if (ec) {
      RCLCPP_ERROR(logger_, "Filesystem error: %s", ec.message().c_str());
    }
    return false;
  }

  /* Add a new watch for file creation, deletion, and moves (not modifications). */
  watch_descriptor_ = inotify_add_watch(inotify_descriptor_, directory_path.c_str(), 
                                         IN_CREATE | IN_DELETE | IN_MOVE);
  if (watch_descriptor_ == -1) {
    RCLCPP_ERROR(logger_, "Error adding watch for: %s, error: %s", 
                 directory_path.c_str(), strerror(errno));
    return false;
  }

  watched_directory_ = directory_path;
  RCLCPP_DEBUG(logger_, "Successfully watching directory: %s", directory_path.c_str());
  return true;
}

void InotifyWatcher::remove_watch() {
  if (watch_descriptor_ != -1) {
    inotify_rm_watch(inotify_descriptor_, watch_descriptor_);
    watch_descriptor_ = -1;
    watched_directory_.clear();
  }
}

void InotifyWatcher::timer_callback() {
  int length = read(inotify_descriptor_, inotify_buffer_, sizeof(inotify_buffer_));

  if (length < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      /* No events, return early. */
      return;
    } else {
      RCLCPP_ERROR(logger_, "Error reading inotify: %s", strerror(errno));
      return;
    }
  }

  /* Process all events in the buffer. */
  int i = 0;
  bool changes_detected = false;
  
  while (i < length) {
    struct inotify_event *event = (struct inotify_event *)&inotify_buffer_[i];
    
    if (event->len) {
      /* Check if this is a file creation, deletion, or move event. */
      if (event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) {
        changes_detected = true;
      }
    }
    
    i += sizeof(struct inotify_event) + event->len;
  }

  /* Call the callback if changes were detected. */
  if (changes_detected && change_callback_) {
    change_callback_();
  }
}

} // namespace inotify_utils

