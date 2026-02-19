#include "filesystem_utils/filesystem_utils.h"
#include <algorithm>
#include <unistd.h>

namespace filesystem_utils {

bool change_working_directory(const std::string& path, rclcpp::Logger logger) {
  /* Check that the directory exists and follow symlinks. */
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || 
      !std::filesystem::is_directory(path, ec)) {
    RCLCPP_ERROR(logger, "Directory does not exist: %s", path.c_str());
    if (ec) {
      RCLCPP_ERROR(logger, "Filesystem error: %s", ec.message().c_str());
    }
    return false;
  }
  
  /* If it's a symlink, resolve it and check the target. */
  std::filesystem::path resolved_path = path;
  if (std::filesystem::is_symlink(path, ec)) {
    resolved_path = std::filesystem::canonical(path, ec);
    if (ec) {
      RCLCPP_ERROR(logger, "Failed to resolve symlink %s: %s", 
        path.c_str(), ec.message().c_str());
      return false;
    }
    RCLCPP_INFO(logger, "Resolved symlink %s to %s", 
      path.c_str(), resolved_path.c_str());
    
    if (!std::filesystem::is_directory(resolved_path, ec)) {
      RCLCPP_ERROR(logger, "Symlink target is not a directory: %s -> %s",
        path.c_str(), resolved_path.c_str());
      return false;
    }
  }
  
  /* Change the working directory. */
  if (chdir(resolved_path.c_str()) != 0) {
    RCLCPP_ERROR(logger, "Failed to change working directory to: %s", 
      resolved_path.c_str());
    return false;
  }
  
  return true;
}

} // namespace filesystem_utils

