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

std::vector<std::string> list_files_with_extensions(
  const std::string& directory,
  const std::vector<std::string>& extensions,
  rclcpp::Logger logger) {
  
  std::vector<std::string> files;
  
  /* List all files with the given extensions in the directory. */
  std::error_code ec;
  try {
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
      if (ec) {
        RCLCPP_WARN(logger, "Error accessing directory %s: %s", 
          directory.c_str(), ec.message().c_str());
        return files;  // Return empty vector
      }
      
      std::error_code entry_ec;
      if (entry.is_regular_file(entry_ec) && !entry_ec) {
        std::string file_extension = entry.path().extension().string();
        
        /* Check if the file has one of the specified extensions. */
        for (const auto& ext : extensions) {
          if (file_extension == ext) {
            files.push_back(entry.path().stem().string());
            break;
          }
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_WARN(logger, "Filesystem error while listing files in %s: %s", 
      directory.c_str(), e.what());
    return files;  // Return whatever we managed to collect
  }
  
  /* Sort files alphabetically. */
  std::sort(files.begin(), files.end());
  
  return files;
}

std::vector<std::string> list_files_with_extension(
  const std::string& directory,
  const std::string& extension,
  rclcpp::Logger logger) {
  
  return list_files_with_extensions(directory, {extension}, logger);
}

} // namespace filesystem_utils

