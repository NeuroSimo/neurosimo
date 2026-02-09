#ifndef FILESYSTEM_UTILS_FILESYSTEM_UTILS_H
#define FILESYSTEM_UTILS_FILESYSTEM_UTILS_H

#include <string>
#include <vector>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"

namespace filesystem_utils {

/**
 * @brief Changes the working directory to the specified path.
 * 
 * This function checks that the directory exists, resolves symlinks if necessary,
 * and changes the current working directory using chdir().
 * 
 * @param path The path to the directory to change to.
 * @param logger The ROS logger to use for error/info messages.
 * @return true if the directory change was successful, false otherwise.
 */
bool change_working_directory(const std::string& path, rclcpp::Logger logger);

} // namespace filesystem_utils

#endif // FILESYSTEM_UTILS_FILESYSTEM_UTILS_H

