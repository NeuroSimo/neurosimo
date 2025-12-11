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

/**
 * @brief Lists all files with specified extension(s) in the given directory.
 * 
 * This function lists all regular files in the specified directory that have
 * one of the specified extensions. Returns only the stem (filename without extension).
 * The results are sorted alphabetically.
 * 
 * @param directory The directory to list files from.
 * @param extensions Vector of extensions to filter by (e.g., {".py", ".yaml"}).
 *                   Each extension should include the leading dot.
 * @param logger The ROS logger to use for warning messages.
 * @return A sorted vector of filenames (without extensions) found in the directory.
 */
std::vector<std::string> list_files_with_extensions(
  const std::string& directory,
  const std::vector<std::string>& extensions,
  rclcpp::Logger logger);

/**
 * @brief Lists all files with a single extension in the given directory.
 * 
 * Convenience overload for listing files with a single extension.
 * 
 * @param directory The directory to list files from.
 * @param extension The extension to filter by (e.g., ".py").
 *                  Should include the leading dot.
 * @param logger The ROS logger to use for warning messages.
 * @return A sorted vector of filenames (without extensions) found in the directory.
 */
std::vector<std::string> list_files_with_extension(
  const std::string& directory,
  const std::string& extension,
  rclcpp::Logger logger);

} // namespace filesystem_utils

#endif // FILESYSTEM_UTILS_FILESYSTEM_UTILS_H

