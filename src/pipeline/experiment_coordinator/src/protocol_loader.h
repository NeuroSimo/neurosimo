#ifndef PROTOCOL_LOADER_H
#define PROTOCOL_LOADER_H

#include <string>
#include <optional>
#include "protocol.h"
#include "rclcpp/rclcpp.hpp"
#include "pipeline_interfaces/msg/protocol_info.hpp"

namespace experiment_coordinator {

/**
 * @brief Result of protocol loading operation
 */
struct LoadResult {
  bool success;
  std::string error_message;
  std::optional<Protocol> protocol;
};

/**
 * @brief Result of protocol info extraction
 */
struct ProtocolInfoResult {
  bool success;
  std::string error_message;
  std::optional<pipeline_interfaces::msg::ProtocolInfo> protocol_info;
};

/**
 * @brief Loads and validates protocol YAML files
 */
class ProtocolLoader {
public:
  explicit ProtocolLoader(rclcpp::Logger logger);
  
  /**
   * @brief Load protocol from YAML file
   * @param filepath Path to the YAML file
   * @return LoadResult containing protocol or error
   */
  LoadResult load_from_file(const std::string& filepath);
  
  /**
   * @brief Load protocol from project directory
   * @param projects_directory Base projects directory path
   * @param project_name Name of the project
   * @param protocol_filename YAML filename of the protocol
   * @return LoadResult containing protocol or error
   */
  LoadResult load_from_project(
    const std::string& projects_directory,
    const std::string& project_name,
    const std::string& protocol_filename);
  
  /**
   * @brief Get protocol info as ROS message
   * @param projects_directory Base projects directory path
   * @param project_name Name of the project
   * @param protocol_filename YAML filename of the protocol
   * @return ProtocolInfoResult containing ROS message or error
   */
  ProtocolInfoResult get_protocol_info(
    const std::string& projects_directory,
    const std::string& project_name,
    const std::string& protocol_filename);
  
  /**
   * @brief Convert protocol to ROS ProtocolInfo message
   * @param protocol Protocol to convert
   * @param yaml_filename Original YAML filename
   * @return ROS ProtocolInfo message
   */
  static pipeline_interfaces::msg::ProtocolInfo to_protocol_info_msg(
    const Protocol& protocol, 
    const std::string& yaml_filename);
  
private:
  rclcpp::Logger logger;
  
  /**
   * @brief Validate that protocol is well-formed
   * @param protocol Protocol to validate
   * @return true if valid, false otherwise
   */
  bool validate_protocol(const Protocol& protocol, std::string& error_message);
};

} // namespace experiment_coordinator

#endif // PROTOCOL_LOADER_H

