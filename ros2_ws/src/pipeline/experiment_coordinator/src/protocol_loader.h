#ifndef PROTOCOL_LOADER_H
#define PROTOCOL_LOADER_H

#include <string>
#include <optional>
#include "protocol.h"
#include "rclcpp/rclcpp.hpp"

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

