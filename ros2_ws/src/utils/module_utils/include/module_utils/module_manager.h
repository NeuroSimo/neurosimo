#ifndef MODULE_UTILS_MODULE_MANAGER_H
#define MODULE_UTILS_MODULE_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "project_interfaces/msg/module_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "inotify_utils/inotify_watcher.h"
#include "filesystem_utils/filesystem_utils.h"

namespace module_utils {

/**
 * @brief Configuration structure for the ModuleManager.
 * 
 * This structure contains all the configuration needed to set up a generic
 * module manager for pipeline components.
 */
struct ModuleManagerConfig {
  /* Name of the component (e.g., "decider", "preprocessor", "presenter") */
  std::string component_name;
  
  /* Base directory where projects are stored (e.g., "/app/projects") */
  std::string projects_base_directory;
  
  /* Subdirectory within each project for this component's modules 
     (e.g., "decider", "preprocessor", "presenter") */
  std::string module_subdirectory;
  
  /* File extensions to watch for (e.g., {".py"} or {".py", ".yaml"}) */
  std::vector<std::string> file_extensions;
  
  /* Default module name to fall back to if requested module not found */
  std::string default_module_name;
  
  /* Topic names for ROS communication */
  std::string active_project_topic = "/projects/active";
  std::string module_list_topic;       // e.g., "/pipeline/decider/list"
  std::string set_module_service;      // e.g., "/pipeline/decider/module/set"
  std::string module_topic;            // e.g., "/pipeline/decider/module"
  std::string set_enabled_service;     // e.g., "/pipeline/decider/enabled/set"
  std::string enabled_topic;           // e.g., "/pipeline/decider/enabled"
};

/**
 * @brief A generic manager for pipeline component modules.
 * 
 * This class handles:
 * - Subscribing to active project changes
 * - Listing available modules in the project directory
 * - Setting and publishing the active module
 * - Enabling/disabling the component
 * - Watching for file system changes using inotify
 * 
 * It can be used by any pipeline component (decider, preprocessor, presenter, etc.)
 * to manage its modules in a consistent way.
 */
class ModuleManager {
public:
  /**
   * @brief Construct a new Module Manager object.
   * 
   * @param node Pointer to the ROS2 node that owns this manager
   * @param config Configuration for the module manager
   */
  ModuleManager(rclcpp::Node* node, const ModuleManagerConfig& config);
  
  /**
   * @brief Destroy the Module Manager object.
   */
  ~ModuleManager() = default;

  /* Getters */
  
  /**
   * @brief Check if the component is enabled.
   * @return true if enabled, false otherwise
   */
  bool is_enabled() const { return enabled_; }
  
  /**
   * @brief Get the current module name.
   * @return Current module name (may be empty if not set)
   */
  std::string get_module_name() const { return module_name_; }
  
  /**
   * @brief Get the active project name.
   * @return Active project name (may be empty if not set)
   */
  std::string get_active_project() const { return active_project_; }
  
  /**
   * @brief Get the list of available modules.
   * @return Vector of module names
   */
  std::vector<std::string> get_modules() const { return modules_; }
  
  /**
   * @brief Get the working directory for the current project.
   * @return Full path to the working directory
   */
  std::string get_working_directory() const { return working_directory_; }
  
  /**
   * @brief Check if the working directory is successfully set.
   * @return true if directory exists and is accessible
   */
  bool is_working_directory_set() const { return is_working_directory_set_; }

private:
  /* ROS service and subscriber handlers */
  
  void handle_set_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  void handle_set_module(
    const std::shared_ptr<project_interfaces::srv::SetModule::Request> request,
    std::shared_ptr<project_interfaces::srv::SetModule::Response> response);
  
  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  
  /* Internal methods */
  
  /**
   * @brief Update the list of available modules from the file system.
   * 
   * Scans the working directory for files with the configured extensions,
   * publishes the updated list, and calls the change callback if set.
   */
  void update_module_list();
  
  /**
   * @brief Get module name with fallback logic.
   * 
   * If the requested module doesn't exist:
   * 1. Try the default module
   * 2. Try the first module in the list
   * 3. Return empty string if no modules available
   * 
   * @param module_name Requested module name
   * @return Valid module name or empty string
   */
  std::string get_module_name_with_fallback(const std::string& module_name);

  /* ROS node and logger */
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  
  /* Configuration */
  ModuleManagerConfig config_;
  
  /* ROS publishers */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr module_publisher_;
  rclcpp::Publisher<project_interfaces::msg::ModuleList>::SharedPtr module_list_publisher_;
  
  /* ROS subscribers */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber_;
  
  /* ROS services */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enabled_service_;
  rclcpp::Service<project_interfaces::srv::SetModule>::SharedPtr set_module_service_;
  
  /* State variables */
  bool enabled_ = false;
  std::string module_name_;
  std::string active_project_;
  std::vector<std::string> modules_;
  std::string working_directory_;
  bool is_working_directory_set_ = false;
  
  /* File system watcher */
  std::unique_ptr<inotify_utils::InotifyWatcher> inotify_watcher_;
  
  /* Callback for module list changes */
  std::function<void()> module_list_change_callback_;
  
  /* ANSI formatting for logs */
  const std::string bold_on = "\033[1m";
  const std::string bold_off = "\033[0m";
};

} // namespace module_utils

#endif // MODULE_UTILS_MODULE_MANAGER_H

