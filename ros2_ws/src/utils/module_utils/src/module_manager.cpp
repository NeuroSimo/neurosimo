#include "module_utils/module_manager.h"
#include <algorithm>

namespace module_utils {

ModuleManager::ModuleManager(rclcpp::Node* node, const ModuleManagerConfig& config)
  : node_(node),
    logger_(node->get_logger()),
    config_(config) {
  
  RCLCPP_INFO(logger_, "Initializing ModuleManager for component: %s", config_.component_name.c_str());
  
  /* Set up QoS profile for persistent state topics */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  
  /* Publisher for enabled state */
  enabled_publisher_ = node_->create_publisher<std_msgs::msg::Bool>(
    config_.enabled_topic,
    qos_persist_latest);
  
  /* Publisher for current module */
  module_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    config_.module_topic,
    qos_persist_latest);
  
  /* Publisher for module list */
  module_list_publisher_ = node_->create_publisher<project_interfaces::msg::ModuleList>(
    config_.module_list_topic,
    qos_persist_latest);
  
  /* Subscriber for active project */
  active_project_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    config_.active_project_topic,
    qos_persist_latest,
    std::bind(&ModuleManager::handle_set_active_project, this, std::placeholders::_1));
  
  /* Service for enabling/disabling */
  set_enabled_service_ = node_->create_service<std_srvs::srv::SetBool>(
    config_.set_enabled_service,
    std::bind(&ModuleManager::handle_set_enabled, this, std::placeholders::_1, std::placeholders::_2));
  
  /* Service for setting module */
  set_module_service_ = node_->create_service<project_interfaces::srv::SetModule>(
    config_.set_module_service,
    std::bind(&ModuleManager::handle_set_module, this, std::placeholders::_1, std::placeholders::_2));
  
  /* Initialize inotify watcher */
  inotify_watcher_ = std::make_unique<inotify_utils::InotifyWatcher>(
    node_, logger_);
  inotify_watcher_->set_change_callback(
    std::bind(&ModuleManager::update_module_list, this));
  
  RCLCPP_INFO(logger_, "ModuleManager initialized successfully");
}

void ModuleManager::handle_set_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  
  enabled_ = request->data;
  
  /* Publish updated state */
  auto msg = std_msgs::msg::Bool();
  msg.data = enabled_;
  enabled_publisher_->publish(msg);
  
  RCLCPP_INFO(logger_, "%s %s", 
    config_.component_name.c_str(),
    enabled_ ? "enabled" : "disabled");
  
  response->success = true;
}

std::string ModuleManager::get_module_name_with_fallback(const std::string& module_name) {
  /* Check if the requested module exists */
  if (std::find(modules_.begin(), modules_.end(), module_name) != modules_.end()) {
    return module_name;
  }
  
  /* Try default module */
  if (!config_.default_module_name.empty() &&
      std::find(modules_.begin(), modules_.end(), config_.default_module_name) != modules_.end()) {
    RCLCPP_WARN(logger_, "Module %s%s%s not found, setting to default: %s%s%s", 
      bold_on.c_str(), module_name.c_str(), bold_off.c_str(),
      bold_on.c_str(), config_.default_module_name.c_str(), bold_off.c_str());
    return config_.default_module_name;
  }
  
  /* Try first module in list */
  if (!modules_.empty()) {
    RCLCPP_WARN(logger_, "Module %s%s%s not found, setting to first module on the list: %s%s%s", 
      bold_on.c_str(), module_name.c_str(), bold_off.c_str(),
      bold_on.c_str(), modules_[0].c_str(), bold_off.c_str());
    return modules_[0];
  }
  
  /* No modules available */
  RCLCPP_WARN(logger_, "No modules found in project: %s%s%s.", 
    bold_on.c_str(), active_project_.c_str(), bold_off.c_str());
  return "";
}

void ModuleManager::handle_set_module(
    const std::shared_ptr<project_interfaces::srv::SetModule::Request> request,
    std::shared_ptr<project_interfaces::srv::SetModule::Response> response) {
  
  module_name_ = get_module_name_with_fallback(request->module);
  
  /* Publish updated module */
  auto msg = std_msgs::msg::String();
  msg.data = module_name_;
  module_publisher_->publish(msg);
  
  bool success = !module_name_.empty();
  
  if (success) {
    RCLCPP_INFO(logger_, "%s set to: %s%s%s", 
      config_.component_name.c_str(),
      bold_on.c_str(), module_name_.c_str(), bold_off.c_str());
  } else {
    RCLCPP_ERROR(logger_, "No %s module set.", config_.component_name.c_str());
  }
  
  response->success = success;
}

void ModuleManager::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  active_project_ = msg->data;
  
  RCLCPP_INFO(logger_, "");
  RCLCPP_INFO(logger_, "Project set to: %s%s%s", 
    bold_on.c_str(), active_project_.c_str(), bold_off.c_str());
  
  /* Build working directory path */
  working_directory_ = config_.projects_base_directory + "/" + 
                       active_project_ + "/" + 
                       config_.module_subdirectory;
  
  /* Change to the working directory */
  is_working_directory_set_ = filesystem_utils::change_working_directory(
    working_directory_, logger_);
  
  if (is_working_directory_set_) {
    /* Update the module list */
    update_module_list();
    
    /* Update inotify watch */
    inotify_watcher_->update_watch(working_directory_);
  }
}

void ModuleManager::update_module_list() {
  if (is_working_directory_set_) {
    modules_ = filesystem_utils::list_files_with_extensions(
      working_directory_, config_.file_extensions, logger_);
  } else {
    modules_.clear();
  }
  
  /* Publish updated module list */
  auto msg = project_interfaces::msg::ModuleList();
  msg.modules = modules_;
  module_list_publisher_->publish(msg);
  
  RCLCPP_DEBUG(logger_, "Module list updated: %zu modules found", modules_.size());
  
  /* Call user-provided callback if set */
  if (module_list_change_callback_) {
    module_list_change_callback_();
  }
}

} // namespace module_utils

