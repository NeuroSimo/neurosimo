# Module Utils

A generic utility package for managing pipeline component modules in the neurosimo system.

## Overview

The `module_utils` package provides a reusable `ModuleManager` class that handles common module management tasks for pipeline components (decider, preprocessor, presenter, etc.):

- **Project Management**: Subscribes to active project changes and updates module directories
- **Module Listing**: Scans and publishes available modules from the project directory
- **Module Selection**: Provides service for setting active modules with fallback logic
- **Enable/Disable**: Service for enabling/disabling components
- **File Watching**: Uses inotify to detect when modules are added/removed/changed
- **ROS Integration**: Manages all necessary publishers, subscribers, and services

## Features

### Key Capabilities

1. **Automatic Module Discovery**: Scans project directories for module files (e.g., `.py` files)
2. **Smart Fallback**: If requested module doesn't exist, falls back to default or first available module
3. **File System Watching**: Detects file changes in real-time using inotify
4. **Consistent State Publishing**: Publishes module state changes to ROS topics with proper QoS
5. **Configurable**: Easy to configure for different components with different requirements

### ROS Communication

For a component named "decider", the ModuleManager automatically creates:

- **Services**:
  - `/pipeline/decider/enabled/set` - Enable/disable the component
  - `/pipeline/decider/module/set` - Set the active module

- **Publishers** (with TRANSIENT_LOCAL durability):
  - `/pipeline/decider/enabled` - Current enabled state
  - `/pipeline/decider/module` - Current active module
  - `/pipeline/decider/list` - List of available modules

- **Subscribers**:
  - `/projects/active` - Active project changes

## Usage

### 1. Include the header

```cpp
#include "module_utils/module_manager.h"
```

### 2. Configure and create the manager

```cpp
// In your node's constructor
module_utils::ModuleManagerConfig config;
config.component_name = "decider";                        // Component name for logging
config.projects_base_directory = "/app/projects";         // Base projects directory
config.module_subdirectory = "decider";                   // Subdirectory within each project
config.file_extensions = {".py"};                         // File extensions to watch
config.default_module_name = "example";                   // Default fallback module

// Topic names (customize as needed)
config.active_project_topic = "/projects/active";
config.module_list_topic = "/pipeline/decider/list";
config.set_module_service = "/pipeline/decider/module/set";
config.module_topic = "/pipeline/decider/module";
config.set_enabled_service = "/pipeline/decider/enabled/set";
config.enabled_topic = "/pipeline/decider/enabled";

// Create the manager
module_manager_ = std::make_unique<module_utils::ModuleManager>(this, config);
```

### 3. Access state information

```cpp
// Check if component is enabled
if (module_manager_->is_enabled()) {
    // Do work
}

// Get current module name
std::string current_module = module_manager_->get_module_name();

// Get working directory
std::string work_dir = module_manager_->get_working_directory();

// Check if working directory is valid
if (module_manager_->is_working_directory_set()) {
    // Safe to use working directory
}

// Get list of available modules
auto modules = module_manager_->get_modules();
```

## Integration Example

Here's a minimal example of integrating ModuleManager into a pipeline component:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "module_utils/module_manager.h"

class MyPipelineComponent : public rclcpp::Node {
public:
  MyPipelineComponent() : Node("my_component") {
    // Configure module manager
    module_utils::ModuleManagerConfig config;
    config.component_name = "my_component";
    config.projects_base_directory = "/app/projects";
    config.module_subdirectory = "my_component";
    config.file_extensions = {".py"};
    config.default_module_name = "example";
    config.module_list_topic = "/pipeline/my_component/list";
    config.set_module_service = "/pipeline/my_component/module/set";
    config.module_topic = "/pipeline/my_component/module";
    config.set_enabled_service = "/pipeline/my_component/enabled/set";
    config.enabled_topic = "/pipeline/my_component/enabled";
    
    module_manager_ = std::make_unique<module_utils::ModuleManager>(this, config);
  }
  
  void process() {
    if (!module_manager_->is_enabled()) {
      return;  // Component disabled
    }
    
    if (module_manager_->get_module_name().empty()) {
      RCLCPP_WARN(get_logger(), "No module set");
      return;
    }
    
    // Use module_manager_->get_module_name() and 
    // module_manager_->get_working_directory() as needed
  }

private:
  std::unique_ptr<module_utils::ModuleManager> module_manager_;
};
```

## Building

### Add to your package's CMakeLists.txt

```cmake
find_package(module_utils REQUIRED)

target_link_libraries(your_target
  module_utils::module_utils
)

ament_target_dependencies(your_target
  module_utils
)
```

### Add to your package.xml

```xml
<depend>module_utils</depend>
```

## Dependencies

- `rclcpp` - ROS 2 C++ client library
- `std_msgs` - Standard ROS messages
- `std_srvs` - Standard ROS services
- `project_interfaces` - Custom project interfaces
- `filesystem_utils` - File system utilities
- `inotify_utils` - File watching utilities

## Design Philosophy

The ModuleManager follows these principles:

1. **Single Responsibility**: Handles only module management concerns
2. **Reusability**: Same class works for all pipeline components
3. **Configuration over Code**: Behavior customized through config struct
4. **Consistent State**: All state changes published to ROS topics
5. **Fail-Safe**: Smart fallback logic when modules don't exist

## Migration Guide

If you're migrating from custom module management code:

### Before (in your node):
```cpp
// Multiple subscribers, publishers, services
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
rclcpp::Publisher<project_interfaces::msg::FilenameList>::SharedPtr module_list_publisher;
rclcpp::Service<project_interfaces::srv::SetModule>::SharedPtr set_module_service;
// ... many more

// State variables
std::string module_name;
std::string working_directory;
std::vector<std::string> modules;
bool is_working_directory_set;

// Handler methods
void handle_set_active_project(...);
void handle_set_module(...);
void update_module_list();
// ... many more
```

### After (with ModuleManager):
```cpp
// Single manager instance
std::unique_ptr<module_utils::ModuleManager> module_manager_;

// Access state through getters
module_manager_->get_module_name();
module_manager_->get_working_directory();
module_manager_->is_enabled();
```

## License

GPL-3.0-only

