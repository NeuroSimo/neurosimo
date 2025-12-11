cmake_minimum_required(VERSION 3.8)
project(module_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(filesystem_utils REQUIRED)
find_package(inotify_utils REQUIRED)

add_library(module_utils SHARED src/module_manager.cpp)
target_include_directories(module_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(module_utils 
  rclcpp 
  std_msgs 
  std_srvs 
  project_interfaces
  filesystem_utils
  inotify_utils
)

# Define an alias for the library
add_library(module_utils::module_utils ALIAS module_utils)

# Install the library and export the target
install(TARGETS module_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export the targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp 
  std_msgs 
  std_srvs 
  project_interfaces
  filesystem_utils
  inotify_utils
)
ament_package()

