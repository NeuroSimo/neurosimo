cmake_minimum_required(VERSION 3.8)
project(inotify_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(inotify_watcher SHARED src/inotify_watcher.cpp)
target_include_directories(inotify_watcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(inotify_watcher rclcpp)

# Define an alias for the library
add_library(inotify_utils::inotify_watcher ALIAS inotify_watcher)

# Install the library and export the target
install(TARGETS inotify_watcher
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
ament_export_dependencies(rclcpp)
ament_package()

