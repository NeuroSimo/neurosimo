cmake_minimum_required(VERSION 3.8)
project(realtime_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(utils SHARED src/utils.cpp)
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(utils rclcpp)

# Define an alias for the utils library
add_library(realtime_utils::utils ALIAS utils)

# Install the library and export the target
install(TARGETS utils
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
