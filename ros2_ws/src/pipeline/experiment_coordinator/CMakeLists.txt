cmake_minimum_required(VERSION 3.8)
project(experiment_coordinator)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eeg_interfaces REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(inotify_utils REQUIRED)
find_package(filesystem_utils REQUIRED)

# Experiment coordinator executable
add_executable(
  experiment_coordinator

  src/experiment_coordinator_node.cpp
  src/experiment_coordinator.cpp
  src/protocol_loader.cpp
)

target_include_directories(experiment_coordinator
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:src>)

target_link_libraries(experiment_coordinator
  yaml-cpp
  inotify_utils::inotify_watcher
  filesystem_utils::filesystem_utils
)

ament_target_dependencies(
  experiment_coordinator
  rclcpp
  std_msgs
  std_srvs
  eeg_interfaces
  project_interfaces
  pipeline_interfaces
  system_interfaces
  inotify_utils
  filesystem_utils
)

install(
  TARGETS
  experiment_coordinator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
message(STATUS "TARGET: ${CMAKE_BUILD_TYPE}")

