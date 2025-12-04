cmake_minimum_required(VERSION 3.8)
project(eeg_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eeg_msgs REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(project_interfaces REQUIRED)

add_executable(
  eeg_recorder

  src/eeg_recorder.cpp
)

ament_target_dependencies(
  eeg_recorder
  rclcpp
  std_msgs
  std_srvs
  eeg_msgs
  system_interfaces
  project_interfaces
)

install(TARGETS
  eeg_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
