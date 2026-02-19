cmake_minimum_required(VERSION 3.8)
project(eeg_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_interfaces REQUIRED)

add_executable(eeg_monitor
        src/eeg_monitor.cpp
)
ament_target_dependencies(eeg_monitor rclcpp std_msgs eeg_interfaces)


install(TARGETS
  eeg_monitor
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
