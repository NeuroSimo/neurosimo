cmake_minimum_required(VERSION 3.8)
project(session_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)

add_executable(
  session_manager

  src/session_manager.cpp
)

target_link_libraries(session_manager
  realtime_utils::utils
)

ament_target_dependencies(
  session_manager

  rclcpp
  std_msgs

  system_interfaces
  realtime_utils
)

install(TARGETS
  session_manager
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
