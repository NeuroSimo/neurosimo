cmake_minimum_required(VERSION 3.8)
project(eeg_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)

add_executable(eeg_bridge
        src/adapters/eeg_adapter.h
        src/adapters/neurone_adapter.cpp
        src/adapters/neurone_adapter.h
        src/adapters/turbolink_adapter.h
        src/adapters/turbolink_adapter.cpp
        src/eeg_bridge.cpp
        src/eeg_bridge.h
)

target_link_libraries(eeg_bridge
  realtime_utils::utils
)

add_executable(mock_eeg_bridge src/mock_eeg_bridge.cpp)
ament_target_dependencies(eeg_bridge rclcpp std_msgs eeg_interfaces system_interfaces pipeline_interfaces realtime_utils)
ament_target_dependencies(mock_eeg_bridge rclcpp std_msgs eeg_interfaces system_interfaces pipeline_interfaces realtime_utils)


install(TARGETS
  eeg_bridge
  mock_eeg_bridge
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
