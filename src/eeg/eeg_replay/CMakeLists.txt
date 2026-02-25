cmake_minimum_required(VERSION 3.8)
project(eeg_replay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_interfaces REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_executable(eeg_replay
        src/eeg_replay.cpp
)
ament_target_dependencies(eeg_replay rclcpp std_msgs eeg_interfaces pipeline_interfaces system_interfaces nlohmann_json rosbag2_cpp rosbag2_storage)


install(TARGETS
  eeg_replay
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
