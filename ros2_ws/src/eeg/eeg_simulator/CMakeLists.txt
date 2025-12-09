cmake_minimum_required(VERSION 3.8)
project(eeg_simulator)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(eeg_interfaces REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)

add_executable(
        eeg_simulator

        src/eeg_simulator.cpp
)

target_include_directories(eeg_simulator
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

target_link_libraries(eeg_simulator
  realtime_utils::utils
)

ament_target_dependencies(
  eeg_simulator
  nlohmann_json
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  eeg_interfaces
  project_interfaces
  system_interfaces
  realtime_utils
)

install(TARGETS
        eeg_simulator
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
