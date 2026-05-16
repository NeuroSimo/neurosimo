cmake_minimum_required(VERSION 3.8)
project(trigger_simulator)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(neurosimo_eeg_interfaces REQUIRED)
find_package(neurosimo_pipeline_interfaces REQUIRED)
find_package(neurosimo_system_interfaces REQUIRED)

include_directories(include)

add_executable(
        trigger_simulator
        src/trigger_simulator.cpp
)

target_include_directories(trigger_simulator
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

ament_target_dependencies(
  trigger_simulator
  rclcpp
  std_msgs
  neurosimo_eeg_interfaces
  neurosimo_pipeline_interfaces
  neurosimo_system_interfaces
)

install(TARGETS
        trigger_simulator
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
