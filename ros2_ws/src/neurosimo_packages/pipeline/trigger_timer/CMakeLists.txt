cmake_minimum_required(VERSION 3.8)
project(trigger_timer)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_msgs REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)

include_directories(include include/labjack)

find_library(LABJACKM_LIBRARY NAMES LabJackM)

add_executable(
        trigger_timer

        src/trigger_timer.cpp
)

target_include_directories(trigger_timer
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

target_link_libraries(trigger_timer ${LABJACKM_LIBRARY})

target_link_libraries(trigger_timer
  realtime_utils::utils
)

ament_target_dependencies(
  trigger_timer
  rclcpp
  rclcpp_action
  std_msgs
  eeg_msgs
  pipeline_interfaces
  system_interfaces
  realtime_utils
)

install(TARGETS
        trigger_timer
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
