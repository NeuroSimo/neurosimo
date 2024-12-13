cmake_minimum_required(VERSION 3.8)
project(eeg_gatherer)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eeg_interfaces REQUIRED)

add_executable(
        eeg_gatherer

        src/gather_eeg_server.cpp
        src/headers/gather_eeg_server.h

        src/eeg_gatherer.cpp
        src/headers/eeg_gatherer.h

        src/memory_utils.cpp
        src/headers/memory_utils.h

        src/scheduling_utils.cpp
        src/headers/scheduling_utils.h
)

target_include_directories(eeg_gatherer
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

ament_target_dependencies(
  eeg_gatherer
  rclcpp
  rclcpp_action
  std_msgs
  eeg_interfaces
)

install(TARGETS
        eeg_gatherer
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
