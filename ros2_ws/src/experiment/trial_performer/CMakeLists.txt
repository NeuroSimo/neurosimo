cmake_minimum_required(VERSION 3.8)
project(trial_performer)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

find_package(event_interfaces REQUIRED)
find_package(experiment_interfaces REQUIRED)
find_package(mep_interfaces REQUIRED)
find_package(mtms_device_interfaces REQUIRED)
find_package(neuronavigation_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)

add_executable(
        trial_performer

        src/trial_performer.cpp
        src/headers/trial_performer.h

        src/memory_utils.cpp
        src/headers/memory_utils.h

        src/scheduling_utils.cpp
        src/headers/scheduling_utils.h
)

target_include_directories(trial_performer
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

ament_target_dependencies(
        trial_performer
        rclcpp
        rclcpp_action
        std_msgs
        event_interfaces
        experiment_interfaces
        mep_interfaces
        mtms_device_interfaces
        neuronavigation_interfaces
        system_interfaces
)

install(
        TARGETS
        trial_performer
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
