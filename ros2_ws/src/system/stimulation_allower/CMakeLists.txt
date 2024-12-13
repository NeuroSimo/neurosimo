cmake_minimum_required(VERSION 3.8)
project(stimulation_allower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(neuronavigation_interfaces REQUIRED)
find_package(mtms_device_interfaces REQUIRED)
find_package(stimulation_interfaces REQUIRED)

# Install stimulation_allower executable.

add_executable(stimulation_allower
  src/nodes/stimulation_allower.cpp
  src/lib/memory_utils.cpp
  include/memory_utils.h
  src/lib/scheduling_utils.cpp
  include/scheduling_utils.h)

target_include_directories(stimulation_allower
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(stimulation_allower rclcpp std_msgs neuronavigation_interfaces mtms_device_interfaces stimulation_interfaces)

install(TARGETS
  stimulation_allower
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
