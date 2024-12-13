cmake_minimum_required(VERSION 3.8)
project(waveforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(event_interfaces REQUIRED)
find_package(targeting_interfaces REQUIRED)

find_package(Boost REQUIRED)

# Install get_default_waveform executable.

add_executable(get_default_waveform
  src/nodes/get_default_waveform.cpp
  src/lib/memory_utils.cpp
  include/memory_utils.h
  src/lib/scheduling_utils.cpp
  include/scheduling_utils.h)

target_include_directories(get_default_waveform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(get_default_waveform rclcpp std_msgs event_interfaces targeting_interfaces)

# Install get_multipulse_waveforms executable.

add_executable(get_multipulse_waveforms
  src/nodes/get_multipulse_waveforms.cpp
  src/lib/memory_utils.cpp
  include/memory_utils.h
  src/lib/scheduling_utils.cpp
  include/scheduling_utils.h)

target_include_directories(get_multipulse_waveforms
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(get_multipulse_waveforms rclcpp std_msgs event_interfaces targeting_interfaces)

target_link_libraries(get_multipulse_waveforms Boost::boost)

# Install reverse_polarity executable.

add_executable(reverse_polarity
  src/nodes/reverse_polarity.cpp
  src/lib/memory_utils.cpp
  include/memory_utils.h
  src/lib/scheduling_utils.cpp
  include/scheduling_utils.h)

target_include_directories(reverse_polarity
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(reverse_polarity rclcpp std_msgs event_interfaces targeting_interfaces)


install(TARGETS
  get_default_waveform
  get_multipulse_waveforms
  reverse_polarity
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
