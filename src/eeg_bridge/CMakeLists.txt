cmake_minimum_required(VERSION 3.8)
project(eeg_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(neurosimo_eeg_interfaces REQUIRED)
find_package(neurosimo_system_interfaces REQUIRED)
find_package(neurosimo_pipeline_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(XXHASH REQUIRED libxxhash)

add_executable(eeg_bridge
        src/adapters/eeg_adapter.h
        src/adapters/neurone_adapter.cpp
        src/adapters/neurone_adapter.h
        src/adapters/socket.cpp
        src/adapters/socket.h
        src/adapters/turbolink_adapter.h
        src/adapters/turbolink_adapter.cpp
        src/eeg_bridge.cpp
        src/eeg_bridge.h
)

target_link_libraries(eeg_bridge
  realtime_utils::utils
  ${XXHASH_LIBRARIES}
)

target_include_directories(eeg_bridge PRIVATE ${XXHASH_INCLUDE_DIRS})

ament_target_dependencies(eeg_bridge rclcpp std_msgs std_srvs neurosimo_eeg_interfaces neurosimo_system_interfaces neurosimo_pipeline_interfaces realtime_utils)


install(TARGETS
  eeg_bridge
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
