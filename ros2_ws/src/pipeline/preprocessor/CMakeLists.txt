cmake_minimum_required(VERSION 3.8)
project(preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eeg_interfaces REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)

find_package(realtime_utils REQUIRED)
find_package(filesystem_utils REQUIRED)

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  find_package(pybind11_vendor REQUIRED)
endif()

add_library(preprocessor_core SHARED
  src/preprocessor_wrapper.cpp
)

target_include_directories(preprocessor_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common/include>
    $<INSTALL_INTERFACE:lib>
    ${Python3_INCLUDE_DIRS}
)

ament_target_dependencies(preprocessor_core
  rclcpp
  std_msgs
  std_srvs
  eeg_interfaces
  pipeline_interfaces
  project_interfaces
  system_interfaces
)

target_link_libraries(preprocessor_core
  Python3::Python
)

add_executable(preprocessor
  src/preprocessor.cpp
)

ament_target_dependencies(preprocessor
  rclcpp
  std_msgs
  std_srvs
  eeg_interfaces
  pipeline_interfaces
  project_interfaces
  system_interfaces
  realtime_utils
  filesystem_utils
)

target_link_libraries(preprocessor
  preprocessor_core
  realtime_utils::utils
  filesystem_utils::filesystem_utils
)

install(TARGETS
  preprocessor
  preprocessor_core
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(cpp_bindings MODULE
  src/cpp_bindings.cpp
)

set_target_properties(cpp_bindings PROPERTIES PREFIX "")

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
  OUTPUT_VARIABLE PYTHON_EXT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(PYTHON_EXT_SUFFIX STREQUAL "")
  set(PYTHON_EXT_SUFFIX ".so")
endif()
set_target_properties(cpp_bindings PROPERTIES SUFFIX "${PYTHON_EXT_SUFFIX}")

target_include_directories(cpp_bindings
  PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/include
)

ament_target_dependencies(cpp_bindings
  rclcpp
)

target_link_libraries(cpp_bindings
  preprocessor_core
  Python3::Python
)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.USER_SITE)"
  OUTPUT_VARIABLE PYTHON_LOCAL_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Installing Python bindings to: ${PYTHON_LOCAL_SITE_PACKAGES}")

install(TARGETS cpp_bindings
  DESTINATION ${PYTHON_LOCAL_SITE_PACKAGES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
message(STATUS "TARGET: ${CMAKE_BUILD_TYPE}")
