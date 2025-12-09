cmake_minimum_required(VERSION 3.8)
project(presenter)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include_directories(${Python3_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(eeg_interfaces REQUIRED)
find_package(pipeline_interfaces REQUIRED)
find_package(project_interfaces REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(realtime_utils REQUIRED)

add_executable(
        presenter

        src/presenter.cpp
        src/presenter_wrapper.cpp
)

target_include_directories(presenter
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>)

target_link_libraries(presenter
        realtime_utils::utils
)

ament_target_dependencies(
  presenter
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  eeg_interfaces
  pipeline_interfaces
  project_interfaces
  system_interfaces
  realtime_utils
)

install(TARGETS
        presenter
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

# PyBind11 modules

add_library(cpp_bindings MODULE src/cpp_bindings.cpp src/presenter_wrapper.cpp)

# XXX: The value of PREFIX should be "${PYTHON_MODULE_PREFIX}" and SUFFIX should be "${PYTHON_MODULE_EXTENSION}".
#   However, they seem to be undefined, resulting in empty strings, hence hardcode the suffix to .so.
set_target_properties(cpp_bindings PROPERTIES PREFIX "" SUFFIX ".so")

ament_target_dependencies(
  cpp_bindings
  pybind11_vendor
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  eeg_interfaces
  pipeline_interfaces
  project_interfaces
  system_interfaces
)

# Find the user's local site-packages directory
execute_process(COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.USER_SITE)"
                OUTPUT_VARIABLE PYTHON_LOCAL_SITE_PACKAGES
                OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "Installing Python bindings to: ${PYTHON_LOCAL_SITE_PACKAGES}")

install(TARGETS cpp_bindings
        DESTINATION ${PYTHON_LOCAL_SITE_PACKAGES})
