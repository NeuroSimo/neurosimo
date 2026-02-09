cmake_minimum_required(VERSION 3.8)
project(system_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(eeg_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DiskStatus.msg"
  "msg/ExportDataType.msg"
  "msg/ExporterState.msg"
  "msg/RecorderState.msg"
  "msg/GlobalConfig.msg"
  "msg/SessionConfig.msg"
  "msg/SessionState.msg"
  "msg/DataSourceState.msg"
  "msg/ComponentHealth.msg"
  "srv/AbortSession.srv"
  "srv/CancelExport.srv"
  "srv/ExportSession.srv"
  "srv/GetGlobalConfig.srv"
  "srv/GetSessionConfig.srv"
  "srv/StartRecording.srv"
  "srv/StopRecording.srv"
  DEPENDENCIES eeg_interfaces
)

ament_package()
