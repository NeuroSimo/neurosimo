cmake_minimum_required(VERSION 3.8)
project(ui_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(neuronavigation_interfaces REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Event.msg"
  "msg/Target.msg"
  "msg/PlannerState.msg"
  "msg/ChannelInfo.msg"
  "msg/Session.msg"
  "msg/PulseSequence.msg"
  "msg/Pulse.msg"
  "msg/Train.msg"
  "srv/AddTarget.srv"
  "srv/ChangeComment.srv"
  "srv/RemoveTarget.srv"
  "srv/RenameTarget.srv"
  "srv/ChangeTargetIndex.srv"
  "srv/SetTarget.srv"
  "srv/ToggleSelectTarget.srv"
  "srv/ToggleVisible.srv"
  "srv/SetTargetOrientation.srv"
  "srv/AddPulseSequence.srv"
  "srv/RenamePulseSequence.srv"
  "srv/ToggleSelectPulseSequence.srv"
  "srv/RemovePulseSequence.srv"
  "srv/RemovePulse.srv"
  "srv/SetPulseSequenceIsi.srv"
  "srv/SetPulseSequenceIntensity.srv"
  "srv/SetPulseIntensity.srv"
  "srv/SetPulseIsi.srv"
  "srv/ToggleSelectPulse.srv"
  "srv/ToggleVisiblePulse.srv"
  "srv/ChangePulseIndex.srv"
  "srv/AddPulseToPulseSequence.srv"
  "srv/ToggleNavigation.srv"
  "srv/StartPulseSequence.srv"
  "srv/ClearState.srv"

  DEPENDENCIES geometry_msgs neuronavigation_interfaces
 )

ament_package()
