cmake_minimum_required(VERSION 3.8)
project(pipeline_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(eeg_interfaces REQUIRED)

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
  "msg/CoilTarget.msg"
  "msg/TimedTrigger.msg"
  "msg/TriggerLoopbackLatency.msg"
  "msg/DecisionTrace.msg"
  "msg/SensoryStimulus.msg"
  "msg/LogMessage.msg"
  "msg/LogMessages.msg"
  "msg/ExperimentState.msg"
  "msg/ProtocolStageInfo.msg"
  "msg/ProtocolRestInfo.msg"
  "msg/ProtocolElementInfo.msg"
  "msg/ProtocolInfo.msg"

  "srv/RequestTimedTrigger.srv"

  "srv/InitializeProtocol.srv"
  "srv/GetProtocolInfo.srv"
  "srv/FinalizePreprocessor.srv"
  "srv/FinalizeDecider.srv"
  "srv/FinalizePresenter.srv"
  "srv/FinalizeStimulationTracer.srv"
  "srv/FinalizeTriggerTimer.srv"

  "srv/InitializePreprocessor.srv"
  "srv/InitializeDecider.srv"
  "srv/InitializePresenter.srv"
  "srv/InitializeStimulationTracer.srv"
  "srv/InitializeTriggerTimer.srv"

  DEPENDENCIES
    diagnostic_msgs
    std_msgs
    eeg_interfaces
)

ament_package()
