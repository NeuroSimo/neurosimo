# TODO: PulseError is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NO_ERROR=0
uint8 INVALID_EXECUTION_CONDITION=1
uint8 INVALID_CHANNEL=2
uint8 INVALID_NUMBER_OF_WAVEFORM_PIECES=3
uint8 INVALID_MODES=4
uint8 INVALID_DURATIONS=5
uint8 LATE=6
uint8 TOO_MANY_PULSES=7
uint8 OVERLAPPING_WITH_CHARGING=8
uint8 OVERLAPPING_WITH_DISCHARGING=9
uint8 NOT_ALLOWED=10
uint8 TRIGGERING_FAILURE=11
uint8 UNKNOWN_ERROR=12

uint8 value
