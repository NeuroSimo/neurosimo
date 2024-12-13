# TODO: DischargeError is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NO_ERROR=0
uint8 INVALID_EXECUTION_CONDITION=1
uint8 INVALID_CHANNEL=2
uint8 INVALID_VOLTAGE=3
uint8 LATE=4
uint8 OVERLAPPING_WITH_CHARGING=5
uint8 OVERLAPPING_WITH_STIMULATION=6
uint8 DISCHARGING_FAILURE=7
uint8 UNKNOWN_ERROR=8

uint8 value
