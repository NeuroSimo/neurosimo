# TODO: ExperimentState should be an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NOT_RUNNING=0
uint8 RUNNING=1
uint8 PAUSED=2
uint8 CANCELED=3

uint8 value
