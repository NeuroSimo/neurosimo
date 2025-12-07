# TODO: GatherEegError is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NO_ERROR=0
uint8 LATE=1
uint8 SAMPLES_DROPPED=2

uint8 value
