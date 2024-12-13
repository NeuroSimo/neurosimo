# TODO: Device state is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when it is
# available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NOT_OPERATIONAL=0
uint8 STARTUP=1
uint8 OPERATIONAL=2
uint8 SHUTDOWN=3

uint8 value
