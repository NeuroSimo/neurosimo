# TODO: Event type is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when it is
# available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 CHARGE=0
uint8 PULSE=1
uint8 DISCHARGE=2
uint8 TRIGGER_OUT=3

float64 when_to_execute
float64 processing_start_time

uint8 event_type
