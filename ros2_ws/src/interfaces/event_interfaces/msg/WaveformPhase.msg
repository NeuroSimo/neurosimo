# TODO: WaveformPhase is naturally an enum. However, enums are not
# yet supported by ROS2, therefore simulate it by using constants
# (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

# See mTMS operator's manual for a description of the waveform phase.

uint8 NON_CONDUCTIVE=0
uint8 RISING=1
uint8 HOLD=2
uint8 FALLING=3
uint8 ALTERNATIVE_HOLD=4

uint8 value
