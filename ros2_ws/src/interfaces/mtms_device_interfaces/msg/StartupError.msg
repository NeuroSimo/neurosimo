# TODO: Startup error is inherently one (and only one) value out of several
# options (cf. SystemError and ChannelError, which can have several concurrent
# errors), thus enum is the natural structure to represent it. However,
# enums are not yet supported by ROS2, therefore simulate it by
# using constants (see link below for an example). Change to actual enum when
# it is available in ROS2.
#
# https://github.com/ros2/rosidl/issues/260

uint8 NO_ERROR=0
uint8 UART_INITIALIZATION_ERROR=1
uint8 BOARD_STARTUP_ERROR=2
uint8 BOARD_STATUS_MESSAGE_ERROR=3
uint8 SAFETY_MONITOR_ERROR=4
uint8 DISCHARGE_CONTROLLER_ERROR=5
uint8 CHARGER_ERROR=6
uint8 SENSORBOARD_ERROR=7
uint8 DISCHARGE_CONTROLLER_VOLTAGE_ERROR=8
uint8 CHARGER_VOLTAGE_ERROR=9
uint8 IGBT_FEEDBACK_ERROR=10
uint8 TEMPERATURE_SENSOR_PRESENCE_ERROR=11
uint8 COIL_MEMORY_PRESENCE_ERROR=12

uint8 value
