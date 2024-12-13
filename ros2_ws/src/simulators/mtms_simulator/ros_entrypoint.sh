#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch mtms_simulator mtms_simulator.launch.py log-level:="$ROS_LOG_LEVEL" channels:="$MTMS_DEVICE_SIMULATOR_NUM_OF_CHANNELS"