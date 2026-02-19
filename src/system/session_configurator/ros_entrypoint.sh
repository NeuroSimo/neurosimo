#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/install/setup.bash

ros2 launch session_configurator session_configurator.launch.py log-level:="$ROS_LOG_LEVEL"
