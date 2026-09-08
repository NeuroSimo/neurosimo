#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch system_configurator system_configurator.launch.py log-level:="$ROS_LOG_LEVEL"
