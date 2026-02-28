#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch global_configurator global_configurator.launch.py log-level:="$ROS_LOG_LEVEL"
