#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/install/setup.bash

ros2 launch project_manager project_manager.launch.py log-level:="$ROS_LOG_LEVEL"
