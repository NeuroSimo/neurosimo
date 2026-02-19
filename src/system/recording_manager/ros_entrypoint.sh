#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/install/setup.bash

ros2 launch recording_manager recording_manager.launch.py log-level:="$ROS_LOG_LEVEL"
