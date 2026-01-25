#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch resource_monitor resource_monitor.launch.py log-level:="$ROS_LOG_LEVEL" \
    disk-warning-threshold:="$DISK_WARNING_THRESHOLD" \
    disk-error-threshold:="$DISK_ERROR_THRESHOLD"
