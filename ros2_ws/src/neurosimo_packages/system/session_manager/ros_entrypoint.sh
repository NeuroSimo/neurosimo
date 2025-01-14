#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

# Return early if the mTMS device is enabled
if [ "$MTMS_DEVICE_ENABLED" = "true" ]; then
    echo "mTMS device is enabled, skipping session manager"
    exit 0
fi

ros2 launch session_manager session_manager.launch.py log-level:="$ROS_LOG_LEVEL"
