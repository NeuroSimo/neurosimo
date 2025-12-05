#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch experiment_coordinator experiment_coordinator.launch.py log-level:="$ROS_LOG_LEVEL" \
    dropped-sample-threshold:="$DROPPED_SAMPLE_THRESHOLD"
