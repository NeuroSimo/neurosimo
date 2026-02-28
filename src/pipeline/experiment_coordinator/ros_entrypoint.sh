#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch experiment_coordinator experiment_coordinator.launch.py log-level:="$ROS_LOG_LEVEL"
