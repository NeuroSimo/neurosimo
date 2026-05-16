#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch trigger_simulator trigger_simulator.launch.py \
  log-level:="$ROS_LOG_LEVEL"
