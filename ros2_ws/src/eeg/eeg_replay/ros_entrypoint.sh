#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch eeg_replay eeg_replay.launch.py log-level:="$ROS_LOG_LEVEL"
