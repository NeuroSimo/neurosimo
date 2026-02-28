#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch eeg_replay eeg_replay.launch.py log-level:="$ROS_LOG_LEVEL"
