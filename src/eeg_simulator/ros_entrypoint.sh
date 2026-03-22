#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

ros2 launch eeg_simulator eeg_simulator.launch.py log-level:="$ROS_LOG_LEVEL"
