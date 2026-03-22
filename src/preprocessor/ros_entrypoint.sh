#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

while true; do
    ros2 launch preprocessor preprocessor.launch.py log-level:="$ROS_LOG_LEVEL"
    echo "Preprocessor exited, restarting..."
done
