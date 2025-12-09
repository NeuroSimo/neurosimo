#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch trigger_timer trigger_timer.launch.py log-level:="$ROS_LOG_LEVEL" \
    triggering-tolerance:="$TRIGGERING_TOLERANCE" \
    simulate-labjack:="$SIMULATE_LABJACK"
