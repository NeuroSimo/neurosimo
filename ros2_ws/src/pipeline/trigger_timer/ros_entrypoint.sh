#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch trigger_timer trigger_timer.launch.py log-level:="$ROS_LOG_LEVEL" \
    maximum-timing-error:="$MAXIMUM_TIMING_ERROR" \
    simulate-labjack:="$SIMULATE_LABJACK" \
    maximum-loopback-latency:="$MAXIMUM_LOOPBACK_LATENCY"
