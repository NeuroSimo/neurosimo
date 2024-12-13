#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch pulse_sequence_controller pulse_sequence_controller.launch.py
