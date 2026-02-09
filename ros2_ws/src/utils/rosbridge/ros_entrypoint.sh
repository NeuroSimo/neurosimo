#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$ROSBRIDGE_PORT
