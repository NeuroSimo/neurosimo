#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

# Note: Action calls can take a while, so run them in separate threads to avoid blocking. Service calls should be fast, so
#   run them consecutively in the main thread. This logic seems to work for now in the front-end.
ros2 launch rosbridge_server rosbridge_websocket_launch.xml call_services_in_new_thread:=false send_action_goals_in_new_thread:=true port:=$ROSBRIDGE_PORT
