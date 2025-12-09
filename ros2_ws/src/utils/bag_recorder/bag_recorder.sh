#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

record_bag() {
  local project_name=$1
  local timestamp=$(date +%Y-%m-%d_%H-%M-%S)
  local bag_directory="/app/projects/${project_name}/bags/${timestamp}"

  echo "Recording new bag for project: ${project_name}"
  ros2 bag record -a -o "${bag_directory}" &
  ROS2_BAG_PID=$!
}

stop_recording_bag() {
  if [ ! -z "$ROS2_BAG_PID" ]; then
    # Send SIGINT to the ros2 bag record process to stop recording gracefully.
    kill -SIGINT "$ROS2_BAG_PID"
    wait "$ROS2_BAG_PID"
    ROS2_BAG_PID=""
  fi
}

# Initialize previous project name.
PREV_PROJECT_NAME=""

# Monitor /projects/active topic for changes.
ros2 topic echo /projects/active | while read -r line
do
  if [[ $line == "data: "* ]]; then
    PROJECT_NAME=$(echo $line | cut -d' ' -f2)

    if [ "$PROJECT_NAME" != "$PREV_PROJECT_NAME" ]; then
      stop_recording_bag

      record_bag $PROJECT_NAME
      PREV_PROJECT_NAME=$PROJECT_NAME
    fi
  fi
done
