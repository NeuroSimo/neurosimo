#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch eeg_bridge eeg_bridge.launch.py \
  log-level:="$ROS_LOG_LEVEL" \
  port:="$EEG_PORT" \
  maximum-dropped-samples:="$MAXIMUM_DROPPED_SAMPLES" \
  eeg-device:="$EEG_DEVICE" \
  turbolink-sampling-frequency:="$TURBOLINK_SAMPLING_FREQUENCY" \
  turbolink-eeg-channel-count:="$TURBOLINK_EEG_CHANNEL_COUNT"
