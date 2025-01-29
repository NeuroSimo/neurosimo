#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

ros2 launch eeg_batcher eeg_batcher.launch.py log-level:="$ROS_LOG_LEVEL" batch-size:="$EEG_BATCHER_BATCH_SIZE" downsample-ratio:="$EEG_BATCHER_DOWNSAMPLE_RATIO"
