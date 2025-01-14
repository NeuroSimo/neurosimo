#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

if [ -z "$BAG_ANALYSIS_BAG_NAME" ]; then
  echo "Error: BAG_ANALYSIS_BAG_NAME is unset in .env file."
  exit 1
fi

if [ -z "$BAG_ANALYSIS_TOPIC" ]; then
  echo "Error: BAG_ANALYSIS_TOPIC is unset in .env file."
  exit 1
fi

if [ -z "$BAG_ANALYSIS_TIMESTAMP" ]; then
  python3 bag_exporter.py --project $PROJECT_NAME --bag $BAG_ANALYSIS_BAG_NAME --topic $BAG_ANALYSIS_TOPIC
else
  python3 bag_exporter.py --project $PROJECT_NAME --bag $BAG_ANALYSIS_BAG_NAME --timestamp $BAG_ANALYSIS_TIMESTAMP --topic $BAG_ANALYSIS_TOPIC
fi
