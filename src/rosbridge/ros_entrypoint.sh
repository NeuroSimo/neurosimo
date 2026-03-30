#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

TOPICS_GLOB="\"[\
'/neurosimo/*/heartbeat',\
'/neurosimo/*/health',\
'/neurosimo/eeg_bridge/state',\
'/neurosimo/eeg_simulator/state',\
'/neurosimo/eeg_simulator/dataset/list',\
'/neurosimo/session/state',\
'/neurosimo/session_exporter/state',\
'/neurosimo/session_configurator/config',\
'/neurosimo/global_configurator/config',\
'/neurosimo/pipeline/*',\
'/neurosimo/experiment/protocol/list',\
'/neurosimo/recording/recordings/list',\
'/neurosimo/system/disk_status',\
'/neurosimo/eeg_device/info',\
'/neurosimo/eeg/statistics',\
'/eeg_bridge/dropped_samples',\
'/rosout'\
]\""

SERVICES_GLOB="\"[\
'/neurosimo/session_configurator/set_parameters',\
'/neurosimo/session_configurator/get_parameters',\
'/neurosimo/global_configurator/set_parameters',\
'/neurosimo/global_configurator/get_parameters',\
'/neurosimo/projects/list',\
'/neurosimo/projects/create',\
'/neurosimo/session/start',\
'/neurosimo/session/abort',\
'/neurosimo/session/export',\
'/neurosimo/recording_manager/recording/get_info',\
'/neurosimo/recording_manager/recording/delete',\
'/neurosimo/eeg_simulator/dataset/get_info',\
'/neurosimo/experiment/pause',\
'/neurosimo/experiment/resume',\
'/neurosimo/experiment_coordinator/protocol/get_info',\
'/rosapi/*'\
]\""

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=$ROSBRIDGE_PORT \
  namespace:=neurosimo \
  topics_glob:="$TOPICS_GLOB" \
  services_glob:="$SERVICES_GLOB" \
  params_glob:="[]"
