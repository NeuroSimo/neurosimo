#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

export CYCLONEDDS_URI=file:///config/cyclonedds.xml

exec "$@"
