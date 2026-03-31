#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

# Embedded Python is linked against the Python 3.11 venv at /opt/py311.
# Do not set PYTHONHOME here: with an embedded interpreter it can break stdlib
# discovery and cause startup failures (for example missing 'encodings').
unset PYTHONHOME

# Make packages installed into the Python 3.11 venv importable at runtime.
# Keep the venv site-packages first so the embedded 3.11 interpreter finds
# PsychoPy and our cpp_bindings module.
export PYTHONPATH=/opt/py311/lib/python3.11/site-packages${PYTHONPATH:+:$PYTHONPATH}

while true; do
    ros2 launch presenter presenter.launch.py log-level:="$ROS_LOG_LEVEL"
    echo "Presenter exited, restarting..."
done
