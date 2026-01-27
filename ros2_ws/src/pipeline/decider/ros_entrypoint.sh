#!/bin/bash

# Run CUDA test script.
if [ "$USE_GPU" = "true" ]; then
    echo "GPU is enabled, testing CUDA..."
    python3 /app/ros2_ws/src/pipeline/decider/test_cuda.py

    success=$?
    if [ $success -ne 0 ]; then
        echo "CUDA test failed, GPU not available, exiting..."
        exit 1
    fi
fi

set -e

source /opt/ros/iron/setup.bash
source /app/ros2_ws/install/setup.bash

# The default number of threads used by OpenBLAS is the number of cores; this seems to slow down
# NumPy operations, whereas setting it to a smaller value such as 1 seems to speed them up.
export OPENBLAS_NUM_THREADS=1

ros2 launch decider decider.launch.py log-level:="$ROS_LOG_LEVEL" \
    minimum-intertrial-interval:="$MINIMUM_INTERTRIAL_INTERVAL" \
    dropped-sample-threshold:="$DROPPED_SAMPLE_THRESHOLD"
