#!/bin/bash

# Run CUDA test script.
if [ "$USE_GPU" = "true" ]; then
    echo "GPU is enabled, testing CUDA..."
    python3 /app/src/decider/test_cuda.py

    success=$?
    if [ $success -ne 0 ]; then
        echo "CUDA test failed, GPU not available, exiting..."
        exit 1
    fi
fi

set -e

source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash

# The default number of threads used by OpenBLAS is the number of cores; multi-threaded reductions by
# BLAS/OpenMP are non-deterministic, so setting it to 1 improves determinism, and also seems to make
# NumPy operations faster by avoiding thread overhead.
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export OMP_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1

while true; do
    ros2 launch decider decider.launch.py log-level:="$ROS_LOG_LEVEL"
    echo "Decider exited, restarting..."
done
