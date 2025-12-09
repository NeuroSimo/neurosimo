FROM nvidia/cuda:12.2.0-base-ubuntu22.04

SHELL ["/bin/bash", "-c"]

# Set up ROS2 repository.
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Set non-interactive mode to avoid prompts during ROS2 installation.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-iron-ros-base \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    ros-iron-rmw-cyclonedds-cpp \
    build-essential \
    dos2unix \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up CUDA environment variables.
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Upgrade pip.
RUN pip3 install --upgrade pip

# Install PyTorch.
RUN pip3 install torch torchvision torchaudio

# Install other Python packages.
RUN pip3 install matplotlib wandb pyyaml omegaconf tqdm diffusers accelerate lovely-tensors \
    numpy scipy ema-pytorch scikit-learn mne mne-realtime mne_icalabel pandas py_neuromodulation statsmodels mneflow torchinfo joblib accelerate einops
RUN pip3 install spectrum

# Set the Cyclone DDS configuration
COPY ros2_ws/config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/
COPY ros2_ws/src/interfaces/project_interfaces src/interfaces/project_interfaces/

COPY ros2_ws/src/interfaces/eeg_msgs src/interfaces/eeg_msgs/
COPY ros2_ws/src/interfaces/system_interfaces src/interfaces/system_interfaces/
COPY ros2_ws/src/realtime_utils src/realtime_utils/

RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/pipeline/decider src/pipeline/decider/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select decider

# Copy CUDA test script
COPY ros2_ws/src/pipeline/decider/test_cuda.py /app

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/pipeline/decider/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
