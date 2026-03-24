FROM nvidia/cuda:12.6.0-base-ubuntu24.04

SHELL ["/bin/bash", "-c"]

# Set up ROS2 repository.
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Set non-interactive mode to avoid prompts during ROS2 installation.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    libxxhash-dev \
    build-essential \
    dos2unix \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up CUDA environment variables.
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install PyTorch.
RUN pip3 install --timeout 300 --retries 3 torch==2.10.0 torchvision==0.25.0 torchaudio==2.10.0

# Core scientific packages
RUN pip3 install matplotlib==3.10.8 wandb==0.25.0 pyyaml==6.0.1 omegaconf==2.3.0 tqdm==4.67.3 diffusers==0.36.0 accelerate==1.12.0 lovely-tensors==0.1.21 \
    numpy==1.26.4 scipy==1.17.1 ema-pytorch==0.7.9 scikit-learn==1.8.0 pandas==3.0.1 statsmodels==0.14.6 torchinfo==1.8.0 joblib==1.5.3 einops==0.8.2

# MNE ecosystem
RUN pip3 install mne==1.7.0 mne-realtime==0.3.0 mne_icalabel==0.8.1

# mneflow (pins tensorflow, install separately to avoid conflicts)
RUN pip3 install mneflow==0.5.10

# py_neuromodulation (heavy deps, install last)
RUN pip3 install py_neuromodulation==0.1.7 --no-deps || pip3 install py_neuromodulation==0.1.7

RUN pip3 install spectrum==0.9.0

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY interfaces/shared_stimulation_interfaces interfaces/shared_stimulation_interfaces/
COPY src/realtime src/realtime/
COPY src/filesystem_utils src/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/common src/common/
COPY src/decider src/decider/
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select decider

# Copy CUDA test script
COPY src/decider/test_cuda.py /app

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy entrypoint

WORKDIR /app

COPY src/decider/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
