FROM nvidia/cuda:12.6.0-base-ubuntu24.04

SHELL ["/bin/bash", "-c"]

# Install ROS2.
ARG ROS_APT_SOURCE_VERSION=1.1.0

RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-ros-base && \
    rm -rf /var/lib/apt/lists/*

# Install other dependencies.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    libxxhash-dev \
    build-essential \
    dos2unix \
    wget \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libxrandr2 \
    v4l-utils \
    libasound2t64 \
    libasound2-dev \
    portaudio19-dev \
    libportaudiocpp0 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Set up CUDA environment variables.
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Python dependencies
WORKDIR /app

COPY src/decider/requirements/torch.txt requirements/torch.txt
RUN pip3 install --timeout 900 --retries 3 -r requirements/torch.txt

COPY src/decider/requirements/science.txt requirements/science.txt
# --ignore-installed is needed because python3-pip (apt) ships a Debian-managed
# numpy without a pip RECORD file, which prevents pip from upgrading it normally.
RUN pip3 install --ignore-installed -r requirements/science.txt

COPY src/decider/requirements/mne.txt requirements/mne.txt
RUN pip3 install -r requirements/mne.txt

# --no-deps is required: mneflow pins mne<=1.7 (would downgrade our mne==1.12.0),
# and py_neuromodulation forces scipy to backtrack to a version with no prebuilt
# wheel, causing a source build that fails without a Fortran compiler.
# All runtime requirements are satisfied by previously installed packages.
RUN pip3 install --no-deps mneflow==0.5.10 py_neuromodulation==0.1.7

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
