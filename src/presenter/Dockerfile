FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
    dos2unix \
    python3.11 \
    python3.11-venv \
    python3.11-dev \
    python3-pip \
    python3-colcon-common-extensions \
    libgtk-3-dev \
    libsdl2-dev \
    freeglut3-dev \
    libnotify-dev \
    libsm-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libjpeg-dev \
    libtiff-dev \
    libwebkit2gtk-4.1-dev \
    ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# PsychoPy and its dependencies
ENV DEBIAN_FRONTEND=noninteractive

## XXX: No do install psychopy for now, as version 2026.1.1 (the most recent version as of Feb 2026) is not compatible with Python 3.12.

# RUN pip install --no-cache-dir --upgrade pip wheel "setuptools<=66.1.1" "packaging<24"

# RUN apt-get update && \
#     apt-get install -y libsdl2-2.0-0 libcairo2-dev libpango1.0-dev libglib2.0-dev

# RUN pip install --no-cache-dir psychopy==2026.1.1
# RUN pip install --no-cache-dir attrdict3 numpy==1.26.4 scipy==1.12.0

# Copy ROS workspace and build packages
WORKDIR /app

# ROS interfaces
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY src/realtime src/realtime/
COPY src/filesystem_utils src/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/common src/common/
COPY src/presenter src/presenter/

RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select presenter

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy entrypoint
WORKDIR /app
COPY src/presenter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh
ENTRYPOINT ["./ros_entrypoint.sh"]
