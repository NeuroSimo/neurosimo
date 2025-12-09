FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    libgtk-3-dev \
    libsdl2-dev \
    freeglut3-dev \
    libnotify-dev \
    libsm-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libjpeg-dev \
    libtiff-dev \
    libsdl1.2-dev \
    libwebkit2gtk-4.0-dev \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# PsychoPy and its dependencies
ENV DEBIAN_FRONTEND=noninteractive

# Keep setuptools <= 66.1.1 for compatibility with PsychoPy
RUN pip3 install --no-cache-dir --upgrade pip wheel "setuptools<=66.1.1" "packaging<24"

RUN apt-get update && \
    apt-get install -y libsdl2-2.0-0 libcairo2-dev libpango1.0-dev libglib2.0-dev

RUN pip3 install --no-cache-dir psychopy==2025.1.1

RUN pip3 install --no-cache-dir attrdict3 numpy==1.26.4 scipy==1.8.0

# Set the Cyclone DDS configuration
COPY ros2_ws/config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages
WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/
COPY ros2_ws/src/interfaces/project_interfaces src/interfaces/project_interfaces/
COPY ros2_ws/src/interfaces/system_interfaces src/interfaces/system_interfaces/
COPY ros2_ws/src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY ros2_ws/src/realtime_utils src/realtime_utils/
RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/pipeline/presenter src/pipeline/presenter/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select presenter

# Copy entrypoint
WORKDIR /app
COPY ros2_ws/src/pipeline/presenter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
