FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    libxxhash-dev \
    ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY src/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/

COPY src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY src/interfaces/system_interfaces src/interfaces/system_interfaces/
COPY src/utils/realtime src/utils/realtime/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/eeg/eeg_bridge src/eeg/eeg_bridge/
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select eeg_bridge

# Copy entrypoint

WORKDIR /app

COPY src/eeg/eeg_bridge/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]