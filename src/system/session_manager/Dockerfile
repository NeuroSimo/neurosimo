FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-docker && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY interfaces/project_interfaces interfaces/project_interfaces/
COPY interfaces/pipeline_interfaces interfaces/pipeline_interfaces/
COPY interfaces/eeg_interfaces interfaces/eeg_interfaces/
COPY interfaces/system_interfaces interfaces/system_interfaces/
RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/system/session_manager src/system/session_manager/
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select session_manager

# Copy entrypoint

WORKDIR /app

COPY src/system/session_manager/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
