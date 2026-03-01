FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    python3-tornado \
    python3-bson \
    python3-cbor2 \
    python3-pil \
    ros-jazzy-ament-cmake-mypy \
    ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app

# Copy shared ROS interfaces
COPY src/interfaces src/interfaces

# Copy project-specific interfaces
ARG INTERFACES_PATH
COPY ${INTERFACES_PATH} src/interfaces

RUN source /opt/ros/jazzy/setup.bash && colcon build

# Copy the rosbridge node
COPY src/utils/rosbridge/rosbridge_suite src/utils/rosbridge/rosbridge_suite

# Change line endings for rosbridge files
RUN find src/utils/rosbridge -type f -exec dos2unix {} +

# Build the rosbridge node after building interfaces
RUN source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to rosbridge_server

# Copy ROS entrypoint
COPY src/utils/rosbridge/ros_entrypoint.sh .
RUN chmod +x ros_entrypoint.sh

# Define entrypoint

ENTRYPOINT ["./ros_entrypoint.sh"]
