FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    ros-iron-ament-cmake-mypy \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

RUN pip install \
    pymongo \
    tornado

# Set the Cyclone DDS configuration
COPY ros2_ws/src/resources/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# Copy shared ROS interfaces
COPY ros2_ws/src/interfaces src/interfaces

# Copy project-specific interfaces
ARG INTERFACES_PATH
COPY ${INTERFACES_PATH} src/interfaces

RUN source /opt/ros/iron/setup.bash && colcon build

# Copy the rosbridge node
COPY ros2_ws/src/rosbridge/rosbridge_suite src/rosbridge/rosbridge_suite

# Change line endings for rosbridge files
RUN find src/rosbridge -type f -exec dos2unix {} +

# Build the rosbridge node after building interfaces
RUN source /opt/ros/iron/setup.bash && colcon build --packages-up-to rosbridge_server

# Copy ROS entrypoint
COPY ros2_ws/src/rosbridge/ros_entrypoint.sh .
RUN chmod +x ros_entrypoint.sh

# XXX: Hack around the symlink issue, allows fail with || : as this isn't needed on linux
RUN cp install/rosbridge_server/lib/rosbridge_server/rosbridge_websocket.py install/rosbridge_server/lib/rosbridge_server/rosbridge_websocket || :

# Define entrypoint

ENTRYPOINT ["./ros_entrypoint.sh"]
