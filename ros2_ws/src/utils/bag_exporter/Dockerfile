FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY ros2_ws/src/resources/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace

WORKDIR /app
COPY ros2_ws/src/interfaces ros2_ws/src/interfaces/

# Build ROS packages

WORKDIR /app/ros2_ws
RUN source /opt/ros/iron/setup.bash && colcon build

# Copy scripts

WORKDIR /app
COPY ros2_ws/src/utils/bag_exporter/bag_exporter.py .

# Copy entrypoint

WORKDIR /app
COPY ros2_ws/src/utils/bag_exporter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
