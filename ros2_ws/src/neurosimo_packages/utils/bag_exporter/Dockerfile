FROM osrf/ros:iron-desktop@sha256:eddad5f79aa7e888af29793c11f0ee44280c656b74dba0d37b2f48a0aa57352e

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY ros2_ws/src/shared_packages/resources/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace

WORKDIR /app
COPY ros2_ws/src/neurosimo_packages/interfaces ros2_ws/src/neurosimo_packages/interfaces/

# Build ROS packages

WORKDIR /app/ros2_ws
RUN source /opt/ros/iron/setup.bash && colcon build

# Copy scripts

WORKDIR /app
COPY ros2_ws/src/neurosimo_packages/utils/bag_exporter/bag_exporter.py .

# Copy entrypoint

WORKDIR /app
COPY ros2_ws/src/neurosimo_packages/utils/bag_exporter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
