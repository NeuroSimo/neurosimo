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

WORKDIR /app/ros2_ws
COPY ros2_ws/src/neurosimo_packages/interfaces src/interfaces/

# Build ROS packages

RUN source /opt/ros/iron/setup.bash && colcon build

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/neurosimo_packages/utils/bag_recorder/bag_recorder.sh .
RUN dos2unix bag_recorder.sh
RUN chmod +x bag_recorder.sh

ENTRYPOINT ["./bag_recorder.sh"]
