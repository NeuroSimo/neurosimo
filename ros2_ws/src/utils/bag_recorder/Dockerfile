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

WORKDIR /app/ros2_ws
COPY ros2_ws/src/interfaces src/interfaces/

# Build ROS packages

RUN source /opt/ros/iron/setup.bash && colcon build

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/utils/bag_recorder/bag_recorder.sh .
RUN dos2unix bag_recorder.sh
RUN chmod +x bag_recorder.sh

ENTRYPOINT ["./bag_recorder.sh"]
