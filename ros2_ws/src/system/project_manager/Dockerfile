FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    ros-iron-rmw-cyclonedds-cpp \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install watchdog

# Set the Cyclone DDS configuration
COPY ros2_ws/config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY ros2_ws/src/interfaces/project_interfaces src/interfaces/project_interfaces/
COPY ros2_ws/src/interfaces/system_interfaces src/interfaces/system_interfaces/
RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/system/project_manager src/system/project_manager/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select project_manager

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/system/project_manager/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
