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

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/neurosimo_packages/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/

COPY ros2_ws/src/shared_packages/interfaces/eeg_msgs src/shared_packages/interfaces/eeg_msgs/
COPY ros2_ws/src/shared_packages/interfaces/system_interfaces src/shared_packages/interfaces/system_interfaces/
COPY ros2_ws/src/shared_packages/realtime_utils src/shared_packages/realtime_utils/

RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/neurosimo_packages/eeg/eeg_bridge src/eeg/eeg_bridge/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select eeg_bridge

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/neurosimo_packages/eeg/eeg_bridge/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]