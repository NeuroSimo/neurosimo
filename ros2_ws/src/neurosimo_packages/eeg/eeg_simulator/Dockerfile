FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    nlohmann-json3-dev \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY ros2_ws/src/shared_packages/resources/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/neurosimo_packages/interfaces/project_interfaces src/interfaces/project_interfaces/

COPY ros2_ws/src/shared_packages/interfaces/eeg_msgs src/shared_packages/interfaces/eeg_msgs/
COPY ros2_ws/src/shared_packages/interfaces/system_interfaces src/shared_packages/interfaces/system_interfaces/
COPY ros2_ws/src/shared_packages/realtime_utils src/shared_packages/realtime_utils/

RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/neurosimo_packages/eeg/eeg_simulator src/eeg/eeg_simulator/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select eeg_simulator

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/neurosimo_packages/eeg/eeg_simulator/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
