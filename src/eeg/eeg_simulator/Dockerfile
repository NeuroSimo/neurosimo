FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    nlohmann-json3-dev \
    libxxhash-dev \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY src/interfaces/project_interfaces src/interfaces/project_interfaces/

COPY src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY src/interfaces/system_interfaces src/interfaces/system_interfaces/
COPY src/utils/realtime src/utils/realtime/

RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY src/eeg/eeg_simulator src/eeg/eeg_simulator/
RUN source /opt/ros/iron/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select eeg_simulator

# Copy entrypoint

WORKDIR /app

COPY src/eeg/eeg_simulator/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
