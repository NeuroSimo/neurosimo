FROM osrf/ros:iron-desktop@sha256:c82d8b91b11291a2c86fad704534d15662ae1a8ad80a84346c54f756a48f5d40

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    wget \
    tar \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Download and install LabJack LJM
RUN wget https://files.labjack.com/installers/LJM/Linux/x64/release/labjack_ljm_software_2019_07_16_x86_64.tar.gz && \
    tar -zvxf labjack_ljm_software_2019_07_16_x86_64.tar.gz && \
    cd labjack_ljm_software_2019_07_16_x86_64 && \
    # XXX: The LJM installer fails with error code 127 when run in Docker container, possibly related to the part
    #   installing device rules. LabJack can still be accessed and used, but it doesn't work over reconnects without
    #   a workaround (mapping /dev in local Ubuntu into the /dev of the container, see docker-compose.yml for a more
    #   detailed explanation.)
    sudo bash -x ./labjack_ljm_installer.run || true && \
    cd .. && \
    rm -rf labjack_ljm_software_2019_07_16_x86_64 labjack_ljm_software_2019_07_16_x86_64.tar.gz

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
COPY ros2_ws/src/neurosimo_packages/pipeline/trigger_timer src/pipeline/trigger_timer/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select trigger_timer

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/neurosimo_packages/pipeline/trigger_timer/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
