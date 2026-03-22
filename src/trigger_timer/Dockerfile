FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    wget \
    tar \
    libusb-1.0-0-dev \
    ros-jazzy-rmw-cyclonedds-cpp && \
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

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/

COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY src/realtime src/realtime/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/trigger_timer src/trigger_timer/
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select trigger_timer

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy entrypoint

WORKDIR /app

COPY src/trigger_timer/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
