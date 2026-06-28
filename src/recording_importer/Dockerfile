FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    python3-numpy && \
    rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN pip3 install --no-cache-dir --ignore-installed \
    mne==1.12.1

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/recording_importer src/recording_importer/
RUN source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select recording_importer

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/

COPY src/recording_importer/ros_entrypoint.sh /ros_entrypoint.sh
RUN dos2unix /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "recording_importer", "recording_importer.launch.py", "log-level:=info"]
