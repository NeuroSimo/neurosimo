FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    libxxhash-dev \
    ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces and utils
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY src/realtime src/realtime/
COPY src/filesystem_utils src/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS nodes
COPY src/common src/common/
COPY src/preprocessor src/preprocessor/
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select preprocessor

# Copy entrypoint

WORKDIR /app

COPY src/preprocessor/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
