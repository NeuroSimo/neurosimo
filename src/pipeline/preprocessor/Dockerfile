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
COPY src/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/
COPY src/interfaces/project_interfaces src/interfaces/project_interfaces/
COPY src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY src/interfaces/system_interfaces src/interfaces/system_interfaces/
COPY src/utils/realtime src/utils/realtime/
COPY src/utils/filesystem_utils src/utils/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS nodes
COPY src/pipeline/common src/pipeline/common/
COPY src/pipeline/preprocessor src/pipeline/preprocessor/
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select preprocessor

# Copy entrypoint

WORKDIR /app

COPY src/pipeline/preprocessor/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
