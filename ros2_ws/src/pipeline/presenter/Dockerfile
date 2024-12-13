FROM osrf/ros:iron-desktop@sha256:eddad5f79aa7e888af29793c11f0ee44280c656b74dba0d37b2f48a0aa57352e

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    python3-pip \
    libgtk-3-dev \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# PsychoPy and its dependencies

# Note: Needed because one of PsychoPy's dependencies asks for keyboard input.
ENV DEBIAN_FRONTEND=noninteractive

# XXX: For now, install required numpy and scipy versions manually. This is due
#   to PsychoPy installation breaking when using NumPy 2.0.0, which is automatically
#   installed with PsychoPy. This is a temporary workaround until PsychoPy is updated
#   to properly deal with NumPy 2.0.0; after that, install PsychoPy normally
#   (i.e., `pip3 install psychopy==2024.1.3`).
RUN pip3 install attrdict3 numpy==1.26.4 scipy==1.8.0 psychopy==2024.1.3

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace and build packages

WORKDIR /app/ros2_ws

# ROS interfaces
COPY ros2_ws/src/interfaces/eeg_interfaces src/interfaces/eeg_interfaces/
COPY ros2_ws/src/interfaces/event_interfaces src/interfaces/event_interfaces/
COPY ros2_ws/src/interfaces/pipeline_interfaces src/interfaces/pipeline_interfaces/
COPY ros2_ws/src/interfaces/project_interfaces src/interfaces/project_interfaces/
COPY ros2_ws/src/interfaces/system_interfaces src/interfaces/system_interfaces/
RUN source /opt/ros/iron/setup.bash && colcon build

# ROS node
COPY ros2_ws/src/pipeline/presenter src/pipeline/presenter/
RUN source /opt/ros/iron/setup.bash && colcon build --symlink-install --packages-select presenter

# Copy entrypoint

WORKDIR /app

COPY ros2_ws/src/pipeline/presenter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
