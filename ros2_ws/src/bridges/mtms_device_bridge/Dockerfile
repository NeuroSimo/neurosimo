FROM osrf/ros:iron-desktop@sha256:eddad5f79aa7e888af29793c11f0ee44280c656b74dba0d37b2f48a0aa57352e

SHELL ["/bin/bash", "-c"]

# Install dependencies

RUN apt-get update && \
    apt-get install -y \
    dos2unix \
    wget \
    unzip \
    ros-iron-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /build

# FPGA dependencies, instructions from https://www.ni.com/fi-fi/support/documentation/supplemental/18/downloading-and-installing-ni-driver-software-on-linux-desktop.html
RUN wget https://download.ni.com/support/softlib/MasterRepository/LinuxDrivers2022Q2/NILinux2022Q2DeviceDrivers.zip
RUN unzip NILinux2022Q2DeviceDrivers.zip

RUN apt-get dist-upgrade -y

# You MIGHT need to run this command on your host PC
# touch /usr/src/linux-headers-$(uname -r)/include/config/modversions.h

RUN apt-get install /build/NILinux2022Q2DeviceDrivers/ni-ubuntu2004firstlook-drivers-2022Q2.deb
RUN apt-get update
RUN apt-get install ni-rseries ni-fpga-interface -y
RUN dkms autoinstall


RUN apt-get update

RUN apt-get install -y \
    dos2unix

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy ROS workspace

WORKDIR /app
COPY ros2_ws/src/bridges/mtms_device_bridge ros2_ws/src/bridges/mtms_device_bridge/
COPY ros2_ws/src/interfaces/mtms_device_interfaces ros2_ws/src/interfaces/mtms_device_interfaces/
COPY ros2_ws/src/interfaces/event_interfaces ros2_ws/src/interfaces/event_interfaces/
COPY ros2_ws/src/interfaces/system_interfaces ros2_ws/src/interfaces/system_interfaces/

# Build ROS packages

WORKDIR /app/ros2_ws
RUN source /opt/ros/iron/setup.bash && \
    colcon build

# Copy entrypoint

COPY ros2_ws/src/bridges/mtms_device_bridge/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

COPY bitfiles/ /app/ros2_ws/bitfiles/

ENV BITFILE_DIRECTORY=/app/ros2_ws/bitfiles/

ENTRYPOINT ["./ros_entrypoint.sh"]
