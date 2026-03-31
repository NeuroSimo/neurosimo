FROM docker.io/ros:jazzy@sha256:be7350a0efd17925415a6a3a076d6009b425af7500b0a5cdb5d717e87ef46f39
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Prefer noninteractive apt and install Python 3.11 alongside Noble's default Python
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
      dos2unix \
      python3.11 \
      python3.11-dev \
      python3.11-venv \
      python3-pip \
      python3-colcon-common-extensions \
      pybind11-dev \
      cmake \
      build-essential \
      pkg-config \
      libgtk-3-dev \
      libsdl2-dev \
      freeglut3-dev \
      libnotify-dev \
      libsm-dev \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer1.0-dev \
      libjpeg-dev \
      libtiff-dev \
      libwebkit2gtk-4.1-dev \
      libcairo2-dev \
      libpango1.0-dev \
      libglib2.0-dev \
      ros-jazzy-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# PsychoPy as of 2026.1.1 doesn't support Python 3.12, so we use Python 3.11 instead.
RUN python3.11 -m venv /opt/py311 && \
    /opt/py311/bin/python -m pip install --upgrade pip setuptools wheel && \
    /opt/py311/bin/python -m pip install \
      psychopy==2026.1.1 \
      catkin_pkg==1.1.0

WORKDIR /app

# ROS interfaces / base packages
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY src/realtime src/realtime/
COPY src/filesystem_utils src/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --event-handlers console_direct+

# ROS node
COPY src/common src/common/
COPY src/presenter src/presenter/

# Forcing CMake to find Python 3.11 in the virtual environment.
RUN source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon build --symlink-install --packages-select presenter \
      --event-handlers console_direct+ \
      --cmake-force-configure \
      --cmake-args \
        -DPYBIND11_FINDPYTHON=ON \
        -DPython_EXECUTABLE=/opt/py311/bin/python \
        -DPython3_EXECUTABLE=/opt/py311/bin/python \
        -DPython3_FIND_STRATEGY=LOCATION \
        -DPython3_FIND_UNVERSIONED_NAMES=NEVER

COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

WORKDIR /app
COPY src/presenter/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh && chmod +x ros_entrypoint.sh
ENTRYPOINT ["./ros_entrypoint.sh"]