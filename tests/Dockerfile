FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pytest \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/neurosimo_tests

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy interfaces
COPY interfaces/eeg_interfaces /opt/neurosimo_tests/interfaces/eeg_interfaces
COPY interfaces/pipeline_interfaces /opt/neurosimo_tests/interfaces/pipeline_interfaces
COPY interfaces/project_interfaces /opt/neurosimo_tests/interfaces/project_interfaces
COPY interfaces/system_interfaces /opt/neurosimo_tests/interfaces/system_interfaces

# Build interfaces
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select eeg_interfaces pipeline_interfaces project_interfaces system_interfaces

# Copy tests
COPY tests /opt/neurosimo_tests/tests
RUN chmod +x /opt/neurosimo_tests/tests/helpers/create-test-project /opt/neurosimo_tests/tests/run-fingerprint-tests.sh

ENTRYPOINT ["/opt/neurosimo_tests/tests/run-fingerprint-tests.sh"]
