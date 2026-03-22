FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pytest \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/neurosimo_tests

# Copy interfaces
COPY interfaces/neurosimo_eeg_interfaces /opt/neurosimo_tests/interfaces/neurosimo_eeg_interfaces
COPY interfaces/neurosimo_pipeline_interfaces /opt/neurosimo_tests/interfaces/neurosimo_pipeline_interfaces
COPY interfaces/neurosimo_project_interfaces /opt/neurosimo_tests/interfaces/neurosimo_project_interfaces
COPY interfaces/neurosimo_system_interfaces /opt/neurosimo_tests/interfaces/neurosimo_system_interfaces

# Build interfaces
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select neurosimo_eeg_interfaces neurosimo_pipeline_interfaces neurosimo_project_interfaces neurosimo_system_interfaces

# Copy tests
COPY tests /opt/neurosimo_tests/tests
RUN chmod +x /opt/neurosimo_tests/tests/helpers/create-test-project /opt/neurosimo_tests/tests/run-fingerprint-tests.sh

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

ENTRYPOINT ["/opt/neurosimo_tests/tests/run-fingerprint-tests.sh"]
