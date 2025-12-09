# Shared packages

Includes ROS 2 packages shared across NeuroSimo projects.

## Packages

- `interfaces`: The shared interfaces, including:

    - `eeg_msgs`: Messages for the EEG data, such as raw sample data and information about the EEG stream.
    - `system_interfaces`: Session and healthcheck messages and services (e.g., start and stop session).

- `realtime_utils`: Real-time utilities for ROS 2, including methods for setting the priority and optimizing memory allocation
    for real-time tasks.

## Usage

Not intended for direct use. Instead, this package is included as a submodule in NeuroSimo projects.

## License

This repository is licensed under the GPL v3 License - see the [LICENSE](LICENSE) file for details.
