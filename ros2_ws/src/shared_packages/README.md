# Shared packages

Includes ROS 2 packages that are shared between NeuroSimo and mTMS projects.

## Packages

- `interfaces`: The shared interfaces, including:

    - `eeg_msgs`: Messages for the EEG data, such as raw sample data and information about the EEG stream.
    - `mep_interfaces`: Messages, services, and actions for analyzing MEP based on the EEG data.
    - `system_interfaces`: Session and healthcheck messages and services (e.g., start and stop session).
    - `targeting_msgs`: Messages for electric targeting, including ElectricTarget, which defines x- and y-coordinates of the
      target, rotation angle, intensity, and targeting algorithm.
    - `mtms_trial_interfaces`: Messages and actions for performing a trial with mTMS, including the trial configuration and
      trial result.

- `realtime_utils`: Real-time utilities for ROS 2, including methods for setting the priority and optimizing memory allocation
    for real-time tasks.

## Usage

Not intended for direct use. Instead, this package is included as a submodule in the NeuroSimo and mTMS projects.

## License

This repository is licensed under the GPL v3 License - see the [LICENSE](LICENSE) file for details.
