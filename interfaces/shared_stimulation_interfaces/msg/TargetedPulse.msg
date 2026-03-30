# SHARED CONTRACT
#
# This message is part of the shared ROS 2 interface between mTMS and NeuroSimo.
# Do not modify the file in this repository only. Any change needs to be synchronized
# with the corresponding interface definition in the other repository.

# Pulse time offset, relative to reference_eeg_device_timestamp, in seconds.
float64 time_offset

# Target displacement in millimeters.
float64 displacement_x
float64 displacement_y

# Coil rotation in degrees [0, 360).
float64 rotation_angle

# Stimulation intensity in V/m.
float64 intensity
