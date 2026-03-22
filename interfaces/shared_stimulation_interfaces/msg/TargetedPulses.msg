# SHARED CONTRACT
#
# This message is part of the shared ROS 2 interface between mTMS and NeuroSimo.
# Do not modify the file in this repository only. Any change needs to be synchronized
# with the corresponding interface definition in the other repository.

# Timing reference
float64 reference_eeg_device_timestamp

# Metadata
uint8[16] session_id
uint64 sequence_number

# Pulses, with times relative to reference_eeg_device_timestamp
TargetedPulse[] pulses
