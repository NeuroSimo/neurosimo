# Timing reference
float64 reference_eeg_device_timestamp

# Metadata
uint8[16] session_id
uint64 sequence_number

# Pulses, with times relative to reference_eeg_device_timestamp
TargetedPulse[] pulses
