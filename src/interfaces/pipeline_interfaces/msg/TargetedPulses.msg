# Metadata for a sequence of targeted pulses.
uint8[16] session_id
float64 reference_sample_time
uint64 sequence_number

# Sequence of targeted pulses.
TargetedPulse[] pulses
