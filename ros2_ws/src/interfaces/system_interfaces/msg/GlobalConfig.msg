# Global system configuration
string active_project

# EEG Configuration
int32 eeg_port
string eeg_device
int32 turbolink_sampling_frequency
int32 turbolink_eeg_channel_count
int32 maximum_dropped_samples

# LabJack Configuration
bool simulate_labjack

# Safety Configuration
float64 minimum_intertrial_interval
float64 maximum_loopback_latency
float64 maximum_timing_error

# Disk Space Monitoring Configuration
string disk_warning_threshold
string disk_error_threshold
