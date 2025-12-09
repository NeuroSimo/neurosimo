# Hardware configuration
uint16 sampling_frequency
uint8 num_eeg_channels
uint8 num_emg_channels

# Session info
bool is_simulation
float64 start_time              # Unix timestamp when recording started

# Pipeline configuration
bool passed_experiment_coordinator
bool passed_preprocessor
