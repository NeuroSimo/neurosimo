# Hardware configuration
uint16 sampling_frequency
uint8 num_of_eeg_channels
uint8 num_of_emg_channels
bool is_simulation
builtin_interfaces/Time system_time

# Pipeline tracking
bool passed_experiment_coordinator
bool passed_preprocessor

# Performance metrics
float64 preprocessing_duration      # Time spent in preprocessing (seconds)
