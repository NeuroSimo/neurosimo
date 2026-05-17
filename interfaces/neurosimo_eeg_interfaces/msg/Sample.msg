# ============================================================================
# Session state (filled by EEG bridge or EEG simulator)
# ============================================================================
bool is_session_start           # First sample of session

# ============================================================================
# Core data (filled by EEG bridge or EEG simulator)
# ============================================================================
float64[] eeg                   # Length = num_eeg_channels, in microvolts
float64[] emg                   # Length = num_emg_channels, in microvolts
float64 time                    # Sample time, seconds since recording start
float64 eeg_device_timestamp    # Raw timestamp from EEG device, seconds
uint64 sample_index             # Sample count (0-based)

# ============================================================================
# Hardware triggers (filled by EEG bridge or EEG simulator)
# ============================================================================
bool pulse_trigger
bool loopback_trigger

# ============================================================================
# Simulated events (filled by EEG simulator)
# ============================================================================
bool event_trigger

# ============================================================================
# Experiment state (filled by ExperimentCoordinator)
# ============================================================================
bool in_rest                    # Protocol-defined rest period
bool in_task                    # Protocol-defined task period
bool paused                     # User-initiated pause
float64 experiment_time         # Pause-adjusted experiment time (seconds)
string stage_name               # Current stage name from protocol
uint32 stage_index              # Current stage index (0-based)
uint32 trial_in_stage           # Current trial slot within stage (0-based), advances on success
uint64 trial_in_session         # Total successful trials in session
uint64 attempt_in_session       # Total attempts in session (advances every trigger)
bool is_new_stage               # True on the first sample of each stage
bool is_attempt_start           # Attempt boundary: fires on session start, after a pulse, or after rest ends

# ============================================================================
# Preprocessing (filled by Preprocessor if enabled)
# ============================================================================
bool preprocessing_failed      # Preprocessing step failed for this sample
bool valid                     # Sample quality check passed
float64 preprocessor_duration  # Time spent in preprocessing (seconds)

# ============================================================================
# Metadata
# ============================================================================
uint64 system_time_data_source_published
uint64 system_time_experiment_coordinator_published
uint64 system_time_preprocessor_published
