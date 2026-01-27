# ============================================================================
# Session state (filled by EEG bridge or EEG simulator)
# ============================================================================
bool is_session_start           # First sample of session
bool is_session_end             # Last sample of session

# ============================================================================
# Core data (filled by EEG bridge or EEG simulator)
# ============================================================================
float64[] eeg                   # Length = num_eeg_channels, in microvolts
float64[] emg                   # Length = num_emg_channels, in microvolts
float64 time                    # Sample time, seconds since recording start
uint64 sample_index             # Sample count (0-based)

# ============================================================================
# Hardware triggers/events (filled by EEG bridge or EEG simulator)
# ============================================================================
bool pulse_trigger
bool latency_trigger

# ============================================================================
# Experiment state (filled by ExperimentCoordinator)
# ============================================================================
bool in_rest                    # Protocol-defined rest period
bool paused                     # User-initiated pause
float64 experiment_time         # Pause-adjusted experiment time (seconds)
string stage_name               # Current stage name from protocol
uint32 stage_index              # Current stage index (0-based)
uint32 trial                    # Current trial within stage (0-based)
uint64 pulse_count              # Total pulses delivered

# ============================================================================
# Preprocessing (filled by Preprocessor if enabled)
# ============================================================================
bool valid                     # Sample quality check passed
float64 preprocessor_duration  # Time spent in preprocessing (seconds)

# ============================================================================
# Metadata
# ============================================================================
uint64 system_time_data_source_published
