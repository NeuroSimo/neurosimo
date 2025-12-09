# ============================================================================
# Core data (filled by EEG Bridge)
# ============================================================================
float64[] eeg                   # Length = num_eeg_channels, in microvolts
float64[] emg                   # Length = num_emg_channels, in microvolts
float64 time                    # Sample time, seconds since recording start
uint64 sample_index             # Sample count (0-based)

# ============================================================================
# Hardware triggers/events (filled by EEG Bridge)
# ============================================================================
bool pulse_delivered

# ============================================================================
# Experiment state (filled by ExperimentCoordinator)
# ============================================================================
bool in_rest                    # Protocol-defined rest period
bool paused                     # User-initiated pause
float64 experiment_time         # Pause-adjusted experiment time (seconds)
string current_stage_name       # Current stage name from protocol
uint32 current_trial            # Current trial number within stage (0-based)
uint32 pulse_count              # Total number of pulses given

# ============================================================================
# Preprocessing (filled by Preprocessor if enabled)
# ============================================================================
bool valid                      # Sample quality check passed

# ============================================================================
# Metadata
# ============================================================================
SampleMetadata metadata
