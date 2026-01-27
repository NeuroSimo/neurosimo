# Recording metadata filename (e.g., "2024-01-15_12-30-45_subject01.json")
string json_filename

# Stream information
uint8 num_eeg_channels
uint8 num_emg_channels
uint16 sampling_frequency

# Recording duration in seconds
float64 duration

# Pipeline configuration
bool preprocessor_enabled
string preprocessor_module
bool decider_enabled
string decider_module
bool presenter_enabled
string presenter_module

# Session configuration
string project_name
string subject_id
string protocol_name
string notes

# Timestamps
string start_time
string end_time

# Software provenance
string git_commit
string git_state
string version

# Export status
bool exported
string export_directory
