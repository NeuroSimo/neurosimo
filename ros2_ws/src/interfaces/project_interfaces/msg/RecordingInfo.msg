# Recording metadata filename (e.g., "2024-01-15_12-30-45_subject01.json")
string json_filename

# Stream information
uint8 num_eeg_channels
uint8 num_emg_channels
uint16 sampling_frequency

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
string protocol_filename
string data_source
string simulator_dataset_filename
float64 simulator_start_time
string replay_bag_id
bool replay_play_preprocessed

# Timing
string start_time
string end_time
float64 duration

# Software provenance
string git_commit
string git_state
string version

# Fingerprints
uint64 data_source_fingerprint
uint64 preprocessor_fingerprint
uint64 decision_fingerprint

# Export status
bool exported
string export_directory
