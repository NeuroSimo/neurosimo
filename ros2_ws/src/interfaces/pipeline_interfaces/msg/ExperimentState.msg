# Current stage
string stage_name
uint32 stage_index
uint32 total_stages

# Trial progress
uint32 trial
uint32 total_trials_in_stage

# State flags
bool ongoing
bool in_rest
bool paused

# Time tracking
float64 experiment_time        # Pause-adjusted
float64 stage_start_time       # When current stage started
float64 stage_elapsed_time     # Time in current stage

# Rest info (if in_rest)
float64 rest_duration          # Total rest duration
float64 rest_elapsed           # Time spent in rest
float64 rest_remaining         # Time remaining in rest

# Next stage preview
string next_stage_name
bool next_is_rest
