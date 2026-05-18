# Current stage
string stage_name
uint32 stage_index
uint32 total_stages

# Trial progress
uint32 trial_in_stage
uint32 total_trials_in_stage
uint64 attempt_in_session
uint32 attempt_in_trial
uint32 failures_in_stage       # Failed trials in current stage (retry count)
uint32 max_failures            # 0 means no retry limit set

# State flags
bool ongoing
bool in_rest
bool in_task
bool paused

# Task info (if in_task)
string task_name

# Time tracking
float64 experiment_time        # Pause-adjusted
float64 session_time           # Seconds since session start
float64 stage_start_time       # When current stage started
float64 stage_elapsed_time     # Time in current stage

# Rest info (if in_rest)
float64 rest_duration          # Total rest duration
float64 rest_elapsed           # Time spent in rest
float64 rest_remaining         # Time remaining in rest

# Next stage preview
string next_stage_name
bool next_is_rest

# Current protocol element ("step"): index in elements[], total count, kind (0=stage, 1=rest, 2=task)
uint32 step_index
uint32 total_steps
uint8 step_type
string step_label
