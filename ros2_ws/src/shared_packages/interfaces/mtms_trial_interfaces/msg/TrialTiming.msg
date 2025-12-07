# The time when the trial is desired to start. If the trial cannot start at this time, it will be scheduled as soon as
# possible if allow_late is true, otherwise the trial will fail.
float64 desired_start_time

# If true, allow performing the trial at any time after 'desired_start_time'. If false and it is past 'desired_start_time',
# fail the trial.
bool allow_late
