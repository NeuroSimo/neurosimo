mep_interfaces/Mep mep

# The actual start time of the trial; because trial can be delayed due to charging or
# discharging taking time, it is not necessarily the same as the time when the trial
# was requested to be performed.
float64 actual_start_time

# The earliest time when the trial could have been performed, measured as the
# internal time of the mTMS device right after the device had been requested to
# perform the pulse(s) of the trial.
float64 earliest_start_time
