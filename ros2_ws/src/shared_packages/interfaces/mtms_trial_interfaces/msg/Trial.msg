targeting_msgs/ElectricTarget[] targets
float64[] pulse_times_since_trial_start

# Note that 'timing' field can be left undefined when performing experiments
# with the experiment performer. In that case, the experiment performer will
# automatically set the pulse times.
TrialTiming timing

TrialConfig config

TriggerConfig[] triggers

bool analyze_mep
mep_interfaces/MepConfiguration mep_config
