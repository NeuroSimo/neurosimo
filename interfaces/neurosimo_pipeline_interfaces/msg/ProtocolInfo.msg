string yaml_filename

string name

string description

# Execution settings
float64 minimum_trial_interval
bool repeat_failed_trials

# Array of protocol elements (stages and rests) in order
ProtocolElementInfo[] elements

# Optional runtime parameters set in the UI before the session starts
RuntimeParameterInfo[] runtime_parameters
