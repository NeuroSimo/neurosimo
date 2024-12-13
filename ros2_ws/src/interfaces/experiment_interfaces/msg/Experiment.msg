ExperimentMetadata metadata

Trial[] trials
IntertrialInterval intertrial_interval

bool randomize_trials

# If true, wait for pedal press before performing each trial in
# the experiment. In that case, ignore 'intertrial_interval' field.
bool wait_for_pedal_press

bool autopause
uint16 autopause_interval
