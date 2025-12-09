# The decision made by the decider, either to stimulate or not to stimulate.
bool stimulate

# True if timing constraints are met, false otherwise.
bool feasible

# The time of the sample based on which the stimulation decision was made.
float64 decision_time

float64 decider_latency
float64 preprocessor_latency

float64 total_latency
