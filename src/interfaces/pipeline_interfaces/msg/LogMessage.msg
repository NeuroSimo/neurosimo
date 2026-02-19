# Log message from pipeline components (decider, preprocessor, presenter)

# The actual log message content
string message

# The sample time at which this log was generated (in seconds)
# Set to 0.0 for initialization and finalization messages
float64 sample_time

# Message severity level
uint8 level

# Level constants
uint8 LEVEL_INFO = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_ERROR = 2

# Phase when this message was generated
uint8 phase

# Phase constants
uint8 PHASE_RUNTIME = 0
uint8 PHASE_INITIALIZATION = 1
uint8 PHASE_FINALIZATION = 2

