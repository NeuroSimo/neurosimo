# Log message from pipeline components (decider, preprocessor, presenter)

# The actual log message content
string message

# The sample time at which this log was generated (in seconds)
# Set to 0.0 for initialization messages
float64 sample_time

# Message severity level
uint8 level

# Level constants
uint8 LEVEL_INFO = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_ERROR = 2

# True if this message was generated during initialization, false otherwise
bool is_initialization

