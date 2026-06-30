# Descriptor for a single protocol runtime parameter.
# Used by the UI to render an input and validate the value before a session starts.

string name        # Key used to look up the value (e.g. 'conditioning_intensity')
string label       # Human-readable label shown in the UI
string type        # Value type: 'float', 'int', 'string', or 'bool'
string unit        # Optional unit shown next to the input (e.g. '% MSO')
bool required      # Whether a value must be provided before the session can start

bool has_min       # Whether 'min' constrains the value
float64 min        # Minimum allowed value (for numeric types)

bool has_max       # Whether 'max' constrains the value
float64 max        # Maximum allowed value (for numeric types)
