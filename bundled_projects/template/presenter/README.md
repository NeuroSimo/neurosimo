# Presenter Module Documentation

## Overview

The Presenter module handles real-time presentation of sensory stimuli (visual, auditory, tactile) during EEG/EMG experiments. It receives stimulus commands from the decider and renders them using PsychoPy.

## Class Methods

### `__init__(subject_id)`

Initializes the presenter and sets up display/audio hardware.

**Parameters:**
- `subject_id` (str): Subject identifier

**Common initialization tasks:**
- Create display window
- Configure audio output
- Set up hardware connections (tactile stimulators, etc.)
- Initialize stimulus resources

### `__del__()`

Cleanup method called when presenter is destroyed. Should properly close hardware connections and windows.

### `get_configuration()`

Called by the pipeline during initialization. Must return a dictionary with configuration parameters.

**Return dictionary keys:**

#### `stimulus_processors` (dict)
Dictionary mapping stimulus types to processor methods. Each stimulus type must have a corresponding processor.

**Format:**
```python
'stimulus_processors': {
    'visual_cue': self.process_visual_cue,
    'text_message': self.process_text_message,
}
```

When a stimulus of a given type is received, its corresponding processor method is called.

### Stimulus Processor Methods

Stimulus processor methods are called when stimuli of their registered type are received. Each processor takes a parameters dictionary and returns a boolean.

**Method signature:**
```python
def process_<stimulus_type>(self, parameters: dict[str, Any]) -> bool:
    """Process a stimulus of this type."""
    # Extract parameters
    param1 = parameters.get('param1', default_value)
    # ... process stimulus ...
    return True  # or False on error
```

**Parameters:**
- `parameters` (dict): Dictionary containing stimulus-specific parameters. All values from the decider are automatically parsed:
  - Numeric values become `int` or `float` when possible
  - Other values remain as strings
  - All keys from the original stimulus definition are preserved

**Return Value:**
- `True`: Stimulus was successfully processed and presented
- `False`: Error occurred (logged by pipeline but doesn't stop execution)

## Integration with Decider

Stimuli are sent from the decider module (either predefined or dynamically generated). The presenter receives them and routes them to the appropriate processor based on the `stimulus_type`.

### Predefined Stimuli

Defined in the decider's `get_configuration()` method:

```python
def get_configuration(self):
    return {
        # ... other config ...
        'predefined_sensory_stimuli': [
            {
                'time': 5.0,  # 5 seconds after session start
                'type': 'visual_cue',
                'parameters': {
                    'color': 'red',
                    'size': 0.3,
                    'duration': 0.5,
                    'position_x': 0,
                    'position_y': 0
                }
            }
        ]
    }
```

### Dynamic Stimuli

Generated in the decider's `process_periodic()` or event processor methods:

```python
def process_periodic(self, reference_time, ...):
    # Analyze EEG data...
    
    return {
        'sensory_stimuli': [
            {
                'time': reference_time + 1.0,  # 1 second from now
                'type': 'visual_cue',
                'parameters': {
                    'color': 'blue',
                    'size': 0.2,
                    'duration': 0.2
                }
            }
        ]
    }
```

## Best Practices

1. **Pre-load resources** (images, sounds) during initialization
2. **Reuse stimulus objects** when possible rather than creating new ones
3. **Always return boolean status** from processor methods
4. **Validate stimulus parameters** before rendering using `.get()` with defaults
5. **Log stimulus timing** for later analysis
6. **Initialize hardware connections** in `__init__()`
7. **Clean up resources** properly in `__del__()`
8. **Register all stimulus types** you plan to use in `stimulus_processors` configuration

## Example Use Cases

### Closed-loop Neurofeedback
- Decider analyzes EEG power in real-time
- Presenter shows continuous visual/auditory feedback
- Stimulus parameters updated every 100-500ms

### Event-Related Experiments  
- Static stimuli scheduled at precise times
- Triggers synchronized with EEG recording
- Post-stimulus response collection

### Brain-Computer Interface
- Real-time classification of EEG patterns
- Immediate feedback based on detected intentions
- Adaptive stimulus parameters based on performance
