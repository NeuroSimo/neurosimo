# Presenter Module Documentation

## Overview

The Presenter module handles real-time presentation of sensory stimuli (visual, auditory, tactile) during EEG/EMG experiments. It receives stimulus commands from the decider and renders them using PsychoPy.

## Class Methods

### `__init__()`

Initializes the presenter and sets up display/audio hardware.

**Common initialization tasks:**
- Create display window
- Configure audio output
- Set up hardware connections (tactile stimulators, etc.)
- Initialize stimulus resources

### `__del__()`

Cleanup method called when presenter is destroyed. Should properly close hardware connections and windows.

### `process(stimulus_type, parameters)`

Main method called by the pipeline when a sensory stimulus needs to be presented.

**Parameters:**

#### `stimulus_type` (str)
The type of stimulus, as specified in the decider's sensory stimulus definition. Examples:
- `'visual_cue'`
- `'auditory_tone'`  
- `'text_message'`
- `'tactile_buzz'`

#### `parameters` (dict)
Dictionary containing stimulus-specific parameters. All values from the decider are automatically parsed:
- Numeric values become `int` or `float` when possible
- Other values remain as strings
- All keys from the original stimulus definition are preserved

**Return Value:**
- `True`: Stimulus was successfully processed and presented
- `False`: Error occurred (logged by pipeline but doesn't stop execution)

## Integration with Decider

### Static Stimuli (Pre-scheduled)

Defined in the decider's `get_configuration()` method:

```python
def get_configuration(self):
    return {
        # ... other config ...
        'sensory_stimuli': [
            {
                'time': 5.0,  # 5 seconds after session start
                'type': 'visual_cue',
                'parameters': {
                    'color': 'red',
                    'size': 100,
                    'duration': 0.5,
                    'position_x': 0,
                    'position_y': 0
                }
            }
        ]
    }
```

### Dynamic Stimuli (Real-time)

Generated in the decider's `process()` method based on EEG/EMG analysis:

```python
def process(self, current_time, ...):
    # Analyze EEG data...
    
    return {
        'sensory_stimuli': [
            {
                'time': current_time + 1.0,  # 1 second from now
                'type': 'feedback_tone',
                'parameters': {
                    'frequency': calculated_frequency,
                    'duration': 0.2,
                    'volume': 0.7
                }
            }
        ]
    }
```

## Best Practices

1. **Pre-load resources** (images, sounds) during initialization
2. **Reuse stimulus objects** when possible rather than creating new ones
3. **Always return boolean status** from `process()` method
4. **Validate stimulus parameters** before rendering
5. **Log stimulus timing** for later analysis
6. **Initialize hardware connections** in `__init__()`
7. **Clean up resources** properly in `__del__()`

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
