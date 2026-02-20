# Decider Module Documentation

## Overview

The Decider module processes real-time EEG/EMG data and makes decisions about when to trigger TMS pulses or present sensory stimuli. This documentation covers the complete API and configuration options.

## Example Deciders

The `project_template/decider/` directory contains several example decider modules:

- **`example.py`**: Basic periodic processing with event handling
- **`example_sensory_stimuli.py`**: Demonstrates both predefined and dynamic sensory stimuli
- **`phastimate.py`**: Real-time phase estimation for brain state-dependent stimulation

## Available Libraries

The following third-party libraries are currently available in the decider environment:

- numpy
- scipy
- scikit-learn
- statsmodels
- mneflow

To add more libraries, modify `src/pipeline/decider/Dockerfile` and run `build-neurosimo` from the command line.

## Class Methods

### `__init__(subject_id, num_eeg_channels, num_emg_channels, sampling_frequency)`

Initializes the decider with device configuration parameters automatically provided by the pipeline.

**Parameters:**
- `subject_id` (str): Subject identifier
- `num_eeg_channels` (int): Number of EEG channels
- `num_emg_channels` (int): Number of EMG channels  
- `sampling_frequency` (int): Sampling frequency in Hz

### `get_configuration()`

Called by the pipeline during initialization. Must return a dictionary with configuration parameters.

**Return dictionary keys:**

#### `periodic_processing_enabled` (bool)
Whether periodic processing is enabled. When `False`, the `process_periodic()` method is never called periodically (only events and EEG triggers are processed).

**Examples:**
- `True`: Enable periodic processing
- `False`: Disable periodic processing (event-driven only)

#### `periodic_processing_interval` (float, optional when disabled)
How frequently the `process_periodic()` method is called, in seconds. Required when `periodic_processing_enabled` is `True`, optional (defaults to `0.0`) when `False`.

**Examples:**
- `1.0`: Process once per second
- `0.1`: Process 10 times per second
- `0.01`: Process 100 times per second

**Validation:** When `periodic_processing_enabled` is `True`, this value must be greater than `0.0`.

#### `first_periodic_processing_at` (float, optional)
Time of the first periodic processing call in seconds (relative to session start). Defaults to `periodic_processing_interval` if not specified.

**Examples:**
- `1.0`: First processing at 1.0s, then every `periodic_processing_interval`
- `0.5`: First processing at 0.5s (useful for offset timing)

**Note:** Only relevant when `periodic_processing_enabled` is `True`. This parameter is optional and primarily exists for precise timing control. Most users should omit it and use the default behavior.

#### `sample_window` (list)
Two-element list `[earliest_seconds, latest_seconds]` defining the buffer size relative to current sample, expressed in **seconds**.
- Current sample is always at `0.0`
- Earliest time is negative or zero
- Values are in seconds; they are converted to samples using the provided sampling frequency

**Examples:**
- `[-0.005, 0.0]`: Keep last 5 ms + current
- `[-1.0, 0.0]`: Keep last second (at any sampling rate)
- `[0.0, 0.0]`: Keep only current sample
- `[-0.005, 0.005]`: Look 5 ms back and 5 ms ahead (introduces 5 ms delay)

#### `pulse_lockout_duration` (float)
Duration in seconds during which periodic processing is disabled after a pulse is delivered.
- Prevents new stimulation during the lockout period
- Events and EEG triggers are still processed during lockout
- Set to `0.0` to disable lockout

**Examples:**
- `2.0`: No periodic processing for 2 seconds after pulse
- `0.0`: No lockout (periodic processing continues immediately)

#### `predefined_events` (list, optional)
List of event times (in seconds) for scheduled processing triggers. Can be omitted if no predefined events are needed.

**Format:**
- Simple list of floats representing event times relative to session start
- When an event time is reached, the `event_processor` (if configured) is called

**Example:**
```python
'predefined_events': [5.0, 10.0, 15.0]  # Events at 5s, 10s, and 15s
```

#### `pulse_processor` (callable or dict, optional)
Processor method called when a pulse event occurs. Can be omitted if pulse events are not needed.

**Simple format:**
```python
'pulse_processor': self.process_pulse,
```

**Advanced format with custom sample window:**
```python
'pulse_processor': {
    'processor': self.process_pulse,
    'sample_window': [-0.500, 0.100],  # Custom window just for pulse events (seconds)
}
```

When a pulse event occurs, the pulse processor is called instead of the regular `process_periodic()` method.

**Example processor method:**
```python
def process_pulse(
        self, reference_time, reference_index, time_offsets, 
        eeg_buffer, emg_buffer, is_coil_at_target):
    """Process pulse events."""
    print(f"Pulse event at {reference_time}")
    # Process pulse-specific logic
    return None
```

#### `event_processor` (callable or dict, optional)
Processor method called when a general event occurs (from `predefined_events`). Can be omitted if events are not needed.

**Simple format:**
```python
'event_processor': self.process_event,
```

**Advanced format with custom sample window:**
```python
'event_processor': {
    'processor': self.process_event,
    'sample_window': [-1.5, 0.3],  # Custom window for events (seconds)
}
```

When an event occurs, the event processor is called instead of the regular `process_periodic()` method.

**Custom sample windows:**
- By default, processors use the same `sample_window` as periodic processing
- You can optionally specify a different window for pulse or event processors
- Custom windows can be smaller or larger, overlapping or non-overlapping
- The system automatically manages buffer sizing to accommodate all windows

**Example processor method:**
```python
def process_event(
        self, reference_time, reference_index, time_offsets, 
        eeg_buffer, emg_buffer, is_coil_at_target):
    """Process general events."""
    print(f"Event at {reference_time}")
    # Process event-specific logic
    return None
```

#### `predefined_sensory_stimuli` (list, optional)
List of pre-defined sensory stimuli sent to the presenter at initialization. Can be omitted if no predefined stimuli are needed.

**Stimulus dictionary structure:**
- `time` (float): When to present stimulus (seconds from session start)
- `type` (str): Stimulus type (e.g., "visual", "auditory", "tactile")
- `parameters` (dict): Stimulus-specific parameters (any key-value pairs)

**Example:**
```python
'predefined_sensory_stimuli': [
    {
        'time': 5.0,
        'type': 'visual',
        'parameters': {
            'color': 'red',
            'size': 100,
            'duration': 0.5,
            'position_x': 0,
            'position_y': 0
        }
    }
]
```

### `process_periodic(...)`

Main processing method called by the pipeline for periodic processing of EEG/EMG samples.

**Parameters:**

#### `reference_time` (float)
Reference time point in seconds. Other times in the buffer are relative to this.

#### `reference_index` (int)
Index in the buffer where `time_offsets[reference_index] == 0`. Points to last sample when `sample_window` is `[-t, 0]`.

#### `time_offsets` (numpy.ndarray)
Time offsets relative to `reference_time`. Shape: `(num_samples,)` where `num_samples` matches the sample window size.
- Absolute time for sample i is: `reference_time + time_offsets[i]`
- For `sample_window = [-1.0, 0]`, offsets range from -1.0 to 0.0 seconds

#### `eeg_buffer` (numpy.ndarray)
EEG sample data. Shape: `(num_samples, num_eeg_channels)`

#### `emg_buffer` (numpy.ndarray)
EMG sample data. Shape: `(num_samples, num_emg_channels)`

#### `is_coil_at_target` (bool)
Whether the coil is currently positioned at the target location (for neuronavigation systems).

#### `is_warm_up` (bool)
`True` when this call is a warm-up round with dummy data. Skip internal state updates in this case; return values are ignored.

**Return Value:**

The `process_periodic()` method can return a dictionary with the following optional keys:

#### `timed_trigger` (float)
Schedule a trigger pulse at specified time (seconds). Uses LabJack T4 for triggering external devices like commercial TMS systems.

**Example:**
```python
return {'timed_trigger': reference_time + 0.005}  # Trigger after 5ms
```

#### `sensory_stimuli` (list)
Dynamic sensory stimuli based on real-time data analysis. Same format as static stimuli in configuration.

**Example:**
```python
return {
    'sensory_stimuli': [
        {
            'time': reference_time + 1.0,
            'type': 'visual_cue',
            'parameters': {
                'color': 'blue',
                'intensity': 0.8,
                'duration': 0.2
            }
        }
    ]
}
```

### Event Processor Methods

Event processor methods (`process_pulse` and `process_event`) are called when events occur (configured via `pulse_processor` and `event_processor`). Each has the same signature as `process_periodic()` except without `is_warm_up` (they are never called during warm-up).

**Example:**
```python
def process_pulse(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target):
    """Process pulse events."""
    print(f"Pulse event at {reference_time}")
    # Process event-specific logic
    return {'sensory_stimuli': [...]}  # Or None

def process_event(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target):
    """Process general events."""
    print(f"Event at {reference_time}")
    # Process event-specific logic
    return None
```

**Processor naming:** Processor method names are arbitrary - they are explicitly mapped in the configuration via `pulse_processor` and `event_processor` keys.

## Processing Precedence

When multiple processing triggers occur at the same sample, only one Python method is called, following this precedence order:

1. **Pulse Events**: `pulse_processor` (if configured) is called
2. **General Events**: `event_processor` (if configured) is called
3. **Periodic Processing**: `process_periodic()` is called at regular intervals

**Notes:**
- When an event occurs at the same time as periodic processing, the event processor takes precedence and the `process_periodic()` method is **not** called for that sample
- However, the periodic processing timer continues to advance normally, so the next periodic processing will still occur at the expected time
- Events are processed even during pulse lockout periods; only periodic processing is suppressed during lockout

**Example Timeline:**
```
With periodic_processing_interval=3.0 and first_periodic_processing_at=1.0:
- 1.0s: Periodic processing scheduled, process_periodic() called
- 2.0s: Pulse event occurs, process_pulse() called (not process_periodic())
- 4.0s: Periodic processing scheduled, process_periodic() called
- 5.0s: General event occurs, process_event() called (not process_periodic())
- 7.0s: Periodic processing scheduled, process_periodic() called
```

In this example, even though events occurred at 2.0s and 5.0s, the periodic processing schedule (1.0s, 4.0s, 7.0s, ...) remains consistent and unaffected.

## Example Workflows

### Continuous Monitoring
```python
def get_configuration(self):
    return {
        'sample_window': [-0.100, 0.0],  # Last 100 ms
        'periodic_processing_enabled': True,
        'periodic_processing_interval': 0.001,  # Every sample (1ms at 1kHz)
        'pulse_lockout_duration': 0.0,
    }
```

### Event-Based Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-0.500, 0.0],
        'periodic_processing_enabled': False,  # No periodic processing
        'predefined_events': [10.0],  # Event at 10 seconds
        'event_processor': self.handle_trial_start,
    }
```

### Regular Interval Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-1.000, 0.0],  # Last second
        'periodic_processing_enabled': True,
        'periodic_processing_interval': 0.1,  # 10 times per second
        'pulse_lockout_duration': 2.0,
    }
```

### Sensory Stimuli Example
For a complete example demonstrating both predefined and dynamic sensory stimuli, see `example_sensory_stimuli.py`.

**Key features demonstrated:**
- Predefined stimuli sent at session start (text messages and visual cues)
- Dynamic stimuli generated during processing based on real-time data
- Compatible stimulus types for use with the presenter (`visual_cue`, `text_message`)

**Predefined stimuli in configuration:**
```python
'predefined_sensory_stimuli': [
    {
        'time': 0.5,
        'type': 'text_message',
        'parameters': {
            'text': 'Session starting...',
            'duration': 2.0
        }
    },
    {
        'time': 3.0,
        'type': 'visual_cue',
        'parameters': {
            'color': 'green',
            'size': 0.3,
            'duration': 1.0,
            'position_x': 0,
            'position_y': 0
        }
    }
]
```

**Dynamic stimuli in process_periodic method:**
```python
def process_periodic(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target, is_warm_up):
    # Generate stimuli based on current time or data
    return {
        'sensory_stimuli': [
            {
                'time': reference_time + 0.5,  # 0.5s from now
                'type': 'visual_cue',
                'parameters': {
                    'color': 'red',
                    'size': 150,
                    'duration': 1.5,
                    'position_x': 200,
                    'position_y': 100
                }
            }
        ]
    }
```

## Performance Optimization

### Warm-up Configuration

To prevent first-call performance delays, decider modules can request automatic warm-up during initialization.

#### `warm_up_rounds` (int)

Return this key from `get_configuration()` to specify the number of warm-up rounds:

```python
def get_configuration(self) -> dict[str, Any]:
    return {
        'sample_window': [-1.0, 0.0],
        'warm_up_rounds': 2,  # Recommended: 2-3 rounds
        # ... other configuration ...
    }
```

**How it works:**
- The C++ wrapper reads this value from `get_configuration()` during module initialization
- It calls your `process_periodic()` method the specified number of times with realistic dummy data
- Each round uses fresh random data (seeded for reproducibility) to avoid state-dependent issues
- This triggers JIT compilation, library loading, and other one-time initialization costs
- Subsequent real processing calls should have a consistent latency

**Configuration options:**
- `0`: Disable warm-up (default behavior)
- `1-5`: Recommended range (2-3 is usually optimal)
- Higher values: May provide additional stability but with diminishing returns

**When to use:**
- Always recommended for computationally intensive deciders
- Essential for real-time applications requiring consistent latency
- Particularly beneficial for modules using scipy, sklearn, or other heavy libraries

**Important for stateful deciders:**
If your decider maintains internal state that depends on real EEG/EMG data patterns (e.g., running averages, learned parameters, adaptive thresholds), you should skip state updates during warm-up rounds. `process_periodic` receives an explicit `is_warm_up` argument for this:

```python
def process_periodic(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target, is_warm_up):
    
    # Your processing logic here...
    processed_data = self.analyze_eeg(eeg_buffer)
    
    # Only update internal state with real data (skip during warm-up)
    if not is_warm_up:
        self.update_internal_state(processed_data)
    
    # Return decisions (warm-up returns are ignored by the system)
    return self.make_decision(processed_data)
```

## Best Practices

1. **Configure warm-up** with `'warm_up_rounds': 2` in `get_configuration()` for consistent performance
2. **Skip state updates during warm-up** using the `is_warm_up` argument in `process_periodic`
3. **Use multiprocessing pool** for computationally intensive tasks to avoid blocking the pipeline
