# Decider Module Documentation

## Overview

The Decider module processes real-time EEG/EMG data and makes decisions about when to trigger TMS pulses or present sensory stimuli. This documentation covers the complete API and configuration options.

## Available Libraries

The following third-party libraries are currently available in the decider environment:

- numpy
- scipy
- scikit-learn
- statsmodels
- mneflow

To add more libraries, modify `ros2_ws/src/neurosimo_packages/pipeline/decider/Dockerfile` and run `build-neurosimo` from the command line.

## Class Methods

### `__init__(num_of_eeg_channels, num_of_emg_channels, sampling_frequency)`

Initializes the decider with device configuration parameters automatically provided by the pipeline.

**Parameters:**
- `num_of_eeg_channels` (int): Number of EEG channels
- `num_of_emg_channels` (int): Number of EMG channels  
- `sampling_frequency` (int): Sampling frequency in Hz

### `get_configuration()`

Called by the pipeline during initialization. Must return a dictionary with configuration parameters.

**Return dictionary keys:**

#### `periodic_processing_interval` (float)
How frequently the `process` method is called, in seconds.

**Special cases:**
- `0.0`: Process method never called periodically (only on events/EEG triggers)
- Very small values (e.g., `0.001`): Near-continuous processing

**Examples:**
- `1.0`: Process once per second
- `0.1`: Process 10 times per second
- `0.01`: Process 100 times per second

**Note:** EEG triggers always call `process_eeg_trigger()` regardless of this setting.

#### `first_periodic_processing_at` (float, optional)
Time of the first periodic processing call in seconds (relative to session start). Defaults to `periodic_processing_interval` if not specified.

**Examples:**
- `1.0`: First processing at 1.0s, then every `periodic_processing_interval`
- `0.5`: First processing at 0.5s (useful for offset timing)

**Note:** This parameter is optional and primarily exists for precise timing control. Most users should omit it and use the default behavior.

#### `sample_window` (list)
Two-element list `[earliest_sample, latest_sample]` defining the buffer size relative to current sample.
- Current sample is always `0`
- Earliest sample is always negative or zero
- Values are in samples, not seconds

**Examples:**
- `[-5, 0]`: Keep last 5 samples + current (6 total)
- `[-999, 0]`: Keep last second at 1000 Hz
- `[0, 0]`: Keep only current sample
- `[-5, 5]`: Look 5 samples back and 5 ahead (introduces 5-sample delay)

#### `pulse_lockout_duration` (float)
Duration in seconds during which periodic processing is disabled after a pulse is delivered.
- Prevents new stimulation during the lockout period
- Events and EEG triggers are still processed during lockout
- Set to `0.0` to disable lockout

**Examples:**
- `2.0`: No periodic processing for 2 seconds after pulse
- `0.0`: No lockout (periodic processing continues immediately)

#### `predefined_events` (list)
List of event dictionaries for scheduled processing triggers.

**Event dictionary structure:**
- `type` (str): Event type identifier (e.g., "pulse", "baseline_start")
- `time` (float): Event time in seconds relative to session start

**Example:**
```python
'predefined_events': [
    {'type': 'pulse', 'time': 10.0},
    {'type': 'baseline_start', 'time': 5.0},
]
```

#### `event_handlers` (dict)
Dictionary mapping event types to handler methods. Each event type must have a corresponding handler.

**Simple format:**
```python
'event_handlers': {
    'pulse': self.handle_pulse,
    'baseline_start': self.handle_baseline_start,
}
```

**Advanced format with custom sample window:**
```python
'event_handlers': {
    'pulse': {
        'handler': self.handle_pulse,
        'sample_window': [-500, 100],  # Custom window just for pulse events
    },
    'baseline_start': self.handle_baseline_start,  # Uses default window
}
```

When an event occurs, its corresponding handler method is called instead of the regular `process()` method. Handler methods have the same signature as `process()` but without event-related parameters (`is_event`, `event_type`).

**Custom sample windows:**
- By default, event handlers use the same `sample_window` as periodic processing
- You can optionally specify a different window for specific events
- Custom windows can be smaller or larger, overlapping or non-overlapping
- The system automatically manages buffer sizing to accommodate all windows

**Example handler method:**
```python
def handle_pulse(self, current_time, timestamps, valid_samples, 
                eeg_buffer, emg_buffer, current_sample_index, 
                ready_for_trial, is_coil_at_target):
    """Handle pulse events."""
    print(f"Pulse event at {current_time}")
    # Process pulse-specific logic
    return None
```

#### `predefined_sensory_stimuli` (list)
List of pre-defined sensory stimuli sent to the presenter at initialization.

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

### `process(...)`

Main processing method called by the pipeline for periodic processing of EEG/EMG samples.

**Parameters:**

#### `current_time` (float)
Time of current sample in seconds.

#### `timestamps` (numpy.ndarray)
Timestamps for samples in the buffer. Shape: `(num_samples,)` where `num_samples` matches the sample window size.

#### `valid_samples` (numpy.ndarray)
Boolean array indicating sample validity. Shape: `(num_samples,)`. 
- Sample validity determined by pipeline preprocessor
- Default preprocessor marks samples invalid for 1 second after each pulse

#### `eeg_buffer` (numpy.ndarray)
EEG sample data. Shape: `(num_samples, num_eeg_channels)`

#### `emg_buffer` (numpy.ndarray)
EMG sample data. Shape: `(num_samples, num_emg_channels)`

#### `current_sample_index` (int)
Index of current sample in the buffer. Points to last sample when `sample_window` is `[-n, 0]`.

#### `ready_for_trial` (bool)
Whether pipeline can accept new trial requests.
- `False` during ongoing trials or target precomputation
- Decider should not trigger trials when `False` (warnings issued if attempted)
- Other processing (logging, filtering, training) can continue regardless

#### `is_coil_at_target` (bool)
Whether the coil is currently positioned at the target location (for neuronavigation systems).

**Return Value:**

The `process` method can return a dictionary with the following optional keys:

#### `timed_trigger` (float)
Schedule a trigger pulse at specified time (seconds). Uses LabJack T4 for triggering external devices like commercial TMS systems.

**Example:**
```python
return {'timed_trigger': current_time + 0.005}  # Trigger after 5ms
```

#### `sensory_stimuli` (list)
Dynamic sensory stimuli based on real-time data analysis. Same format as static stimuli in configuration.

**Example:**
```python
return {
    'sensory_stimuli': [
        {
            'time': current_time + 1.0,
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

### `process_eeg_trigger(...)`

**Mandatory method** called when an EEG trigger is received from the EEG device.

**Parameters:**
Same as `process()` method but without `is_event` and `event_type` parameters:
- `current_time` (float)
- `timestamps` (numpy.ndarray)
- `valid_samples` (numpy.ndarray)
- `eeg_buffer` (numpy.ndarray)
- `emg_buffer` (numpy.ndarray)
- `current_sample_index` (int)
- `ready_for_trial` (bool)
- `is_coil_at_target` (bool)

**Return Value:**
Same format as `process()` method.

**Example:**
```python
def process_eeg_trigger(self, current_time, timestamps, valid_samples,
                       eeg_buffer, emg_buffer, current_sample_index,
                       ready_for_trial, is_coil_at_target):
    """Handle EEG trigger from the EEG device."""
    print(f"EEG trigger received at {current_time}")
    # Return None if you don't care about EEG triggers
    return None
```

**Note:** EEG triggers are not available with simulated data. Use events for similar functionality.

### Event Handler Methods

Event handler methods are called when events occur (defined in `event_handlers` configuration). Each handler has the same signature as `process_eeg_trigger()`.

**Example:**
```python
def handle_pulse(self, current_time, timestamps, valid_samples,
                eeg_buffer, emg_buffer, current_sample_index,
                ready_for_trial, is_coil_at_target):
    """Handle pulse events."""
    print(f"Pulse event at {current_time}")
    # Process event-specific logic
    return {'sensory_stimuli': [...]}  # Or None
```

**Handler naming:** Handler method names are arbitrary - they are explicitly mapped in `event_handlers` configuration.

## Example Workflows

### Continuous Monitoring
```python
def get_configuration(self):
    return {
        'sample_window': [-100, 0],  # Last 100ms at 1kHz
        'periodic_processing_interval': 0.001,  # Every sample (1ms at 1kHz)
        'predefined_events': [],
        'event_handlers': {},
        'predefined_sensory_stimuli': [],
        'pulse_lockout_duration': 0.0,
    }
```

### Event-Based Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-500, 0],
        'periodic_processing_interval': 0.0,  # No periodic processing
        'predefined_events': [
            {'type': 'trial_start', 'time': 10.0}
        ],
        'event_handlers': {
            'trial_start': self.handle_trial_start,
        },
        'predefined_sensory_stimuli': [],
        'pulse_lockout_duration': 0.0,
    }
```

### Regular Interval Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-1000, 0],  # Last second
        'periodic_processing_interval': 0.1,  # 10 times per second
        'predefined_events': [],
        'event_handlers': {},
        'predefined_sensory_stimuli': [],
        'pulse_lockout_duration': 2.0,
    }
```

## Performance Optimization

### Warm-up Configuration

To prevent first-call performance delays, decider modules can request automatic warm-up during initialization.

#### `warm_up_rounds` (int)

Set this attribute in your decider's `__init__` method to specify the number of warm-up rounds:

```python
def __init__(self, num_eeg_channels, num_emg_channels, sampling_frequency):
    # ... other initialization code ...
    
    # Configure warm-up (recommended: 2-3 rounds)
    self.warm_up_rounds = 2
```

**How it works:**
- The C++ wrapper automatically detects this attribute during module initialization
- It calls your `process()` method the specified number of times with realistic dummy data
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
If your decider maintains internal state that depends on real EEG/EMG data patterns (e.g., running averages, learned parameters, adaptive thresholds), you should skip state updates during warm-up rounds. Warm-up calls can be identified by checking if `current_time == 0.0`:

```python
def process(self, current_time, timestamps, valid_samples, eeg_buffer, emg_buffer, 
            current_sample_index, ready_for_trial, is_coil_at_target):
    
    # Skip state updates during warm-up (dummy data)
    is_warmup = (current_time == 0.0)
    
    # Your processing logic here...
    processed_data = self.analyze_eeg(eeg_buffer)
    
    # Only update internal state with real data
    if not is_warmup:
        self.update_internal_state(processed_data)
    
    # Return decisions (warm-up returns are ignored by the system)
    return self.make_decision(processed_data)
```

## Best Practices

1. **Check `ready_for_trial`** before scheduling triggers
2. **Validate samples** using `valid_samples` array before processing
3. **Configure warm-up** with `self.warm_up_rounds = 2` for consistent performance
4. **Skip state updates during warm-up** by checking `current_time == 0.0` for stateful deciders
5. **Use multiprocessing pool** for computationally intensive tasks to avoid blocking the pipeline

## mTMS Device Usage

The mTMS (multi-locus TMS) device requires special configuration and is only compatible with specific deciders. Use `mtms_decider_template.py` when working with the mTMS device.

### Prerequisites

- NeuroSimo must be running concurrently with the mTMS device software (see https://github.com/connect2brain/mtms)
- mTMS device must be enabled in the environment configuration file (.env)
- To perform trials more frequently than every 2 seconds, reduce `MINIMUM_INTERTRIAL_INTERVAL` in the environment config and restart the pipeline

### Target Configuration

mTMS targets are defined as lists of dictionaries. Each target dictionary contains:

- `displacement_x` (float): X-coordinate in millimeters (-15 to 15)
- `displacement_y` (float): Y-coordinate in millimeters (-15 to 15)
- `rotation_angle` (float): Rotation angle in degrees (0 to 359)
- `intensity` (float): Intensity in V/m
- `algorithm` (str): Algorithm for computing capacitor voltages
  - `'least_squares'`: Use least squares algorithm
  - `'genetic'`: Use genetic algorithm

**Target Types:**
- **Single pulse**: List with one dictionary
- **Paired pulse**: List with two dictionaries

**Example targets:**
```python
SINGLE_PULSE = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 25,
        'algorithm': 'least_squares',
    }
]

PAIRED_PULSE = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 30,
        'algorithm': 'least_squares',
    },
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 20,
        'algorithm': 'least_squares',
    }
]
```

### mTMS Triggers

The mTMS device supports up to two output triggers that can coincide with TMS pulses. Each trigger is configured with:

- `enabled` (bool): Whether the trigger is enabled
- `delay` (float): Delay in seconds before trigger activation
  - `0.0`: Trigger coincides with pulse
  - Positive: Trigger after pulse
  - Negative: Trigger before pulse

**Example:**
```python
TRIGGERS = [
    {'enabled': True, 'delay': 0.0},      # Trigger 1: coincides with pulse
    {'enabled': True, 'delay': -0.01}     # Trigger 2: 10ms before pulse
]
```

### mTMS Trial Return Format

When using mTMS, the `process` method should return a dictionary with a `'trial'` key:

```python
return {
    'trial': {
        'targets': targets,           # List of target dictionaries
        'pulse_times': pulse_times,   # List of pulse times in seconds
        'triggers': TRIGGERS,         # Trigger configuration
    }
}
```

### Target Precomputation

For real-time operation, the mTMS pipeline precomputes capacitor voltages for all targets. Define all experimental targets in `self.targets` during initialization:

```python
self.targets = [
    LOW_INTENSITY_TARGET,
    HIGH_INTENSITY_TARGET,  
    PAIRED_PULSE_TARGET,
]
```

Missing targets will be computed on-the-fly, introducing 10-20 second delays.

### Minimum Pulse Delay

Set `MINIMUM_DELAY_BEFORE_PULSE` based on your pipeline's processing latency:
- Start with 50ms (`0.050`) for basic processing
- Increase for complex algorithms or slower hardware
- Account for EEG device filtering, preprocessing, and decision-making time
