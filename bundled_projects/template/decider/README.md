# Decider Module Documentation

## Overview

The Decider module processes real-time EEG/EMG data and makes decisions about when to trigger TMS pulses or present sensory stimuli. This documentation covers the complete API and configuration options.

## Example Deciders

The `project_template/decider/` directory contains several example decider modules:

- **`example.py`**: Basic periodic processing with event handling
- **`example_predetermined.py`**: Demonstrates predetermined trial timing with per-trial ITI scheduling
- **`example_sensory_stimuli.py`**: Demonstrates both predefined and dynamic sensory stimuli
- **`phastimate.py`**: Real-time phase estimation for brain state-dependent stimulation

## Available Libraries

The following third-party libraries are currently available in the decider environment:

- numpy
- scipy
- scikit-learn
- statsmodels
- mneflow

To add more libraries, modify `src/decider/Dockerfile` and run `build-neurosimo` from the command line.

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

#### `periodic_processing_interval` (float, optional)
How frequently the `process_periodic()` method is called, in seconds. Must be greater than `0.0`. Defaults to `0.1` (10 times per second) if not specified.

**Examples:**
- `0.1`: Process 10 times per second (default)
- `1.0`: Process once per second
- `0.01`: Process 100 times per second

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

#### `predefined_events` (list, optional)
List of event times (in seconds) for scheduled processing triggers. Can be omitted if no predefined events are needed.

**Format:**
- Simple list of floats representing event times relative to session start
- When an event time is reached, `process_event()` is called (if the method is defined)

**Example:**
```python
'predefined_events': [5.0, 10.0, 15.0]  # Events at 5s, 10s, and 15s
```

#### `pulse_sample_window` (list, optional)
Custom sample window for `process_pulse()` calls, as a two-element list `[earliest_seconds, latest_seconds]`. If omitted, the default `sample_window` is used.

**Example:**
```python
'pulse_sample_window': [-0.500, 0.100],
```

#### `event_sample_window` (list, optional)
Custom sample window for `process_event()` calls, as a two-element list `[earliest_seconds, latest_seconds]`. If omitted, the default `sample_window` is used.

**Example:**
```python
'event_sample_window': [-1.5, 0.3],
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

#### `stage_name` (str)
Current protocol stage name from the experiment coordinator.

#### `trial_in_stage` (int)
Total successful trials in session.

#### `is_warm_up` (bool)
`True` when this call is a warm-up round with dummy data. Skip internal state updates in this case; return values are ignored.
During warm-up calls, `stage_name` is an empty string (`""`).

**Return Value:**

The `process_periodic()` method can return a dictionary with the following optional keys:

#### `trigger_offset` (float)
Schedule a trigger pulse using an offset in seconds relative to `reference_time`. Uses LabJack T4 for triggering external devices like commercial TMS systems. Only allowed from `process_periodic()` — returning this from pulse or event processors will cause an error.

**Example:**
```python
return {'trigger_offset': 0.005}  # Trigger 5ms after reference_time
```

#### `targeted_pulses` (list)
Publish targeted pulse requests to `/mtms/targeted_pulses` for external stimulation software.
Do not return `trigger_offset` and `targeted_pulses` in the same result.

Each list item must be a dictionary with:
- `time_offset` (float): Pulse time as an offset in seconds relative to `reference_time`
- `displacement_x` (float): X-coordinate in millimeters
- `displacement_y` (float): Y-coordinate in millimeters
- `rotation_angle` (float): Rotation angle in degrees
- `intensity` (float): Intensity in V/m

**Example:**
```python
return {
    'targeted_pulses': [
        {
            'time_offset': 0.010,
            'displacement_x': 0.0,
            'displacement_y': 0.0,
            'rotation_angle': 0.0,
            'intensity': 30.0,
        },
        {
            'time_offset': 0.060,
            'displacement_x': 0.0,
            'displacement_y': 0.0,
            'rotation_angle': 0.0,
            'intensity': 20.0,
        },
    ]
}
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

#### `events` (list)
Dynamically schedule new events by returning a list of event times (in seconds, relative to session start). These are added to the same event queue as `predefined_events` and will trigger `process_event()` when reached.

**Example:**
```python
return {
    'events': [reference_time + 5.0, reference_time + 10.0]
}
```

#### `coil_target` (str)
Direct the neuronavigation system to a named coil target.

**Example:**
```python
return {
    'coil_target': 'target_1'
}
```

### `process_pulse(...)`

Called when a pulse event occurs, if the method is defined on the `Decider` class. Same signature as `process_periodic()` except without `is_warm_up` (never called during warm-up).

**Return Value:**

May return `None` or a dictionary with `sensory_stimuli`, `events`, `coil_target` (same format as `process_periodic()`), and optionally:

#### `trial_invalid` (bool, optional)
Mark the current trial as invalid (e.g. artifact, failed quality check). Defaults to `false` if omitted. When `true`, the experiment coordinator does not advance the stage trial counter; the attempt is retried. Stages may set `max_failures` in the protocol to cap how many invalid trials are allowed before the stage ends (see protocols README).

**Example:**
```python
def process_pulse(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target, stage_name, trial_in_stage):
    """Process pulse events."""
    if self.has_artifact(eeg_buffer):
        return {'trial_invalid': True}
    return None
```

### `process_event(...)`

Called when a general event occurs (from `predefined_events` or dynamically scheduled events), if the method is defined on the `Decider` class. Same signature as `process_periodic()` except without `is_warm_up` (never called during warm-up).

**Example:**
```python
def process_event(
        self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target, stage_name, trial_in_stage):
    """Process general events."""
    print(f"Event at {reference_time}")
    return None
```

**Example Timeline:**
```
With periodic_processing_interval=3.0:
- 3.0s: Periodic processing scheduled, process_periodic() called
- 4.0s: Pulse event occurs, process_pulse() called (not process_periodic())
- 6.0s: Periodic processing scheduled, process_periodic() called
- 7.0s: General event occurs, process_event() called (not process_periodic())
- 9.0s: Periodic processing scheduled, process_periodic() called
```

In this example, even though events occurred at 4.0s and 7.0s, the periodic processing schedule (3.0s, 6.0s, 9.0s, ...) remains consistent and unaffected.

### `process_predetermined(reference_time, stage_name, trial, trial_type)`

Called once per **predetermined** trial (protocol stages with `timing: predetermined`) when the trial counter advances. The method must return the trigger schedule for that trial upfront; the pipeline then schedules the trigger accordingly without waiting for the next periodic cycle.

Not called during warm-up rounds.

**Parameters:**

#### `reference_time` (float)
Current sample time in seconds since recording start. Use this as the base time when computing the trigger offset.

#### `stage_name` (str)
Name of the current protocol stage.

#### `trial` (int)
Zero-based index of the current trial within the stage.

#### `trial_type` (str)
The `type` string from the protocol entry that owns this trial (e.g. `"low_iti"`, `"open_loop_fast"`). An empty string if no `type` was specified in the protocol.

**Return value:** Same format as `process_periodic()` — a dictionary with `trigger_offset` or `targeted_pulses`. Returning `None` skips the trial.

**Example:**
```python
def process_predetermined(
        self, reference_time: float, stage_name: str, trial: int, trial_type: str) -> dict[str, Any] | None:
    if trial_type == 'low_iti':
        iti = self.rng.uniform(3.0, 5.0)
    elif trial_type == 'high_iti':
        iti = self.rng.uniform(5.0, 7.0)
    else:
        assert False, f"Unknown trial type: {trial_type}"

    return {'trigger_offset': iti}
```

## Example Workflows

### Predetermined Trial Timing
```python
def get_configuration(self):
    return {
        'sample_window': [-1.0, 0.0],
        'warm_up_rounds': 2,
    }

def process_predetermined(
        self, reference_time: float, stage_name: str, trial: int, trial_type: str):
    """Called once per predetermined trial to schedule its trigger upfront."""
    iti = self.rng.uniform(3.0, 5.0)
    return {'trigger_offset': iti}
```

For a complete example, see `example_predetermined.py`.

### Continuous Monitoring
```python
def get_configuration(self):
    return {
        'sample_window': [-0.100, 0.0],  # Last 100 ms
        'periodic_processing_interval': 0.001,  # Every sample (1ms at 1kHz)
    }
```

### Event-Based Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-0.500, 0.0],
        'periodic_processing_interval': 10.0,  # Infrequent periodic processing
        'predefined_events': [10.0],  # Event at 10 seconds
    }

def process_event(self, reference_time, reference_index, time_offsets,
        eeg_buffer, emg_buffer, is_coil_at_target, stage_name, trial_in_stage):
    """Handle trial start event."""
    # ...
```

### Regular Interval Processing
```python
def get_configuration(self):
    return {
        'sample_window': [-1.000, 0.0],  # Last second
        # periodic_processing_interval defaults to 0.1 (10 times per second)
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
        eeg_buffer, emg_buffer, is_coil_at_target, stage_name, trial_in_stage, is_warm_up):
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
        # periodic_processing_interval defaults to 0.1 s if omitted
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
        eeg_buffer, emg_buffer, is_coil_at_target, stage_name, trial_in_stage, is_warm_up):
    
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
