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

#### `processing_interval_in_samples` (int)
How frequently the `process` method is called, in samples.

**Special cases:**
- `1`: Process method called for every new sample (continuous monitoring)
- `0`: Process method never called periodically (only on triggers/events if enabled)

**Examples:**
- `100`: Process every 100 samples
- `sampling_frequency`: Process once per second

#### `process_on_trigger` (bool)
Whether to call `process` method when triggers are received from the EEG device.
- `True`: Process on triggers in addition to periodic processing
- `False`: Only process periodically

#### `sample_window` (list)
Two-element list `[earliest_sample, latest_sample]` defining the buffer size relative to current sample.
- Current sample is always `0`
- Earliest sample is always negative
- Values are in samples, not seconds

**Examples:**
- `[-5, 0]`: Keep last 5 samples + current (6 total)
- `[-999, 0]`: Keep last second at 1000 Hz
- `[0, 0]`: Keep only current sample
- `[-5, -5]`: Look 5 samples into future (introduces 5-sample delay)

#### `events` (list)
List of event dictionaries for scheduled processing triggers.

**Event dictionary structure:**
- `type` (str): Event type identifier (e.g., "prepulse", "postpulse", "baseline_start")
- `time` (float): Event time in seconds relative to session start

**Example:**
```python
'events': [
    {'type': 'prepulse', 'time': 10.0},
    {'type': 'postpulse', 'time': 10.5}
]
```

#### `sensory_stimuli` (list)
List of pre-defined sensory stimuli sent to the presenter.

**Stimulus dictionary structure:**
- `time` (float): When to present stimulus (seconds from session start)
- `type` (str): Stimulus type (e.g., "visual", "auditory", "tactile")
- `parameters` (dict): Stimulus-specific parameters (any key-value pairs)

**Example:**
```python
'sensory_stimuli': [
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

Main processing method called by the pipeline for new EEG/EMG samples.

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

#### `is_trigger` (bool)
Whether a trigger was received from EEG device on this sample.
- Not available with simulated data
- Use events for similar functionality

#### `is_event` (bool)
Whether an event occurred on this sample (from `events` configuration).

#### `event_type` (str)
Type of event that occurred (e.g., "prepulse", "postpulse").

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

## Example Workflows

### Continuous Monitoring
```python
def get_configuration(self):
    return {
        'processing_interval_in_samples': 1,  # Every sample
        'process_on_trigger': False,
        'sample_window': [-100, 0],  # Last 100ms at 1kHz
        'events': [],
        'sensory_stimuli': []
    }
```

### Event-Based Processing
```python
def get_configuration(self):
    return {
        'processing_interval_in_samples': 0,  # No periodic processing
        'process_on_trigger': True,
        'sample_window': [-500, 0],
        'events': [
            {'type': 'trial_start', 'time': 10.0}
        ],
        'sensory_stimuli': []
    }
```

### Regular Interval Processing
```python
def get_configuration(self):
    return {
        'processing_interval_in_samples': self.sampling_frequency // 10,  # 10 times per second
        'process_on_trigger': False,
        'sample_window': [-1000, 0],  # Last second
        'events': [],
        'sensory_stimuli': []
    }
```

## Best Practices

1. **Check `ready_for_trial`** before scheduling triggers
2. **Validate samples** using `valid_samples` array before processing
3. **Use multiprocessing pool** for computationally intensive tasks to avoid blocking the pipeline
4. **Handle edge cases** gracefully (empty buffers, invalid data)
5. **Log important events** for debugging and analysis
6. **Keep processing efficient** to maintain real-time performance

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
