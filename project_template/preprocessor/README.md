# Preprocessor Module Documentation

## Overview

The Preprocessor module handles real-time filtering, artifact removal, and signal conditioning of EEG/EMG data before it reaches the decider. It processes data sample-by-sample and determines sample validity for downstream analysis.

## Available Libraries

No third-party libraries are currently available in the preprocessor environment.

To add more libraries, modify `src/pipeline/preprocessor/Dockerfile` and run `build-neurosimo` from the command line.

## Class Methods

### `__init__(subject_id, num_eeg_channels, num_emg_channels, sampling_frequency)`

Initializes the preprocessor with device configuration parameters automatically provided by the pipeline.

**Parameters:**
- `subject_id` (str): Subject identifier
- `num_eeg_channels` (int): Number of EEG channels
- `num_emg_channels` (int): Number of EMG channels
- `sampling_frequency` (int): Sampling frequency in Hz

### `process(reference_time, reference_index, time_offsets, eeg_buffer, emg_buffer, pulse_given)`

Main processing method called for each new EEG/EMG sample.

**Parameters:**

#### `reference_time` (float)
Reference time point in seconds. Other times in the buffer are relative to this.

#### `reference_index` (int)
Index in the buffer where `time_offsets[reference_index] == 0`. Points to the current sample being processed.

#### `time_offsets` (numpy.ndarray)
Time offsets relative to `reference_time`. Shape: `(num_samples,)` where `num_samples` matches the sample window size.
- Absolute time for sample i is: `reference_time + time_offsets[i]`
- For `sample_window = [-1.0, 0]`, offsets range from -1.0 to 0.0 seconds

#### `eeg_buffer` (numpy.ndarray)
EEG sample buffer containing recent samples. Shape: `(num_samples, num_eeg_channels)`

#### `emg_buffer` (numpy.ndarray)
EMG sample buffer containing recent samples. Shape: `(num_samples, num_emg_channels)`

#### `pulse_given` (bool)
Whether a TMS pulse was delivered on this sample. Used for artifact detection and removal.

**Return Value:**

Must return a dictionary with the following keys:

#### `eeg_sample` (numpy.ndarray)
Preprocessed EEG sample for the current timestamp. Shape: `(num_eeg_channels,)`

#### `emg_sample` (numpy.ndarray)
Preprocessed EMG sample for the current timestamp. Shape: `(num_emg_channels,)`

#### `valid` (bool)
Whether the current sample is valid for analysis. Invalid samples are typically:
- Affected by TMS pulse artifacts
- Contaminated by movement or other artifacts
- Outside acceptable signal ranges
- Missing or corrupted data

The decider receives this validity information and can choose to skip processing when samples are invalid.

## Sample Window Configuration

The `sample_window` configuration defines the buffer size available for preprocessing:

- **Format**: `[earliest_seconds, latest_seconds]` relative to current sample, expressed in **seconds**
- **Current sample**: Always at `reference_index` where `time_offsets[reference_index] == 0`
- **Values**: Converted to samples using the provided sampling frequency

**Examples:**
- `[-0.005, 0.0]`: Keep last 5 ms + current sample
- `[-1.0, 0.0]`: Keep last second (at any sampling rate)
- `[0.0, 0.0]`: Keep only current sample
- `[-0.005, 0.005]`: Look 5 ms back and 5 ms ahead (introduces 5 ms delay)

**Delay implications:**
- Positive latest_seconds values introduce processing delay
- `[-0.005, 0.005]` means 5 ms delay for causal processing
- Choose based on your filtering requirements vs. acceptable latency

## Common Preprocessing Tasks

### Artifact Detection
```python
# Detect TMS pulse artifacts
if pulse_given:
    self.ongoing_pulse_artifact = True
    self.samples_after_pulse = 0

# Mark samples invalid for 1000 samples after pulse (artifact duration)
if self.ongoing_pulse_artifact:
    self.samples_after_pulse += 1
    if self.samples_after_pulse == 1000:
        self.ongoing_pulse_artifact = False
```

### Filtering
```python
# Apply high-pass filter to remove DC drift
from scipy import signal
if hasattr(self, 'filter_zi'):
    eeg_filtered, self.filter_zi = signal.lfilter(
        self.b, self.a, [eeg_buffer[reference_index, :]], 
        zi=self.filter_zi
    )
else:
    # Initialize filter state on first sample
    self.b, self.a = signal.butter(4, 1.0/(self.sampling_frequency/2), 'high')
    self.filter_zi = signal.lfilter_zi(self.b, self.a) * eeg_buffer[reference_index, :]
```

### Amplitude-Based Artifact Rejection
```python
# Reject samples exceeding amplitude thresholds
amplitude_threshold = 100  # microvolts
current_eeg = eeg_buffer[reference_index, :]
valid = not np.any(np.abs(current_eeg) > amplitude_threshold)
```

### Movement Artifact Detection
```python
# Use EMG channels to detect movement artifacts
emg_power = np.mean(np.square(emg_buffer[reference_index, :]))
movement_threshold = 50  # adjust based on your setup
movement_detected = emg_power > movement_threshold
valid = not movement_detected
```

## Example Workflows

### Pass-Through Preprocessor
```python
def process(self, reference_time, reference_index, time_offsets, eeg_buffer, emg_buffer, pulse_given):
    # No actual preprocessing, just pass through
    return {
        'eeg_sample': eeg_buffer[reference_index, :],
        'emg_sample': emg_buffer[reference_index, :], 
        'valid': True
    }
```

### Simple Artifact Rejection
```python
def process(self, reference_time, reference_index, time_offsets, eeg_buffer, emg_buffer, pulse_given):
    # Mark samples invalid for 1 second after pulse
    if pulse_given:
        self.samples_since_pulse = 0
        
    valid = self.samples_since_pulse > self.sampling_frequency
    self.samples_since_pulse += 1
    
    return {
        'eeg_sample': eeg_buffer[reference_index, :],
        'emg_sample': emg_buffer[reference_index, :],
        'valid': valid
    }
```

### Real-Time Filtering
```python
def process(self, reference_time, reference_index, time_offsets, eeg_buffer, emg_buffer, pulse_given):
    # Apply bandpass filter (1-30 Hz)
    current_eeg = eeg_buffer[reference_index, :]
    
    if not hasattr(self, 'eeg_filtered_prev'):
        self.eeg_filtered_prev = current_eeg
        filtered_eeg = current_eeg
    else:
        # Simple moving average (replace with proper filter)
        alpha = 0.1
        filtered_eeg = alpha * current_eeg + (1 - alpha) * self.eeg_filtered_prev
        self.eeg_filtered_prev = filtered_eeg
    
    return {
        'eeg_sample': filtered_eeg,
        'emg_sample': emg_buffer[reference_index, :],
        'valid': True
    }
```

## Integration with Pipeline

### Data Flow
1. **Raw samples** arrive from EEG/EMG device
2. **Preprocessor** processes each sample and marks validity
3. **Decider** receives preprocessed samples and validity flags
4. **Pipeline** uses validity information for trial decisions

### Performance Considerations
- **Real-time constraints**: Keep processing lightweight for high sampling rates
- **Filter initialization**: Handle filter state properly on startup
- **Memory usage**: Avoid accumulating large buffers unnecessarily
- **Latency**: Balance filtering quality vs. acceptable processing delay

### State Management
- **Initialize state variables** in `__init__` method
- **Update state** incrementally in each `process` call  
- **Handle edge cases** like startup, pulse events, and data gaps
- **Reset state** when needed (e.g., after long breaks in data)

## Best Practices

1. **Always return required keys** (`eeg_sample`, `emg_sample`, `valid`)
2. **Handle pulse artifacts** appropriately for your experimental design
3. **Validate input data** before processing (check for NaN, inf values)
4. **Keep preprocessing efficient** to maintain real-time performance
5. **Log important events** for debugging and validation
6. **Test with simulated data** before using with real experiments
7. **Consider filter startup** transients and initialization
8. **Document your artifact rejection criteria** for reproducibility
