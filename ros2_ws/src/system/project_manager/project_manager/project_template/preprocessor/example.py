from typing import Any
import numpy as np


class Preprocessor:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency

        # Track pulse artifacts
        self.ongoing_pulse_artifact = False
        self.samples_after_pulse = 0

        print("Preprocessor initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-0.005, 0.0],  # 5 ms look-back, 0 ms look-ahead
        }

    def process(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            pulse_given: bool) -> dict[str, Any]:
        """Process incoming EEG/EMG samples and return preprocessed data."""
        
        # Handle pulse artifact detection
        if pulse_given:
            self.ongoing_pulse_artifact = True
            self.samples_after_pulse = 0
            print("A pulse was given.")

        # Mark samples invalid for 1000 samples after pulse (artifact duration)
        if self.ongoing_pulse_artifact:
            self.samples_after_pulse += 1
            if self.samples_after_pulse == 1000:
                self.ongoing_pulse_artifact = False

        # Pass through raw samples (no actual preprocessing in this example)
        eeg_sample_preprocessed = eeg_buffer[reference_index, :]
        emg_sample_preprocessed = emg_buffer[reference_index, :]
        
        # Mark sample validity
        valid = not self.ongoing_pulse_artifact

        return {
            'eeg_sample': eeg_sample_preprocessed,
            'emg_sample': emg_sample_preprocessed,
            'valid': valid,
        }
