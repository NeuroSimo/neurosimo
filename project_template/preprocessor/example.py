import numpy as np


class Preprocessor:
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        # Track pulse artifacts
        self.ongoing_pulse_artifact = False
        self.samples_after_pulse = 0
        
        # Configure sample window for buffering
        self.sample_window = [-5, 5]

    def process(self, timestamps, eeg_samples, emg_samples, current_sample_index, pulse_given):
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
        eeg_sample_preprocessed = eeg_samples[current_sample_index, :]
        emg_sample_preprocessed = emg_samples[current_sample_index, :]
        
        # Mark sample validity
        valid = not self.ongoing_pulse_artifact

        return {
            'eeg_sample': eeg_sample_preprocessed,
            'emg_sample': emg_sample_preprocessed,
            'valid': valid,
        }
