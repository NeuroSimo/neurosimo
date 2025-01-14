# An example preprocessor that does not do any actual processing to the data, just
# passes the data through. This can be a useful starting point for more advanced
# preprocessing algorithms. For another example of a preprocessor, see `sound.py`
# file in the same directory.
#
# For a more comprehensive documentation, please see a corresponding example of a
# decider in the decider directory; most of the concepts are the same.

import numpy as np


class Preprocessor:
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        # Initialize state.
        self.ongoing_pulse_artifact = False
        self.samples_after_pulse = 0
        self.sample_count = 0

        # Configure the length of sample window.
        self.sample_window = [-5, 5]

    def process(self, timestamps, eeg_samples, emg_samples, current_sample_index, pulse_given):
        self.sample_count += 1

        # If a pulse has been given on the previous sample, set the ongoing_pulse_artifact flag
        # and starting counting the samples after the pulse.
        if pulse_given:
            self.ongoing_pulse_artifact = True
            self.samples_after_pulse = 0
            print("A pulse was given.")

        # Assuming that an ongoing artifact lasts for 1000 samples; after that, reset the flag.
        if self.ongoing_pulse_artifact:
            self.samples_after_pulse += 1
            if self.samples_after_pulse == 1000:
                self.ongoing_pulse_artifact = False

        # Return the incoming raw sample as it is; doesn't do any actual processing to the data.
        eeg_sample_preprocessed = eeg_samples[current_sample_index,:]
        emg_sample_preprocessed = emg_samples[current_sample_index,:]

        # Mark the sample as invalid if there is an ongoing pulse artifact.
        valid = not self.ongoing_pulse_artifact

        # Based on the incoming buffer of raw EEG/EMG samples, return a single sample that
        # corresponds to the current timestamp. These samples are collected by the decider
        # in its own buffer, in turn, to be used for the stimulation decision.
        return {
            'eeg_sample': eeg_sample_preprocessed,
            'emg_sample': emg_sample_preprocessed,
            'valid': valid,
        }
