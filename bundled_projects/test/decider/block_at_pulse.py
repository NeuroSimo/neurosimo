from typing import Any

import multiprocessing
import time

import numpy as np


class Decider:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency

        print("Decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1.0, 0.0],
            'warm_up_rounds': 2,

            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 1.0,
            'pulse_lockout_duration': 2.0,

            # Event system
            'pulse_processor': self.process_pulse,
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, is_warm_up: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically."""

        # Trigger TMS device after 5ms delay
        trigger_time = reference_time + 0.005

        print(f"Creating trigger for time {trigger_time} seconds")

        return {
            'timed_trigger': trigger_time,
        }

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}, blocking after pulse.")

        blocking_time = 3
        for i in range(blocking_time):
            print(f"Still blocking...")
            time.sleep(1)

        print("Blocking complete.")

        return None
