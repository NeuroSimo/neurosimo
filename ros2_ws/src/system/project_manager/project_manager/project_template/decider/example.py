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
        
        # Initialize multiprocessing pool for background computations
        self.pool = multiprocessing.Pool(processes=1)

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

        print("Decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1.0, 0.0],
            
            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 1.0,  # Process once per second
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse

            # Event system
            # Simple format: just the processor function (uses default sample_window)
            'pulse_processor': self.process_pulse,
            'event_processor': self.process_event,
            
            # Alternative format with custom sample windows:
            # 'pulse_processor': {
            #     'processor': self.process_pulse,
            #     'sample_window': [-2.0, 0.5]  # Custom window for pulse events
            # },
            # 'event_processor': {
            #     'processor': self.process_event,
            #     'sample_window': [-1.5, 0.3]  # Custom window for other events
            # },
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically."""

        # Trigger TMS device after 5ms delay
        trigger_time = reference_time + 0.005

        print(f"Creating trigger for time {trigger_time} seconds")

        return {
            'timed_trigger': trigger_time,
        }

    def process_event(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process event."""
        print(f"Event received at time {reference_time}.")
        # This example doesn't process events, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}.")
        # Add your pulse event handling logic here
        return None
