from typing import Any, Dict, List, Optional, Union
import multiprocessing
import time

import numpy as np


class Decider:
    def __init__(self, num_of_eeg_channels: int, num_of_emg_channels: int, sampling_frequency: float):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency
        
        # Initialize multiprocessing pool for background computations
        self.pool = multiprocessing.Pool(processes=1)

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

        print("Decider initialized with sampling frequency: ", sampling_frequency, "Hz")

    def get_configuration(self) -> Dict[str, Union[int, bool, List]]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1000, 0],
            
            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 1.0,  # Process once per second
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse

            # Event system
            'event_processors': {
                'pulse': self.process_pulse,
            },
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, valid_samples: np.ndarray,
            ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG/EMG buffer periodically."""
        print(f"Periodic processing at time {reference_time}.")

        if not np.all(valid_samples):
            return None

        return {
            # Trigger TMS device after 5ms delay
            'timed_trigger': reference_time + 0.005,
        }

    def process_eeg_trigger(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, valid_samples: np.ndarray,
            ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG trigger from the EEG device."""
        print(f"EEG trigger received at time {reference_time}.")
        # This example doesn't process EEG triggers, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, valid_samples: np.ndarray,
            ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}.")
        # Add your pulse event handling logic here
        return None
