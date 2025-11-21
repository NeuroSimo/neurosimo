from typing import Any, Dict, List, Optional, Union
import multiprocessing
import time
from enum import Enum

import numpy as np

class Event(Enum):
    PREPULSE = "prepulse"
    POSTPULSE = "postpulse"


class Decider:
    def __init__(self, num_of_eeg_channels: int, num_of_emg_channels: int, sampling_frequency: float):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency
        
        # Leave empty when mTMS device is not used
        self.targets = []
        
        # Initialize multiprocessing pool for background computations
        self.pool = multiprocessing.Pool(processes=1)

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

        print("Decider initialized with sampling frequency: ", sampling_frequency, "Hz")

    def get_configuration(self) -> Dict[str, Union[int, bool, List]]:
        """Return configuration dictionary for the pipeline."""
        return {
            'periodic_processing_interval': 1.0,  # Process once per second
            'process_on_trigger': False,
            'sample_window': [-1000, 0],
            'events': [],
            'sensory_stimuli': [],
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse
        }

    def process(self, current_time: float, timestamps: np.ndarray, valid_samples: np.ndarray, 
               eeg_buffer: np.ndarray, emg_buffer: np.ndarray, 
               current_sample_index: int, ready_for_trial: bool, 
               is_event: bool, event_type: str, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG/EMG samples and decide whether to trigger stimulation."""
        print(f"Processing sample at time {current_time}.")

        if is_event:
            print(f"Event received: {event_type}")

        if not np.all(valid_samples):
            return None

        return {
            # Trigger TMS device after 5ms delay
            'timed_trigger': current_time + 0.005,

            # Example dynamic sensory stimulus
            # 'sensory_stimuli': [
            #     {
            #         'time': current_time + 1.0,
            #         'type': 'visual_cue',
            #         'parameters': {
            #             'color': 'blue',
            #             'intensity': 0.8,
            #             'duration': 0.2
            #         }
            #     }
            # ]
        }

    def process_eeg_trigger(self, current_time: float, timestamps: np.ndarray, 
                           valid_samples: np.ndarray, eeg_buffer: np.ndarray, 
                           emg_buffer: np.ndarray, current_sample_index: int, 
                           ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Handle EEG trigger from the EEG device."""
        print(f"EEG trigger received at time {current_time}.")
        # This example doesn't process EEG triggers, just log them
        return None
