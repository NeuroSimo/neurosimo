from typing import Any, Dict, List, Optional, Union
import multiprocessing
import time
from enum import Enum

import numpy as np

# Define target types for mTMS device
LOW_INTENSITY_TARGET = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 20,
        'algorithm': 'least_squares',
    },
]

HIGH_INTENSITY_TARGET = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 30,
        'algorithm': 'least_squares',
    },
]

PAIRED_PULSE_TARGET = [
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
    },
]

class Event(Enum):
    PREPULSE = 1
    POSTPULSE = 2

# Configure mTMS trigger outputs
TRIGGERS = [
    {
        'enabled': True,
        'delay': 0.0,
    },
    {
        'enabled': True,
        'delay': 0.0,
    },
]

MINIMUM_DELAY_BEFORE_PULSE = 0.050  # seconds


class Decider:
    def __init__(self, num_of_eeg_channels: int, num_of_emg_channels: int, sampling_frequency: float):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        self.buffer_count = 0
        
        # Define targets for precomputation
        self.targets = [
            LOW_INTENSITY_TARGET,
            HIGH_INTENSITY_TARGET,
            PAIRED_PULSE_TARGET,
        ]

        # Track which target type to use next
        self.target_type = 0
        
        # Perform trial every 2 seconds (2 process calls at 1 second intervals)
        self.intertrial_interval_in_process_calls = 2

        # Initialize multiprocessing pool for background computations
        self.pool = multiprocessing.Pool(processes=1)

    def get_configuration(self) -> Dict[str, Union[int, bool, List]]:
        """Return configuration dictionary for the pipeline."""
        events = [
            {
                'type': Event.PREPULSE.value,
                'time': 2.0,
            },
            {
                'type': Event.POSTPULSE.value,
                'time': 3.0,
            },
        ]

        return {
            'periodic_processing_interval': 1.0,  # Process once per second
            'process_on_trigger': True,
            'sample_window': [-5, 0],
            'events': events,
            'sensory_stimuli': [],
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse
        }

    def process(self, current_time: float, timestamps: np.ndarray, valid_samples: np.ndarray, 
               eeg_buffer: np.ndarray, emg_buffer: np.ndarray, 
               current_sample_index: int, ready_for_trial: bool, 
               is_event: bool, event_type: str, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG/EMG samples and decide whether to trigger mTMS stimulation."""
        print("Processing EEG/EMG samples at time {:.4f}".format(current_time))

        self.buffer_count += 1

        if is_event:
            print("Event of type {} received at time {:.4f}".format(event_type, current_time))

        # Skip if pipeline not ready or samples invalid
        if not ready_for_trial or not np.all(valid_samples):
            return None

        # Perform trial at specified intervals
        if self.buffer_count % self.intertrial_interval_in_process_calls != 0:
            return None

        # Cycle through target types
        targets = self.targets[self.target_type]
        self.target_type = (self.target_type + 1) % len(self.targets)

        # Calculate pulse timing
        start_time = current_time + MINIMUM_DELAY_BEFORE_PULSE

        if len(targets) == 2:  # Paired-pulse
            pulse_times = [start_time, start_time + 0.1]
        else:  # Single pulse
            pulse_times = [start_time]

        print("Decided at time {:.4f} to perform trial at time {:.4f} with {} target(s)".format(
            current_time, start_time, len(targets)))

        trial = {
            'targets': targets,
            'pulse_times': pulse_times,
            'triggers': TRIGGERS,
        }

        return {'trial': trial}

    def process_eeg_trigger(self, current_time: float, timestamps: np.ndarray, 
                           valid_samples: np.ndarray, eeg_buffer: np.ndarray, 
                           emg_buffer: np.ndarray, current_sample_index: int, 
                           ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Handle EEG trigger from the EEG device."""
        print("EEG trigger received at time {:.4f}".format(current_time))
        # This example doesn't process EEG triggers, just log them
        return None
