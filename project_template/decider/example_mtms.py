from typing import Any, Dict, List, Optional, Union
import multiprocessing
import time

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
        predefined_events = [
            {
                'type': 'pulse',
                'time': 2.0,
            },
            {
                'type': 'pulse',
                'time': 3.0,
            },
        ]

        return {
            # Data configuration
            'sample_window': [-5, 0],
            
            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 1.0,  # Process once per second
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse
            
            # Event system
            'predefined_events': predefined_events,
            'event_handlers': {
                'pulse': self.handle_pulse,
            },
            
            # Sensory stimuli
            'predefined_sensory_stimuli': [],
        }

    def process(self, current_time: float, timestamps: np.ndarray, valid_samples: np.ndarray, 
               eeg_buffer: np.ndarray, emg_buffer: np.ndarray, 
               current_sample_index: int, ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG/EMG buffer (periodic processing)."""
        print("Periodically processing EEG/EMG buffer at time {:.4f}".format(current_time))

        self.buffer_count += 1

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

    def handle_pulse(self, current_time: float, timestamps: np.ndarray, 
                    valid_samples: np.ndarray, eeg_buffer: np.ndarray, 
                    emg_buffer: np.ndarray, current_sample_index: int, 
                    ready_for_trial: bool, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Handle pulse event."""
        print("Pulse event received at time {:.4f}".format(current_time))
        # Add your pulse event handling logic here
        return None
