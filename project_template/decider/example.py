import multiprocessing
import time
from enum import Enum

import numpy as np

class Event(Enum):
    PREPULSE = "prepulse"
    POSTPULSE = "postpulse"


class Decider:
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency
        
        # Leave empty when mTMS device is not used
        self.targets = []
        
        # Initialize multiprocessing pool for background computations
        self.pool = multiprocessing.Pool(processes=1)

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

    def get_configuration(self):
        """Return configuration dictionary for the pipeline."""
        return {
            'processing_interval_in_samples': self.sampling_frequency,  # Process once per second
            'process_on_trigger': False,
            'sample_window': [-1000, 0],
            'events': [],
            'sensory_stimuli': [],
        }

    def process(self, current_time, timestamps, valid_samples, eeg_buffer, emg_buffer, 
               current_sample_index, ready_for_trial, is_trigger, is_event, event_type, is_coil_at_target):
        """Process EEG/EMG samples and decide whether to trigger stimulation."""
        print(f"Processing sample at time {current_time}.")

        if is_event:
            print(f"Event received: {event_type}")

        if not np.all(valid_samples):
            return

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
