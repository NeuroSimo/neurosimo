from typing import Any, Dict, List, Optional, Union
import numpy as np


class Decider:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

        print("Decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> Dict[str, Union[int, bool, List]]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1.0, 0.0],
            
            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 2.0,  # Process every 2 seconds
            
            # Event system
            'event_processors': {
                'pulse': self.process_pulse,
            },
            
            # Predefined sensory stimuli (sent at session start)
            'predefined_sensory_stimuli': [
                # Welcome message at session start
                {
                    'time': 0.5,
                    'type': 'text_message',
                    'parameters': {
                        'text': 'Session starting...',
                        'duration': 2.0
                    }
                },
                # Visual cue at 3 seconds
                {
                    'time': 3.0,
                    'type': 'visual_cue',
                    'parameters': {
                        'color': 'green',
                        'size': 0.3,
                        'duration': 1.0,
                        'position_x': 0,
                        'position_y': 0
                    }
                },
                # Another visual cue at 5 seconds
                {
                    'time': 5.0,
                    'type': 'visual_cue',
                    'parameters': {
                        'color': 'blue',
                        'size': 0.2,
                        'duration': 0.5,
                        'position_x': -0.5,
                        'position_y': 0
                    }
                },
            ],
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG/EMG buffer periodically."""
        print(f"Periodic processing at time {reference_time:.1f}s")

        # Example: Send dynamic sensory stimuli based on processing time
        # Every 4 seconds (at 2s, 6s, 10s, etc. when we process at 2s intervals)
        if int(reference_time) % 4 == 0:
            print(f"Sending dynamic visual cue at {reference_time:.1f}s")
            return {
                'sensory_stimuli': [
                    {
                        'time': reference_time + 0.5,  # 0.5 seconds from now
                        'type': 'visual_cue',
                        'parameters': {
                            'color': 'red',
                            'size': 150,
                            'duration': 1.5,
                            'position_x': 0,
                            'position_y': 0
                        }
                    },
                    {
                        'time': reference_time + 2.0,  # 2 seconds from now
                        'type': 'text_message',
                        'parameters': {
                            'text': f'Processing at {reference_time:.1f}s',
                            'duration': 1.0
                        }
                    }
                ]
            }

        return None

    def process_eeg_trigger(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG trigger from the EEG device."""
        print(f"EEG trigger received at time {reference_time}.")
        # This example doesn't process EEG triggers, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}.")
        
        # Send a text message when pulse occurs
        return {
            'sensory_stimuli': [
                {
                    'time': reference_time + 0.1,  # Shortly after pulse
                    'type': 'text_message',
                    'parameters': {
                        'text': 'Pulse delivered!',
                        'duration': 1.0
                    }
                }
            ]
        }

