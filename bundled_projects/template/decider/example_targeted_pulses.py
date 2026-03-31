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

        print("Decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1.0, 0.0],
            'warm_up_rounds': 2,  # Number of warm-up rounds to perform (0 to disable)

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
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, stage_name: str, pulse_count: int, is_warm_up: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically."""

        trigger_offset = 0.005
        print(f"Creating paired pulses for +{trigger_offset * 1000:.1f} ms")

        targeted_pulses = [
            {
                "time_offset": trigger_offset,  # seconds, relative to reference_time
                "displacement_x": 1,            # displacement along X (mm)
                "displacement_y": 2,            # displacement along Y (mm)
                "rotation_angle": 3,            # rotation angle (degrees)
                "intensity": 40,                # intensity value (V/m)
            },
            {
                "time_offset": trigger_offset + 0.050,
                "displacement_x": -1,
                "displacement_y": -2,
                "rotation_angle": 40,
                "intensity": 50,
            },
        ]
        
        return {
            "targeted_pulses": targeted_pulses,
        }

    def process_event(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool, stage_name: str, pulse_count: int) -> dict[str, Any] | None:
        """Process event."""
        print(f"Event received at time {reference_time}.")
        # This example doesn't process events, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool, stage_name: str, pulse_count: int) -> dict[str, Any] | None:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}.")
        # Add your pulse event handling logic here

        # Example: return a (paired) set of TargetedPulse dictionaries.
        # The C++ wrapper (`decider_wrapper.cpp`) expects each targeted pulse dict to contain:
        #   time_offset, displacement_x, displacement_y, rotation_angle, intensity
        #
        # paired_pulses = [
        #     {
        #         "time_offset": reference_time,      # seconds (see your pipeline conventions)
        #         "displacement_x": 0.010,    # displacement along X
        #         "displacement_y": 0.000,    # displacement along Y
        #         "rotation_angle": 0.0,      # radians
        #         "intensity": 1.0,           # intensity value per your system
        #     },
        #     {
        #         "time_offset": reference_time,      # same time -> "paired" spatial targeting
        #         "displacement_x": -0.010,
        #         "displacement_y": 0.000,
        #         "rotation_angle": 0.0,
        #         "intensity": 1.0,
        #     },
        # ]
        #
        # return {
        #     # Return targeted pulses via this key:
        #     "targeted_pulses": paired_pulses,
        #
        #     # Optional (only if your pipeline uses it):
        #     # "coil_target": "left_coil",
        # }
        return None
