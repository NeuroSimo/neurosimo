from typing import Any

import multiprocessing
import time

import numpy as np


class Decider:
    def __init__(self, subject_id: int, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
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

            # Optional: custom sample windows for pulse/event processing
            # (defaults to sample_window if omitted)
            # 'pulse_sample_window': [-2.0, 0.5],
            # 'event_sample_window': [-1.5, 0.3],
        }

    def prepare_trial(self, stage_name: str, trial_in_stage: int) -> None:
        """Called once at the beginning of a new trial."""
        print(f"Preparing trial {trial_in_stage} in '{stage_name}'")

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, stage_name: str, trial_in_stage: int, is_warm_up: bool) -> dict[str, Any] | None:
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
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool, stage_name: str, trial_in_stage: int) -> dict[str, Any] | None:
        """Process event."""
        print(f"Event received at time {reference_time}.")
        # This example doesn't process events, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool, stage_name: str, trial_in_stage: int) -> dict[str, Any] | None:
        """Process pulse event."""
        print(f"Pulse event received at time {reference_time}.")
        # Add your pulse event handling logic here
        return None
