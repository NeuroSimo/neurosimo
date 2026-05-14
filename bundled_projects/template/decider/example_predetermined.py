from typing import Any

import random

import numpy as np


class Decider:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency

        # Seed RNG from subject_id for reproducibility
        self.rng = random.Random(subject_id)

        print(f"Decider initialized for subject {subject_id} with sampling frequency {sampling_frequency} Hz.")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            'sample_window': [-1.0, 0.0],
            'warm_up_rounds': 2,
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, stage_name: str, trial_count: int, is_warm_up: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically (for periodic trials)."""
        trigger_offset = 0.005  # 5ms delay
        print(f"[periodic] Creating trigger for +{trigger_offset * 1000:.1f} ms")

        return {
            'trigger_offset': trigger_offset,
        }

    def process_predetermined(
            self, reference_time: float, stage_name: str, trial: int, trial_type: str) -> dict[str, Any] | None:
        """Return the next trial for predetermined timing.

        Called once per predetermined trial when the trial counter changes.

        Args:
            reference_time: Current sample time in seconds since recording start.
            stage_name: Name of the current protocol stage.
            trial: Current trial index within the stage (0-based).
            trial_type: Stimulation type string from the protocol (e.g. "low_iti" or "high_iti").

        Returns:
            Dictionary with 'trigger_offset' or 'targeted_pulses' key,
            same format as process_periodic.
        """
        # Randomize inter-trial interval based on trial type
        if trial_type == 'low_iti':
            iti = self.rng.uniform(3.0, 5.0)
        elif trial_type == 'high_iti':
            iti = self.rng.uniform(5.0, 7.0)
        else:
            assert False, f"Unknown trial type: {trial_type}"

        print(f"[predetermined] Trial {trial} in '{stage_name}' (type={trial_type}): "
              f"scheduling trigger at +{iti:.2f} s")

        return {
            'trigger_offset': iti,
        }
