from typing import Any

import random

import numpy as np


class Decider:
    def __init__(self, subject_id: int, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
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

    def prepare_trial(self, start_time: float, stage_name: str, trial_in_stage: int) -> dict[str, Any] | None:
        """Called once at the beginning of a new trial.

        For the 'predetermined_block' stage, returns a trigger_offset to schedule
        the trigger upfront (no periodic processing). For other stages, returns
        None to fall through to process_periodic.
        """
        if stage_name == 'predetermined_block':
            iti = self.rng.uniform(3.0, 7.0)
            print(f"[prepare_trial] Trial {trial_in_stage} in '{stage_name}': "
                  f"scheduling trigger at +{iti:.2f} s")
            return {'trigger_offset': iti}

        print(f"[prepare_trial] Trial {trial_in_stage} in '{stage_name}': periodic")
        return None

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, stage_name: str, trial_in_stage: int, is_warm_up: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically (for periodic trials)."""
        trigger_offset = 0.005  # 5ms delay
        print(f"[periodic] Creating trigger for +{trigger_offset * 1000:.1f} ms")

        return {
            'trigger_offset': trigger_offset,
        }
