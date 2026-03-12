from typing import Any

import time

import numpy as np


class Decider:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency
        
        print("Decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def test_sample_window_coverage(self, time_offsets: np.ndarray, eeg_buffer: np.ndarray,
                                   emg_buffer: np.ndarray, sample_window: list[float], reference_time: float, window_name: str) -> None:
        """
        Test that sample windows are correctly configured and timestamps span the full range.

        Args:
            time_offsets: Array of time offsets relative to reference time
            eeg_buffer: EEG data buffer (channels x samples)
            emg_buffer: EMG data buffer (channels x samples)
            sample_window: [start_time, end_time] in seconds
            window_name: Descriptive name for the window (for error messages)
        """

        # Test that timestamps span the full sample window range
        assert len(time_offsets) > 0, f"{window_name}: Time offsets array is empty"
        assert np.isclose(time_offsets[0], sample_window[0], rtol=1e-10), f"{window_name}: First time offset {time_offsets[0]} doesn't match window start {sample_window[0]}"
        assert np.isclose(time_offsets[-1], sample_window[1], rtol=1e-10), f"{window_name}: Last time offset {time_offsets[-1]} doesn't match window end {sample_window[1]}"

        # Test that samples correspond to timestamps
        expected_samples = len(time_offsets)
        assert eeg_buffer.shape[0] == expected_samples, f"{window_name}: EEG buffer has {eeg_buffer.shape[0]} samples, expected {expected_samples}"
        assert eeg_buffer.shape[1] == self.num_eeg_channels, f"{window_name}: EEG buffer has {eeg_buffer.shape[1]} channels, expected {self.num_eeg_channels}"
        assert emg_buffer.shape[0] == expected_samples, f"{window_name}: EMG buffer has {emg_buffer.shape[0]} samples, expected {expected_samples}"
        assert emg_buffer.shape[1] == self.num_emg_channels, f"{window_name}: EMG buffer has {emg_buffer.shape[1]} channels, expected {self.num_emg_channels}"

        # Verify timestamps are evenly spaced according to sampling frequency
        if len(time_offsets) > 1:
            expected_dt = 1.0 / self.sampling_frequency
            actual_dt = np.diff(time_offsets)
            assert np.allclose(actual_dt, expected_dt, rtol=1e-6), f"{window_name}: Time offsets not evenly spaced at {self.sampling_frequency}Hz"

        # Verify each channel in the buffers contains values equal to the timestamps
        for ch in range(self.num_eeg_channels):
            assert np.allclose(eeg_buffer[:, ch], time_offsets + reference_time, rtol=1e-10), \
                f"{window_name}: EEG buffer channel {ch} values don't match time offsets {time_offsets + reference_time}"
        for ch in range(self.num_emg_channels):
            assert np.allclose(emg_buffer[:, ch], time_offsets + reference_time, rtol=1e-10), \
                f"{window_name}: EMG buffer channel {ch} values don't match time offsets {time_offsets + reference_time}"

        print(f"✓ {window_name} validation passed: {expected_samples} samples from {sample_window[0]}s to {sample_window[1]}s")

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Data configuration
            'sample_window': [-1.0, -0.1],
            'warm_up_rounds': 0,  # Number of warm-up rounds to perform (0 to disable)

            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 1.0,  # Process once per second
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse

            # Event system
            # Simple format: just the processor function (uses default sample_window)
            'pulse_processor': self.process_pulse,
            'event_processor': self.process_event,
            
            # Alternative format with custom sample windows:
            'pulse_processor': {
                'processor': self.process_pulse,
                'sample_window': [0.2, 0.6]  # Custom window for pulse events
            },
            'event_processor': {
                'processor': self.process_event,
                'sample_window': [0.0, 1.0]  # Custom window for other events
            },
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            is_coil_at_target: bool, is_warm_up: bool) -> dict[str, Any] | None:
        """Process EEG/EMG buffer periodically."""

        # Get sample window from configuration
        config = self.get_configuration()
        sample_window = config['sample_window']

        # Test sample window coverage
        self.test_sample_window_coverage(time_offsets, eeg_buffer, emg_buffer, sample_window, reference_time, "Periodic processing")

        return None

    def process_event(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process event."""

        # Get sample window from configuration
        config = self.get_configuration()
        sample_window = config['event_processor']['sample_window']

        # Test sample window coverage
        self.test_sample_window_coverage(time_offsets, eeg_buffer, emg_buffer, sample_window, reference_time, "Event processing")

        print(f"Event received at time {reference_time}.")

        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> dict[str, Any] | None:
        """Process pulse event."""

        # Get sample window from configuration
        config = self.get_configuration()
        sample_window = config['pulse_processor']['sample_window']

        # Test sample window coverage
        self.test_sample_window_coverage(time_offsets, eeg_buffer, emg_buffer, sample_window, reference_time, "Pulse processing")

        print(f"Pulse event received at time {reference_time}.")

        return None
