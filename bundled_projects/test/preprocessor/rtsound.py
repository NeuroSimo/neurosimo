"""
rtSOUND preprocessor for NeuroSimo real-time EEG processing.

Copied to be a part of NeuroSimo test-cases from the original repository, which is licensed under GPL-3.0, and modified to be deterministic in its
output for testing purposes. Date of copying: 2026-03-12.

Original repository:

https://github.com/matildamakkonen/rt-sound-neurosimo-plugin
"""

from typing import Any

import time

import numpy as np


class Preprocessor:
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        self.subject_id = subject_id
        self.num_eeg_channels = num_eeg_channels
        self.num_emg_channels = num_emg_channels
        self.sampling_frequency = sampling_frequency

        # Load the lead field matrix from a CSV file
        #
        # IMPORTANT! Change the file leadfield.csv to a lead field matrix compatible with your real-time streaming data
        self.lead_field_matrix = np.genfromtxt('leadfield.csv', delimiter=',')

        # Check that the lead field matrix is a square matrix
        if self.lead_field_matrix.shape[0] != self.lead_field_matrix.shape[1]:
            raise ValueError(f"Lead field matrix is not a square matrix ({self.lead_field_matrix.shape[0]}x{self.lead_field_matrix.shape[1]})")

        # Check that the number of EEG channels is compatible with the lead field matrix
        if self.num_eeg_channels != self.lead_field_matrix.shape[1]:
            raise ValueError(f"Number of EEG channels ({self.num_eeg_channels}) is not compatible with the lead field matrix ({self.lead_field_matrix.shape[1]})")

        # User-changeable parameters for the SOUND algorithm
        self.num_iterations = 10
        self.lambda0 = 0.1
        self.convergence_boundary = 0.01
        self.baseline_update_rate = 0.0006
        self.pulse_artifact_duration = 1.0  # Duration in seconds to mark samples invalid after pulse
        self.sigmas_update_coefficient = 0.05  # Smooth sigmas update coefficient for SOUND updating
        self.sound_update_interval_samples = 0.1 * self.sampling_frequency  # Update interval for the SOUND filter (every 100 ms)
        self.verbose = False  # Verbosity level

        # Sigma values for the first SOUND run
        self.sigmas = np.ones((self.num_eeg_channels, 1))

        # Regularized lead-field matrix for the SOUND function
        self.LL = self.lead_field_matrix @ (self.lead_field_matrix.T)
        self.regularization_term = self.lambda0 * np.trace(self.LL) / self.num_eeg_channels
        self.LL_reg = self.LL / self.regularization_term

        # Initialize state
        self.filter = np.identity(self.num_eeg_channels)
        self.samples_collected = 0
        self.baseline_correction = None

        # Track pulse artifacts
        self.ongoing_pulse_artifact = False
        self.samples_after_pulse = 0

    def get_configuration(self) -> dict[str, Any]:
        """Return configuration dictionary for the pipeline."""
        return {
            # Configure the length of sample window to 0.1 seconds
            'sample_window': [-0.1, 0.0],
        }

    def process(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray,
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray,
            pulse_given: bool) -> dict[str, np.ndarray | bool]:
        """Process incoming EEG/EMG samples and return preprocessed data."""

        # Initialize baseline correction value if not already done
        if self.baseline_correction is None:
            self.baseline_correction = np.mean(eeg_buffer, 0)

        # If a pulse was given, mark an ongoing pulse artifact
        if pulse_given:
            self.ongoing_pulse_artifact = True
            self.samples_after_pulse = 0
            if self.verbose:
                print("A pulse was given.")

        # Mark samples as invalid after pulse for the duration of the pulse artifact
        if self.ongoing_pulse_artifact:
            self.samples_after_pulse += 1
            if self.samples_after_pulse >= self.pulse_artifact_duration * self.sampling_frequency:
                self.ongoing_pulse_artifact = False

        # Check if the SOUND filter needs to be updated
        self.samples_collected += 1
        should_update_filter = not self.ongoing_pulse_artifact and self.samples_collected % self.sound_update_interval_samples == 0

        # Update the SOUND filter synchronously to keep processing deterministic.
        if should_update_filter:
            new_filter, new_sigmas = compute_sound_filter(
                eeg_buffer.copy(),
                self.baseline_correction,
                self.sigmas,
                self.num_eeg_channels,
                self.lead_field_matrix,
                self.LL_reg,
                self.num_iterations,
                self.lambda0,
                self.sigmas_update_coefficient,
                self.convergence_boundary,
                self.verbose,
            )
            self.filter = new_filter
            self.sigmas = new_sigmas
        
        # Update baseline correction
        self.baseline_correction = self.baseline_update_rate * eeg_buffer[reference_index, :] + (1 - self.baseline_update_rate) * self.baseline_correction

        # Baseline correct the most recent sample
        eeg_sample = eeg_buffer[reference_index, :] - self.baseline_correction
        eeg_sample_preprocessed = eeg_sample @ self.filter.T

        emg_sample = emg_buffer[reference_index, :]

        return {
            'eeg_sample': eeg_sample_preprocessed,
            'emg_sample': emg_sample,
            'valid': not self.ongoing_pulse_artifact,
        }

def compute_sound_filter(eeg_buffer, baseline_correction, sigmas, num_eeg_channels, lead_field_matrix, LL_reg, num_iterations, lambda0, sigmas_update_coefficient, convergence_boundary, verbose):
    # If there are no channels, return an empty filter.
    if num_eeg_channels == 0:
        return np.identity(0), np.identity(0)

    # Actual baseline correction for EEG buffer
    eeg_buffer = eeg_buffer - baseline_correction
    eeg_buffer = eeg_buffer.T

    if verbose:
        start = time.time()

    # Perform SOUND algorithm for the given data
    n0, _ = eeg_buffer.shape
    eeg_buffer = np.reshape(eeg_buffer, (n0, -1))

    # Run beamformer SOUND
    #
    # See Metsomaa et al. 2024 Brain Topography for equations

    # Estimate the data covariance matrix as sample covariance
    data_cov = np.matmul(eeg_buffer, eeg_buffer.T) / (eeg_buffer.shape[1] - 1)

    # Save the previous sigma values before the new iteration:
    sigmas_previous_update = np.copy(sigmas)

    # Iterate over channels
    for k in range(num_iterations):
        # Save the previous sigma values
        old_sigmas = np.copy(sigmas)

        # Update noise estimate values
        gamma = np.linalg.pinv(LL_reg + np.diagflat(np.square(sigmas)))  # Eq. 18 in Metsomaa et al. 2024
        sigmas = [(gamma[:, i] / gamma[i, i]).T @ (data_cov @ (gamma[:, i] / gamma[i, i])) for i in range(num_eeg_channels)]  # Eq. 20 in Metsomaa et al. 2024
        
        # Compute the maximum noise estimate change for checking convergence
        max_noise_estimate_change = np.max(np.abs(old_sigmas - sigmas) / old_sigmas)
        if verbose:
            print("Max noise estimate change: {}".format(max_noise_estimate_change))

        # Terminate the iteration if the convergence boundary is reached
        if max_noise_estimate_change < convergence_boundary:
            if verbose:
                print("Convergence reached after {} iterations".format(k+1))
            break

    # Make sure sigmas is a numpy array
    sigmas = np.array(sigmas, dtype=np.float32)
    sigmas = np.expand_dims(sigmas, axis=1)

    # Change sigmas smoothly
    sigmas = sigmas_update_coefficient * sigmas + (1 - sigmas_update_coefficient) * sigmas_previous_update

    # Final data correction based on the final noise-covariance estimate
    # Calculate matrices needed for SOUND spatial filter
    W = np.diag(1.0 / np.squeeze(sigmas))
    WL = np.matmul(W, lead_field_matrix)
    WLLW = np.matmul(WL, WL.T)
    C = (WLLW + lambda0 * np.trace(WLLW) / num_eeg_channels * np.eye(num_eeg_channels))
    new_filter = np.matmul(lead_field_matrix, np.matmul(WL.T, np.linalg.solve(C, W)))

    if verbose:
        # Find the best-quality channel
        best_channel = np.argmin(sigmas)

        # Calculate the relative error in the best channel caused by SOUND overcorrection:
        relative_error = np.linalg.norm(new_filter[best_channel, :] @ eeg_buffer - eeg_buffer[best_channel, :]) / np.linalg.norm(eeg_buffer[best_channel, :])
        print("Relative error in the best channel: {}".format(relative_error))

        end = time.time()
        print("SOUND update time: {:.1f} ms".format(10 ** 3 * (end - start)))

    return new_filter, sigmas
