# ------------------- NB! -----------------------------------
# This is a beta version of the online Sound algorithm and is not yet fully validated.
# The given lead field matrix LFM_Aalto_ReftepPP.csv only works with the Aalto Reftep++ 64-ch channel
# configuration (reference channel TP10). If your channel configuration is different, please make your own
# lead field or re-reference your data to TP10. If you are planning to use this code for experiments, please
# contact Tuomas Mutanen and Matilda Makkonen. The optimal parameters might be different for your use case.
# ------------------- NB! -----------------------------------

# Authors: Tuomas Mutanen, Olli-Pekka Kahilakoski, Matilda Makkonen, Johanna Metsomaa

import multiprocessing
import time

import numpy as np
import cpp_bindings

# Override Python's native print() function.
def print(x):
    cpp_bindings.log(str(x))


class Preprocessor:
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        # Load a generic average-head lead field tailored for the NeurOne data:
        self.lfm = np.genfromtxt('models/LFM_Aalto_ReftepPP.csv',delimiter=',') # works for Aalto Reftep++ data

        # -----------------------
        # SOUND parameters
        self.iterations = 10
        self.lambda0 = 0.1
        self.convergence_boundary = 0.01
        self.update_interval_in_samples = 0.1*self.sampling_frequency # 100 ms
        # initialize sigma values for first Sound run
        self.sigmas = np.ones((num_of_eeg_channels, 1))

        # baseline update rate for geometric weighting of samples:
        self.baseline_update_rate = 0.0006
        # -----------------------

        # Initialize state
        self.filter = np.identity(self.num_of_eeg_channels)
        self.updating_filter = False

        self.samples_collected = 0

        self.ongoing_pulse_artifact = False
        self.samples_after_pulse = 0

        # Initialize multiprocessing
        self.pool = multiprocessing.Pool(processes=2)
        self.result = None

        # Configure the length of sample window.
        self.sample_window = [-499, 0]

        # initialize baseline correction
        self.baseline_correction = 0

    def sleep(self, duration_us):
        duration = duration_us / 10 ** 6

        now = time.perf_counter()
        end = now + duration
        while now < end:
            now = time.perf_counter()

    def process(self, timestamps, eeg_samples, emg_samples, current_sample_index, pulse_given):
        self.samples_collected += 1

        if self.samples_collected == 1:
            self.baseline_correction = np.mean(eeg_samples,0)

        if pulse_given:
            self.ongoing_pulse_artifact = True
            self.samples_after_pulse = 0
            print("A pulse was given.")

        # Assuming that an ongoing artifact lasts for 5000 samples; after that, reset the flag.
        if self.ongoing_pulse_artifact:
            self.samples_after_pulse += 1
            if self.samples_after_pulse == 5000:
                self.ongoing_pulse_artifact = False

        if self.result is None and \
            self.samples_collected % self.update_interval_in_samples == 0 and \
            not self.ongoing_pulse_artifact:

            self.result = self.pool.apply_async(sound, (
                eeg_samples,
                self.baseline_correction,
                self.sigmas,
                self.num_of_eeg_channels,
                self.lfm,
                self.iterations,
                self.lambda0,
                self.convergence_boundary,
            ))

        if self.result is not None:
            try:
                new_filter, new_sigmas = self.result.get(timeout=0)

                self.filter = new_filter
                self.sigmas = new_sigmas

                self.result = None

                print("Output: lambda = {}".format(self.lambda0))

            except multiprocessing.TimeoutError as e:
                pass

        # Define baseline correction for single-sample use
        self.baseline_correction = (1-self.baseline_update_rate)*self.baseline_correction + self.baseline_update_rate*eeg_samples[current_sample_index, :]

        # Baseline correct most recent sample:
        current_sample = eeg_samples[current_sample_index, :] - self.baseline_correction

        eeg_sample_preprocessed = current_sample @ self.filter.T
        emg_sample_preprocessed = emg_samples[current_sample_index, :]

        return {
            'eeg_sample': eeg_sample_preprocessed,
            'emg_sample': emg_sample_preprocessed,
            'valid': not self.ongoing_pulse_artifact,
        }


def sound(eeg_samples, baseline_correction, sigmas, num_of_channels, lfm, iterations, lambda0, convergence_boundary):
    # If there are no channels, return an empty filter.
    if num_of_channels == 0:
        return np.identity(0), lambda0, np.identity(0), np.identity(0), np.identity(0)

    # Actual baseline correction for Sound data buffer
    eeg_samples = eeg_samples - baseline_correction

    # Smooth sigmas update coeff:
    sigmas_update_coeff = 0.05

    # Performs the SOUND algorithm for a given data.

    data = eeg_samples.T

    start = time.time()

    n0, _ = data.shape
    data = np.reshape(data, (n0, -1))

    LL = lfm @ lfm.T
    dn = np.empty((iterations, 1)) # Empty vector for convergences

    #################### Run beamformer SOUND #####################################################
    # See Metsomaa et al. 2024 Brain Topography for equations

    dataCov = np.matmul(data, data.T) / data.shape[1] # Estimate the data covariance matrix as sample covariance

    # Estimate the neuronal covariance
    LL = lfm @ (lfm.T)
    regularization_term = lambda0*np.trace(LL) / num_of_channels
    LL_reg = LL / regularization_term

    # Save the previous sigma values before the new iteration:
    sigmas_prev_update = np.copy(sigmas)

    # Iterate over channels
    for k in range(iterations):
        # Save the previous sigma values
        sigmas_old = np.copy(sigmas)
        # Update noise estimate values
        GAMMA = np.linalg.pinv(LL_reg + np.diagflat(np.square(sigmas))) # Eq. 18 in Metsomaa et al. 2024
        sigmas = [(GAMMA[:, i] / GAMMA[i, i]).T @ (dataCov @ (GAMMA[:, i] / GAMMA[i, i])) for i in range(num_of_channels)] # Eq. 20 in Metsomaa et al. 2024

        # Following and storing the convergence of the algorithm
        max_noise_estimate_change = np.max(np.abs(sigmas_old - sigmas) / sigmas_old)
        #print("Output: Max noise estimate change = {}".format(max_noise_estimate_change))
        if max_noise_estimate_change < convergence_boundary: # terminates the iteration if the convergence boundary is reached
            print("Output: Convergence reached after {} iterations!".format(k+1))
            break

    # Make sure sigmas is a numpy array:
    sigmas = np.array(sigmas, dtype=np.float32)
    sigmas = np.expand_dims(sigmas,axis = 1)
    # Change sigmas smoothly:
    sigmas = sigmas_update_coeff*sigmas + (1 - sigmas_update_coeff)*sigmas_prev_update

    # Final data correction based on the final noise-covariance estimate.
    # Calculates matrices needed for SOUND spatial filter (for other functions)
    #SOUND_filter = LL @ np.linalg.inv(LL + regularization_term*C_noise)
    W = np.diag(1.0 / np.squeeze(sigmas))
    WL = np.matmul(W, lfm)
    WLLW = np.matmul(WL, WL.T)
    C = (WLLW + lambda0 * np.trace(WLLW) / num_of_channels * np.eye(num_of_channels))
    SOUND_filter = np.matmul(lfm, np.matmul(WL.T, np.linalg.solve(C, W)))

    # Check whether the regularization level is optimal and adjust with the
    # learning rate, when appropriate:

    # find the best-quality channel
    best_ch = np.argmin(sigmas)
    # Calculate the relative error in the best channel caused by SOUND overcorrection:
    rel_err = np.linalg.norm(SOUND_filter[best_ch,:]@data - data[best_ch,:])/np.linalg.norm(data[best_ch,:])
    print("Output: Relative error in best channel = {}".format(rel_err))

    end = time.time()
    print("Output: SOUND update time = {:.1f} ms".format(10 ** 3 * (end-start)))

    return SOUND_filter, sigmas
