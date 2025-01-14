# An example decider that sends a trigger to the TMS device to deliver a pulse once every two seconds.

# Native Python modules and other third-party libraries can be imported here. Of third-party libraries,
# the following are currently available:
#
#  - numpy
#  - scipy
#  - scikit-learn
#  - statsmodels
#  - mneflow
#
# If more third-party libraries are needed, they can be installed by adding them in
# ros2_ws/src/neurosimo_packages/pipeline/decider/Dockerfile, and running 'build-neurosimo' on the
# command line after modifying the file.

import multiprocessing
import time
from enum import Enum

import numpy as np

class Event(Enum):
    PREPULSE = 1
    POSTPULSE = 2


class Decider:
    # When the decider is initialized, the number of EEG and EMG channels and the sampling frequency are automatically
    # provided by the pipeline. They are determined based on how the EEG device is configured.
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):

        # Store the number of EEG and EMG channels and the sampling frequency for potential later use.
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        # Leave the targets empty when the mTMS device is not used.
        self.targets = []

        # Initialize a multiprocessing pool with one process. The pool is used so that EEG/EMG sample stream processing can be
        # continued while a long-standing computation is performed in the background.
        self.pool = multiprocessing.Pool(processes=1)

    def get_configuration(self):
        """The 'get_configuration' method is called by the pipeline when the decider is initialized. The method should return
        a dictionary with the following keys:

        - 'processing_interval_in_samples': An integer indicating how frequently the process method is called. The pipeline
          uses this value to determine how often the process method is called. For example, if the value is 100, the process
          method is called every 100 samples.

          There are two special cases:

          1) When 'processing_interval_in_samples' is set to 1, the process method is called for every
             new EEG/EMG sample. This is useful when the decider needs to continuously monitor the EEG/EMG
             stream.

          2) When 'processing_interval_in_samples' is set to 0, the process method is not called periodically at all.
             However, it may still be called if other conditions are met (currently, if process_on_trigger is set to True).

        - 'process_on_trigger': A boolean indicating whether the process method is called each time a trigger is received.
           If set to True, the process method is called each time a trigger is received, in addition to the periodic calls
           defined by 'processing_interval_in_samples'.

        - 'sample_window': A list with two elements indicating the earliest and latest samples kept in the buffer relative
          to the current sample. The buffer is managed by the pipeline and is used to store the previous EEG and EMG samples.

          The sample window is defined in samples, not in seconds. The earliest sample is always negative. The current sample
          is always 0.

          Examples:

           - If the sample window is [-5, 0], the buffer will keep the latest 5 samples and the current
             sample, for a total of 6 samples. The buffer will discard older samples.

           - Using a sampling frequency of 1000 Hz, defining the sample window as [-999, 0] will keep the
             last second of samples in the buffer.

           - If the sample window is [0, 0], the buffer will keep only the current sample.

           - If the sample window is [-5, -5], the buffer will keep the 5 past samples but will also 'look'
             5 samples into the future. In practice, this means that the incoming EEG/EMG samples will be
             delayed by 5 samples. This can be useful, e.g., for various kinds of filtering.

        - 'events': A list of dictionaries, each dictionary with the following keys:
           - 'type': An integer indicating the type of the event.
           - 'time': A float indicating the time of the event in seconds, relative to the start of the session.

           Events can be used to trigger processing in the decider, in addition to the periodic processing defined by
           'processing_interval_in_samples' and triggers received from the EEG device (if 'process_on_trigger' is set
           to True).

           For example, an event can be used to trigger processing prior to a pre-defined pulse time so that the decider
           can then decide whether to perform a trial at that time, or alternatively always perform a trial at a fixed time
           after the event.

           Events can also be used to trigger post-pulse processing, such as logging or model training based on the received
           EEG/EMG samples after the pulse.

           If an empty list or not provided, the pipeline will not trigger processing based on events.
        """
        return {
            'processing_interval_in_samples': self.sampling_frequency,  # Process once per second
            'process_on_trigger': False,
            'sample_window': [-1000, 0],
            'events': [],
        }

    def process(self, current_time, timestamps, valid_samples, eeg_buffer, emg_buffer, current_sample_index, ready_for_trial, is_trigger, is_event, event_type):
        """The 'process' method is called by the pipeline for new EEG/EMG samples with a specified interval (see get_configuration method).
        This method receives the following arguments:

           - current_time:
               The time of the current sample in seconds.

           - timestamps:
               A list of timestamps for the EEG/EMG samples; a NumPy array of shape (num_of_samples,), where num_of_samples
               is the number of samples in the buffer defined by self.sample_window (e.g, 6 if the sample window is [-5, 0]).

          - valid_samples:
               A list of booleans indicating whether the EEG/EMG samples are valid; a NumPy array of shape (num_of_samples,).
               The validity of the samples is determined on a sample-by-sample basis by the preprocessor of the pipeline.

               The example preprocessor (used as the default) marks all samples as valid, except for one second after
               each pulse.

          - eeg_buffer:
               A NumPy array of shape (num_of_samples, num_of_eeg_channels), where num_of_eeg_channels is the number of EEG
               channels and num_of_samples is the number of samples in the buffer defined by self.sample_window.

          - emg_buffer:
               A NumPy array of shape (num_of_samples, num_of_emg_channels), where num_of_emg_channels is the number of EMG
               channels and num_of_samples is the number of samples in the buffer defined by self.sample_window.

          - current_sample_index:
               The index of the current sample in the buffer. It always points to the last sample of 'eeg_buffer' and
               'emg_buffer' buffers if self.sample_window is set to [-n, 0]. However, if self.sample_window is set,
               e.g., to [-10, 5], current_sample_index would be 10 while the buffers would contain 16 samples.

               Note that this is redundant information, as the current sample can be calculated from self.sample_window,
               but it is provided for convenience.

          - ready_for_trial:
               A boolean indicating whether the pipeline is ready to perform a trial. The pipeline is not ready for a trial
               if the previous trial is still ongoing or if the pipeline is in the process of precomputing the targets.

               For instance, if the pipeline has just started, it will not be ready for a trial until all the targets are
               precomputed, which may take 10-20 seconds for each target.

               Another example is when the decider has decided to perform a trial, say, 0.5 seconds after the current sample.
               During these 0.5 seconds, the pipeline is not ready for another trial, and ready_for_trial will be False.

               The decider should not perform a trial if the pipeline is not ready. However, it can do other processing, such
               as logging, filtering, computing metrics, or training models. If the decider still decides to perform a trial,
               the pipeline will print a warning and ignore the trial.

          - is_trigger:
                A boolean indicating whether a trigger was received on that sample. The trigger comes from the EEG device and
                it cannot currently be used with simulated data. However, events behave similarly to triggers and can be used
                for the same purpose.

          - is_event:
               A boolean indicating whether a event was received on that sample. See get_configuration method for more information
               on events.

          - event_type:
               An unsigned integer indicating the event type. See get_configuration method for more information on events."""

        print(f"Processing sample at time {current_time}.")

        if not np.all(valid_samples):
            return

        return {
            # In addition to performing a trial using the mTMS device, the decider can create timed triggers
            # using LabJack T4. This is useful for triggering commercial TMS devices or other
            # devices that require a trigger signal, and when the mTMS device is not available.
            'timed_trigger': current_time + 0.005,
        }
