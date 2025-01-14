# An example decider to use with the multi-locus (mTMS) device.
#
# Note: Not to be used with other TMS devices than the mTMS device. To use this Decider, NeuroSimo needs
# to be running concurrently with the mTMS device software (see https://github.com/connect2brain/mtms).
# The mTMS device must also be enabled in the environment configuration file (.env in the repository
# root directory).

# The decider alternates between different target types, all targeted
# at the same location (x = 0, y = 0, rotation = 0). The target types are as follows:
#
#   - Single pulse with low intensity (20 V/m).
#   - Single pulse with high intensity (30 V/m)
#   - Paired pulse with high intensity followed by low intensity.
#
# A trial with one of the above target types is performed once every two seconds.
#
# Note that if you want to perform trials more frequently, you need to reduce the value
# of MINIMUM_INTERTRIAL_INTERVAL in the environment configuration file (e.g., sites/Aalto/.env
# in the repository root directory) and restart the pipeline (e.g., by running `restart` on the
# command line), as 2.0 seconds is the default minimum intertrial interval for safety reasons.


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

# Define a few target types.
#
# Each target is a list of dictionaries, each dictionary with the following keys:
#  - 'displacement_x': The x-coordinate of the target in millimeters (between -15 and 15).
#  - 'displacement_y': The y-coordinate of the target in millimeters (between -15 and 15).
#  - 'rotation_angle': The rotation angle of the target in degrees (between 0 and 359).
#  - 'intensity': The intensity of the target in V/m.
#  - 'algorithm': The algorithm to use for computing the capacitor voltages based on the target.
#                 The available options are:
#
#     - 'least_squares': Use the least squares algorithm.
#     - 'genetic': Use the genetic algorithm.
#
# A single-pulse target consists of a single dictionary, while a paired-pulse target consists of two dictionaries in a list.

LOW_INTENSITY_TARGET = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 20,
        'algorithm': 'least_squares',
    },
]

HIGH_INTENSITY_TARGET = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 30,
        'algorithm': 'least_squares',
    },
]

PAIRED_PULSE_TARGET = [
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 30,
        'algorithm': 'least_squares',
    },
    {
        'displacement_x': 0,
        'displacement_y': 0,
        'rotation_angle': 0,
        'intensity': 20,
        'algorithm': 'least_squares',
    },
]

class Event(Enum):
    PREPULSE = 1
    POSTPULSE = 2


# The mTMS device allows coinciding the TMS pulse with one or two output triggers.
# The triggers can be configured to be enabled or disabled and to have a delay before activation.
#
# Each trigger is a dictionary with the following keys:
#
# - 'enabled': A boolean indicating whether the trigger is enabled.
# - 'delay': The delay in seconds before the trigger is activated. The delay can be 0.0, positive,
#            or negative. A positive delay means that the trigger is activated after the pulse,
#            while a negative delay means that the trigger is activated before the pulse.
TRIGGERS = [
    {
        'enabled': True,
        'delay': 0.0,
    },
    {
        'enabled': True,
        'delay': 0.0,
    },
]

# The minimum time between the currently processed EEG/EMG samples and the pulse, in seconds. If it is too
# short, the pulse may not be performed due to the end-to-end latency of the pipeline. The overall latency
# includes the following components:
#
# - The time it takes for the EEG device to filter and send the data.
# - The time it takes for the pipeline to preprocess the data (see preprocessor).
# - The time it takes for the decider to make a decision. This is greatly affected by the complexity
#   of the decision-making algorithm.
#
# In practice, 5 ms is a good value to start with when the pipeline does not do any significant processing.
# The delay should be adjusted based on the pipeline's processing time.
#
# Note that in this example, the pulse is timed exactly at the minimum delay, but in real applications,
# the algorithm implemented in the decider can also time it farther in the future.
MINIMUM_DELAY_BEFORE_PULSE = 0.050  # seconds


class Decider:
    # When the decider is initialized, the number of EEG and EMG channels and the sampling frequency are automatically
    # provided by the pipeline. They are determined based on how the EEG device is configured.
    def __init__(self, num_of_eeg_channels, num_of_emg_channels, sampling_frequency):

        # Store the number of EEG and EMG channels and the sampling frequency for potential later use.
        self.num_of_eeg_channels = num_of_eeg_channels
        self.num_of_emg_channels = num_of_emg_channels
        self.sampling_frequency = sampling_frequency

        # Initialize state variable to keep track of the number of EEG/EMG buffers processed so far.
        self.buffer_count = 0

        # To work in real-time, the pipeline will precompute the capacitor voltages and pulse parameters for the targets used
        # in the experiment. For that purpose, the decider needs to provide the pipeline with a list of targets via the
        # 'targets' attribute below.
        #
        # Any targets used in the experiment should be defined here. If a target is missing from the list, the pipeline will
        # compute it on-the-fly when needed, but that introduces a delay of 10-20 seconds before the trial is performed.
        self.targets = [
            LOW_INTENSITY_TARGET,
            HIGH_INTENSITY_TARGET,
            PAIRED_PULSE_TARGET,
        ]

        # Keep track of which target type to use next. This is for alternating between low intensity, high intensity, and
        # paired-pulse targets.
        self.target_type = 0

        # Perform a trial once every two seconds. (Note that the processing interval, defined in get_configuration method,
        # is 100 samples, so with a sampling frequency of 1000 Hz, the interval is 0.1 seconds, implying that performing the
        # trial every 20 process calls is equivalent to performing it every four seconds.)
        self.intertrial_interval_in_process_calls = 20

        # As an example, write the decision times (= time of the EEG sample based on which the trial was decided to be performed)
        # into a file. Other information about the analysis can be written into a file, as well, for later analysis.
        self.file = open("decision_times.txt", "a")

        # Initialize a multiprocessing pool with one process. The pool is used so that EEG/EMG sample stream processing can be
        # continued while a long-standing computation is performed in the background.
        self.pool = multiprocessing.Pool(processes=1)

        # Model training is performed only occasionally; the interval is defined as calls to process method.
        self.model_training_interval = 10

        # A variable to store the ongoing training process. This is used to check if the training process is finished.
        self.training_process = None

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
        events = [
            {
                'type': Event.PREPULSE.value,
                'time': 2.0,
            },
            {
                'type': Event.POSTPULSE.value,
                'time': 3.0,
            },
        ]

        return {
            'processing_interval_in_samples': 1000,
            'process_on_trigger': True,
            'sample_window': [-5, 0],
            'events': events,
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
               A NumPy array of shape (num_of_eeg_channels, num_of_samples), where num_of_eeg_channels is the number of EEG
               channels and num_of_samples is the number of samples in the buffer defined by self.sample_window.

          - emg_buffer:
               A NumPy array of shape (num_of_emg_channels, num_of_samples), where num_of_emg_channels is the number of EMG
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

        print("Processing EEG/EMG samples at time {:.4f}".format(current_time))

        # Increment the buffer count for each received buffer.
        self.buffer_count += 1

        if is_event:
            print("Event of type {} received at time {:.4f}".format(event_type, current_time))

        # As an example, print the value of the first EEG channel at the current sample index into the decider log. Note that
        # using 'print' function here would congest the log with too many messages. Instead, use 'print_throttle' to print
        # the message once every second.
        #
        # Beware of using 'print_throttle' in several places in the code, as the throttling interval is shared among all calls.
        print_throttle("Printed once every second: current EEG sample is {:.4f}".format(eeg_buffer[0, current_sample_index]))

        # Silently return without performing a trial if the pipeline is not ready for a trial. Usually it is good to be verbose
        # and log the reason for not performing a trial, but not being ready for a trial is a common situation that does not
        # require logging.
        if not ready_for_trial:
            return

        # Similarly, do not perform a trial if there are invalid samples in the current buffer. Assuming that the stimulation
        # decision is based on analysis of the whole buffer, it is better to conservatively skip the trial if there are any
        # invalid samples.
        if not np.all(valid_samples):
            return

        # Start training a model every 10 'process' method calls, as defined by self.model_training_interval.
        #
        # See Python's multiprocessing documentation for more information on how to use the multiprocessing module:
        # https://docs.python.org/3/library/multiprocessing.html
        #
        if self.buffer_count % self.model_training_interval == 0:
            print("Starting to train a model at time {:.4f}".format(current_time))
            self.training_process = self.pool.apply_async(train_model, (
                eeg_buffer,
            ))

        # If the training is finished, get the model parameters and execution time.
        if self.training_process is not None:
            try:
                model_parameters, training_time = self.training_process.get(timeout=0)

                print("Finished training a model at time {:.4f}".format(current_time))
                self.training_process = None

                # Print the latest model parameters and execution time.
                print("Model parameters: {}, Training time (s): {:.4f}".format(
                    model_parameters,
                    training_time,
                    current_time))

            except multiprocessing.TimeoutError as e:
                pass

        # Otherwise, perform trial once every four seconds, as defined by self.intertrial_interval_in_process_calls.
        if self.buffer_count % self.intertrial_interval_in_process_calls != 0:
            return

        # Write the decision time into a file.
        self.file.write("{:.4f}\n".format(current_time))

        # Flush the file after each write so that the data is written to disk immediately.
        self.file.flush()

        # Cycle among the three target types.
        targets = self.targets[self.target_type]
        self.target_type = (self.target_type + 1) % len(self.targets)

        # Compute the pulse time based on the current time and the minimum delay before the pulse.
        start_time = current_time + MINIMUM_DELAY_BEFORE_PULSE

        # If performing the paired-pulse target, use a delay of 100 ms between the pulses.
        if len(targets) == 2:
            pulse_times = [start_time, start_time + 0.1]
        else:
            pulse_times = [start_time]

        # Some additional logging for debugging purposes.
        print("Decided at time {:.4f} to perform trial at time {:.4f} with {} target(s)".format(
            current_time,
            start_time,
            len(targets)))

        # A trial consists of the targets, the pulse times, and the information for trigger outputs of the mTMS device.
        trial = {
            'targets': targets,
            'pulse_times': pulse_times,
            'triggers': TRIGGERS,
        }

        return {
            'trial': trial,

            # In addition to performing a trial using the mTMS device, the decider can create timed triggers
            # using LabJack T4. This is useful for triggering commercial TMS devices or other
            # devices that require a trigger signal, and when the mTMS device is not available.
            'timed_trigger': current_time + 0.1,
        }


def train_model(X):
    start = time.time()

    # Train the model here. For now, just sleep for a while.
    time.sleep(6.0)

    training_time = time.time() - start

    # Return a dummy result; the first element are the model parameters, and the second element is
    # the model training time in seconds.
    return np.array([1.0, 2.0, 3.0]), training_time
