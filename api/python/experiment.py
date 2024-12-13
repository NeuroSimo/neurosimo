import os
import time
from datetime import datetime

import numpy as np
from pytimedinput import timedKey

from event_interfaces.msg import ExecutionCondition, PulseError
from targeting_interfaces.msg import TargetingAlgorithm


class Color:
   PURPLE = '\033[95m'
   CYAN = '\033[96m'
   DARKCYAN = '\033[36m'
   BLUE = '\033[94m'
   GREEN = '\033[92m'
   YELLOW = '\033[93m'
   RED = '\033[91m'
   BOLD = '\033[1m'
   UNDERLINE = '\033[4m'
   END = '\033[0m'


class Experiment:
    def __init__(self, experiment_name, test_experiment, output_directory, api):
        self.experiment_name = experiment_name
        self.test_experiment = test_experiment
        self.output_directory = output_directory
        self.api = api

        os.makedirs(output_directory, exist_ok=True)

        self.experiment_start_time = datetime.now()

        self.output_filename = "{}_{}.csv".format(
            self.experiment_start_time.strftime('%Y-%m-%d_%H-%M-%S'),
            self.experiment_name,
        )
        self.output_path = os.path.join(self.output_directory, self.output_filename)

        # Enable line buffering by using 'buffering' argument.
        self.output_file = open(self.output_path, "w", buffering=1)

        header = "{};{};{};{};{};{};{};{};{};{};{};{}\n".format(
            "Trial index",
            "Description",
            "Time",
            "Time (pause adjusted)",
            "x (mm)",
            "y (mm)",
            "Angle (deg)",
            "Intensity (V/m)",
            "Pulse success",
            "MEP success",
            "MEP amplitude (uV)",
            "MEP latency (s)",
        )
        self.output_file.write(header)

        self.trials = []
        self.total_duration_of_pauses = 0.0

        np.random.seed(1)

    def configure_mep_analysis(self, enabled, mep_configuration):
        self.analyze_mep_enabled = enabled
        self.mep_configuration = mep_configuration

    def add_trials(self, actions, condition):
        self.trials += [
            {
                'time': None,
                'condition': condition,
                'actions': action,
            } for action in actions
        ]
        self.num_of_trials = len(self.trials)

    def permute_trials(self):
        self.trials = np.random.permutation(self.trials)

    def add_trial_times(self, isi_low, isi_high, first_trial_time=5.0):
        time = first_trial_time
        for i in range(self.num_of_trials):
            self.trials[i]['time'] = time
            time += np.random.uniform(isi_low, isi_high)

    def execute_pulse(self, condition, time, params):
        x = int(params['x'])
        y = int(params['y'])
        angle = int(params['angle'])
        intensity = int(params['intensity'])

        # Cap intensity at 5 V/m when testing the experiment.
        if self.test_experiment is True:
            print("Testing the experiment: Capping intensity to 5 V/m")
            print("")

            intensity = min(intensity, 5)

        voltages, reverse_polarities = self.api.get_target_voltages(
            displacement_x=x,
            displacement_y=y,
            rotation_angle=angle,
            intensity=intensity,
            algorithm=TargetingAlgorithm.LEAST_SQUARES,
        )

        self.api.send_immediate_charge_or_discharge_to_all_channels(
            target_voltages=voltages,
            wait_for_completion=False,
        )

        ids = self.api.send_timed_default_pulse_to_all_channels(
            reverse_polarities=reverse_polarities,
            time=time,
            wait_for_completion=False,
        )

        print("")
        print("Executing pulse at ({}, {}, {}) with intensity {} V/m at time {:.4f} s.".format(x, y, angle, intensity, time))

        return ids

    def execute_trigger(self, condition, time, params):
        port = params['port']
        delta_time = params['delta_time']

        time_adjusted = time + delta_time

        self.api.send_trigger_out(
            port=port,
            execution_condition=ExecutionCondition.TIMED,
            time=time_adjusted,
            wait_for_completion=False,
        )

        print("")
        print("Executing trigger on port {} at time {:.4f} s.".format(port, time_adjusted))

    # Validation

    def validate_pulse(self, params, verbose):
        x = int(params['x'])
        y = int(params['y'])
        angle = int(params['angle'])
        intensity = int(params['intensity'])

        if verbose:
            print("Validating pulse at ({}, {}, {}) with intensity {} V/m.".format(x, y, angle, intensity))

        maximum_intensity = self.api.get_maximum_intensity(
            displacement_x=x,
            displacement_y=y,
            rotation_angle=angle,
            algorithm=TargetingAlgorithm.LEAST_SQUARES,
        )

        if intensity > maximum_intensity:
            if verbose:
                print("The intensity ({} V/m) of the trial is higher than the maximum intensity ({} V/m). Skipping trial.".format(
                    intensity,
                    maximum_intensity,
                ))

            return False

        return True

    def validate_trial(self, trial, verbose=True):
        actions = trial['actions']
        for action in actions:
            action_type = action['type']
            params = action['params']

            if action_type == 'pulse':
                pulse_ok = self.validate_pulse(params, verbose=verbose)
                if not pulse_ok:
                    return False

        return True

    def check_valid_trials(self):
        num_of_all_trials = len(self.trials)
        num_of_valid_trials = 0

        for trial in self.trials:
            trial_ok = self.validate_trial(trial, verbose=False)
            if trial_ok:
                num_of_valid_trials += 1

        print("{}/{} of the trials are valid.".format(num_of_valid_trials, num_of_all_trials))

    # Performing trial

    def perform_trial(self, trial):
        condition = trial['condition']
        time = trial['time']
        actions = trial['actions']

        time_pause_adjusted = time + self.total_duration_of_pauses

        pulse_ids = []
        for action in actions:
            action_type = action['type']
            params = action['params']

            if action_type == 'pulse':
                pulse_ids = self.execute_pulse(condition, time_pause_adjusted, params)

            elif action_type == 'trigger':
                self.execute_trigger(condition, time_pause_adjusted, params)

        if self.analyze_mep_enabled:
            mep_measurement_success, amplitude, latency = self.analyze_mep(time_pause_adjusted)

        self.api.wait_until(time + 0.1)

        # Determine if the pulse was executed successfully.
        pulse_feedbacks = [self.api.get_event_feedback(id) for id in pulse_ids]
        pulses_ok = [feedback.value == PulseError.NO_ERROR for feedback in pulse_feedbacks]
        pulse_success = all(pulses_ok)

        # Determine if MEP was successful.
        #
        # Note: For MEP to be successful, both the pulse and the measurement of the MEP must be successful.
        #
        mep_success = pulse_success and mep_measurement_success

        trial['mep'] = {
            'amplitude': amplitude,
            'latency': latency,
            'success': mep_success,
        }
        trial['pulse_success'] = pulse_success

        print("")
        if mep_success:
            print("Successfully analyzed MEP with amplitude {:.1f} (\u03BCV) and latency {:.1f} (ms).".format(amplitude, 1000 * latency))
        elif mep_measurement_success:
            print("{}MEP measurement was successful but pulse failed.{}".format(Color.RED, Color.END))
        elif pulse_success:
            print("{}Pulse was successful but MEP analysis failed.{}".format(Color.RED, Color.END))
        else:
            print("{}Pulse and MEP measurement both failed.{}".format(Color.RED, Color.END))
        print("")

    def analyze_mep(self, time):
        emg_channel = self.mep_configuration.emg_channel

        print("")
        print("Analyzing MEP on EMG channel {} at time {:.4f} s.".format(emg_channel, time))

        mep, errors = self.api.analyze_mep(
            time=time,
            mep_configuration=self.mep_configuration,
        )

        amplitude = mep.amplitude
        latency = mep.latency

        success = True

        if errors.mep_error.value != 0:
            print("WARNING: MEP error occurred.")
            success = False

        if errors.gather_mep_error.value != 0:
            print("WARNING: Gather MEP error occurred.")
            success = False

        if errors.gather_preactivation_error.value != 0:
            print("WARNING: Gather preactivation error occurred.")
            success = False

        return success, amplitude, latency

    def write_trial(self, i, trial):
        condition = trial['condition']
        time = trial['time']

        time_pause_adjusted = time + self.total_duration_of_pauses

        x = self.get_param(trial, 'x')
        y = self.get_param(trial, 'y')
        angle = self.get_param(trial, 'angle')
        intensity = self.get_param(trial, 'intensity')

        pulse_success = trial['pulse_success']

        mep_success = self.get_mep_attribute(trial, 'success')
        mep_amplitude = self.get_mep_attribute(trial, 'amplitude')
        mep_latency = self.get_mep_attribute(trial, 'latency')

        s = "{};{};{:.3f};{:.3f};{};{};{};{};{};{};{:.1f};{:.4f}\n".format(
            i,
            condition,
            time,
            time_pause_adjusted,
            x,
            y,
            angle,
            intensity,
            "true" if pulse_success else "false",
            "true" if mep_success else "false",
            mep_amplitude if mep_success else 0.0,
            mep_latency if mep_success else 0.0,
        )
        self.output_file.write(s)

    def yes_or_no(self, text="Continue?"):
        ans = None
        while ans not in ['y', 'Y', 'n', 'N']:
            ans = input("{} (Y/N) ".format(text))
        return ans

    def pause(self):
        start = time.time()
        ans = self.yes_or_no()
        end = time.time()

        self.total_duration_of_pauses += end - start

        if ans in ['y', 'Y']:
            return False

        return True

    def perform(self):
        # Start the device if not started.
        self.api.start_device()

        # Restart session.
        self.api.stop_session()
        self.api.start_session()

        # Do not allow stimulation when testing the experiment.
        if self.test_experiment:
            self.api.allow_stimulation(True)

        try:
            # Cap number of trials to perform to 10 when testing the experiment.
            if self.test_experiment:
                self.num_of_trials = 10
                print("Testing the experiment: Capping # of trials to 10")
                print("")

            i = 0
            while i < self.num_of_trials:
                trial = self.trials[i]
                _, timed_out = timedKey("Press any key to pause before the trial ", allowCharacters="", timeout=0.5)

                if not timed_out:
                    stop_experiment = self.pause()
                    if stop_experiment:
                        break

                print("")
                print("")
                print("{}{}Trial {} / {}{}".format(Color.BOLD, Color.UNDERLINE, i + 1, self.num_of_trials, Color.END))
                print("")

                trial_ok = self.validate_trial(trial)
                if not trial_ok:
                    i += 1
                    continue

                self.perform_trial(trial)
                self.write_trial(i, trial)

                mep_success = self.get_mep_attribute(trial, 'success')
                if mep_success:
                    i += 1
                    print("Trial finished.")
                else:
                    print("Redoing trial...")

                    # HACK: Okay, this is quite hacky, but let's settle for it for now.
                    #   A proper way to implement redoing trials would be to not plan
                    #   the trial times so carefully in advance, but instead adapt on the fly.
                    if i > 0:
                        self.total_duration_of_pauses += self.trials[i]['time'] - self.trials[i-1]['time']
                    else:
                        self.total_duration_of_pauses += self.trials[i]['time']
        finally:
            self.api.stop_session()
            self.output_file.close()

        print("")
        print("Experiment finished.")
        print("")

    def get_valid_trials(self):
        valid_trial_indices = ['mep' in trial and trial['mep']['amplitude'] is not None and trial['pulse_success'] is True for trial in self.trials]
        valid_trials = self.trials[valid_trial_indices]

        return valid_trials

    def get_mep_attribute(self, trial, attribute):
        return trial['mep'][attribute] if 'mep' in trial else None

    def get_param(self, trial, param):
        if param in trial['actions'][0]['params']:
            return trial['actions'][0]['params'][param]
        else:
            return np.nan
