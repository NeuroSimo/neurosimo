from MTMSApi import MTMSApi

from mep_interfaces.msg import (
    MepConfiguration,
    PreactivationCheck
)
from eeg_interfaces.msg import TimeWindow
from targeting_interfaces.msg import (
    TargetingAlgorithm,
    ElectricTarget
)
from experiment_interfaces.msg import Experiment, ExperimentMetadata, Trial, IntertrialInterval, TrialConfig, TriggerConfig


api = MTMSApi(
    verbose=False,
)

handler = api.get_experiment_handler()


# Define the experiment.
metadata = ExperimentMetadata(
    experiment_name="Example experiment",
    subject_name="Example subject",
)

target = ElectricTarget(
    displacement_x=0,  # mm
    displacement_y=0,  # mm
    rotation_angle=45,  # deg
    intensity=10,  # V/m
    algorithm=TargetingAlgorithm(
        value=TargetingAlgorithm.LEAST_SQUARES,
    ),
)

mep_config = MepConfiguration(
    emg_channel=0,  # EMG channel 0 corresponds to the first EMG channel in the amplifier.
    time_window=TimeWindow(
        start=0.020,
        end=0.040,
    ),
    preactivation_check=PreactivationCheck(
        enabled=True,
        time_window=TimeWindow(
            start=-0.040,
            end=-0.020,
        ),
        voltage_range_limit=70.0,
    ),
)

# Enable both trigger outs, coinciding with the beginning of the trial with zero delay.
triggers = [
    TriggerConfig(
        enabled=True,
        delay=0.0,
    ),
    TriggerConfig(
        enabled=True,
        delay=0.0,
    ),
]

trial_config = TrialConfig(
    voltage_tolerance_proportion_for_precharging=0.1,  # Do not modify
    use_pulse_width_modulation_approximation=True,  # Do not modify
    recharge_after_trial=True,
    dry_run=False,
)

# If analyze MEP is set to True, the mTMS software will automatically analyze the MEPs and write the analysis results
# into projects/[project-directory]/csv.
#
# TODO: Implement a way to change the project directory from the API.
#
# Note that enabling MEP analysis leads to many ways in which a trial can fail, e.g., if EEG is not available
# or the preactivation check fails.
analyze_mep = False

# Define the trials.
single_pulse_trial = Trial(
    targets=[target],
    pulse_times_since_trial_start=[0.0],

    # These are the same for all trials.
    config=trial_config,
    triggers=triggers,
    analyze_mep=analyze_mep,
    mep_config=mep_config,
)

paired_pulse_trial = Trial(
    targets=[target, target],
    pulse_times_since_trial_start=[0.0, 0.1],

    # These are the same for all trials.
    config=trial_config,
    triggers=triggers,
    analyze_mep=analyze_mep,
    mep_config=mep_config,
)

print("Validating single pulse trial...")

is_valid = handler.validate_trial(single_pulse_trial)
if not is_valid:
    print("Single pulse trial is invalid.")
    exit()

print("Validating paired pulse trial...")

is_valid = handler.validate_trial(paired_pulse_trial)
if not is_valid:
    print("Paired pulse trial is invalid.")
    exit()

trials = 3 * [single_pulse_trial] + 3 * [paired_pulse_trial]

# Other experiment settings.
intertrial_interval = IntertrialInterval(
    min=3.5,
    max=4.5,
    tolerance=0.1, # do not modify
)

# Note that the randomization in the mTMS software uses a constant seed for reproducibility;
# alternatively, you can randomize the 'trials' list yourself.
randomize_trials = True

wait_for_pedal_press = False
autopause = False
autopause_interval = 0

# Define the experiment.
experiment = Experiment(
    metadata=metadata,
    trials=trials,
    intertrial_interval=intertrial_interval,
    randomize_trials=randomize_trials,
    wait_for_pedal_press=wait_for_pedal_press,
    autopause=autopause,
    autopause_interval=autopause_interval,
)

api.start_device()
api.start_session()

handler.perform_experiment(experiment)

while not handler.is_done() and not api.is_interrupted():
    # XXX: Note that this is pretrial feedback, not posttrial feedback. They should be separated in the future.
    feedback = handler.wait_for_feedback()

    # Feedback is None only when the experiment has been aborted. Check that case separately.
    #
    # TODO: Implement feedback in the mTMS software even when the experiment is aborted.
    if feedback is None:
        continue

    handler.print_feedback()

    attempt_number = feedback.attempt_number
    trial_number = feedback.trial_number

    if attempt_number == 3:
        print("Too many failed attempts, pausing experiment after the next attempt.")

        handler.pause_experiment()

        inp = input("Press Enter to resume the experiment.")
        handler.resume_experiment()

    if attempt_number == 4:
        print("Too many failed attempts, aborting experiment after the next attempt.")
        handler.cancel_experiment()

if api.is_interrupted():
    print("Experiment was interrupted, aborting experiment.")
    handler.cancel_experiment()

# 'results' is a list containing TrialResults objects, each including the actual start time of each trial
# and the results of the MEP analysis if it was enabled. The MEP analysis results also include the raw EMG buffer
# for custom analysis. See TrialResult.msg and Mep.msg for more information.
results = handler.get_trial_results()

api.stop_session()
