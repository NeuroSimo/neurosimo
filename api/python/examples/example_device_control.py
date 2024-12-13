from MTMSApi import MTMSApi

from event_interfaces.msg import (
    ExecutionCondition,
    WaveformPhase,
    WaveformPiece,
    Waveform
)
from mep_interfaces.msg import (
    MepConfiguration,
    PreactivationCheck
)
from eeg_interfaces.msg import TimeWindow
from targeting_interfaces.msg import (
    TargetingAlgorithm,
    ElectricTarget
)

api = MTMSApi()

## Single events

# Charge channel 0 to 20 V.

target_voltage = 20

# Note that the TMS channel indexing starts from 0.
channel = 1
execution_condition = ExecutionCondition.IMMEDIATE

api.send_charge(
    channel=channel,
    target_voltage=target_voltage,
    execution_condition=execution_condition,
)
api.wait_for_completion()

# Execute pulse on channel 0, using the default waveform.
channel = 0

waveform = api.get_default_waveform(channel)
reverse_polarity = False

execution_condition = ExecutionCondition.IMMEDIATE

api.send_pulse(
    channel=channel,
    waveform=waveform,
    execution_condition=execution_condition,
    reverse_polarity=reverse_polarity,
)
api.wait_for_completion()

# Discharge channel 0 completely.
target_voltage = 0

channel = 0
execution_condition = ExecutionCondition.IMMEDIATE

api.send_discharge(
    channel=channel,
    target_voltage=target_voltage,
    execution_condition=execution_condition,
)
api.wait_for_completion()

# Send trigger out on port 1.
port = 1
duration_us = 1000

execution_condition = ExecutionCondition.IMMEDIATE

api.send_trigger_out(
    port=port,
    duration_us=duration_us,
    execution_condition=execution_condition,
)
api.wait_for_completion()

# Send pulse on channel 0 and a simultaneous trigger out on port 1.
waveform = api.get_default_waveform(channel)
reverse_polarity = False

channel = 0
execution_condition = ExecutionCondition.TIMED
time = api.get_time() + 1.0

api.allow_stimulation(True)

api.send_pulse(
    channel=channel,
    waveform=waveform,
    reverse_polarity=reverse_polarity,
    execution_condition=execution_condition,
    time=time,
)
# Do not wait for completion here, as we want to execute the trigger out simultaneously with the pulse.

port = 1
duration_us = 1000

# Use the same time and execution condition as for the pulse.
api.send_trigger_out(
    port=port,
    duration_us=duration_us,
    execution_condition=execution_condition,
    time=time,
)

# Once both pulse and trigger out are sent, wait for the completion of both.
api.wait_for_completion()

## Send pulse on channel 1 and analyze MEP.

# Use default waveform for the pulse.
waveform = api.get_default_waveform(channel)
reverse_polarity = False

# Generate a timed pulse.
channel = 1
execution_condition = ExecutionCondition.TIMED
time = api.get_time() + 3.0

api.send_pulse(
    channel=channel,
    waveform=waveform,
    execution_condition=execution_condition,
    time=time,
    reverse_polarity=reverse_polarity,
)
# Do not wait for completion here, as we want to execute the pulse simultaneously with the MEP analysis.

# Analyze MEP on EMG channel 1, coinciding with the pulse.
mep_configuration = MepConfiguration(
    emg_channel=1,

    time_window=TimeWindow(
        start=0.020,  # in ms, after the stimulation pulse
        end=0.040,  # in ms
    ),
    preactivation_check=PreactivationCheck(
        enabled=True,
        time_window=TimeWindow(
            start=-0.040,  # in ms, minus sign indicates that the window starts before the stimulation pulse
            end=-0.020,
        ),
        voltage_range_limit=70.0,  # Maximum allowed voltage range inside the time window, in uV.
    ),
)

mep, errors = api.analyze_mep(
    time=time,
    mep_configuration=mep_configuration,
)

amplitude = mep.amplitude
latency = mep.latency


## Targeting

# Define the target.

# Available algorithms:
#   TargetingAlgorithm.LEAST_SQUARES
#   TargetingAlgorithm.GENETIC

algorithm = TargetingAlgorithm(
    value=TargetingAlgorithm.GENETIC
)
displacement_x = 0  # mm
displacement_y = 0  # mm
rotation_angle = 45  # deg
intensity = 10  # V/m

target = ElectricTarget(
    displacement_x=displacement_x,
    displacement_y=displacement_y,
    rotation_angle=rotation_angle,
    intensity=intensity,
    algorithm=algorithm,
)

target_voltages, reverse_polarities = api.get_target_voltages(target)

# Charge all channels to target voltages.
api.send_immediate_charge_or_discharge_to_all_channels(
    target_voltages=target_voltages,
)
api.wait_for_completion()

# Send default pulse to all channels.
api.send_immediate_default_pulse_to_all_channels(
    reverse_polarities=reverse_polarities,
)
api.wait_for_completion()


## Paired pulse targeting

algorithm = TargetingAlgorithm(
    value=TargetingAlgorithm.GENETIC
)

# Define the targets.
first_target = ElectricTarget(
    displacement_x=0,  # mm
    displacement_y=0,  # mm
    rotation_angle=45,  # deg
    intensity=10,  # V/m
    algorithm=algorithm,
)

second_target = ElectricTarget(
    displacement_x=5,  # mm
    displacement_y=5,  # mm
    rotation_angle=90,  # deg
    intensity=5,  # V/m
    algorithm=algorithm,
)

targets = [first_target, second_target]

initial_voltages, approximated_waveforms = api.get_multipulse_waveforms(targets)

# Charge all channels to initial voltages.
api.send_immediate_charge_or_discharge_to_all_channels(
    target_voltages=initial_voltages,
)
api.wait_for_completion()

# Set the times (in seconds).
time = api.get_time() + 1.0
time_between_pulses = 0.003

# Send the first pulse.
api.send_timed_pulse_to_all_channels(
    waveforms=approximated_waveforms[0],
    time=time,
)

# Send the second pulse.
api.send_timed_pulse_to_all_channels(
    waveforms=approximated_waveforms[1],
    time=time + time_between_pulses,
)

# Wait for completion of both pulses.
api.wait_for_completion()


## Targeting-related utilities

# Get maximum intensity
algorithm = TargetingAlgorithm(
    value=TargetingAlgorithm.GENETIC
)
displacement_x = 0  # mm
displacement_y = 0  # mm
rotation_angle = 45  # deg

maximum_intensity = api.get_maximum_intensity(
    displacement_x=displacement_x,
    displacement_y=displacement_y,
    rotation_angle=rotation_angle,
    algorithm=algorithm,
)


## Targeting combined with MEP analysis

displacement_x = 5  # mm
displacement_y = 5  # mm
rotation_angle = 90  # deg
intensity = 5  # V/m

target_voltages, reverse_polarities = api.get_target_voltages(
    displacement_x=displacement_x,
    displacement_y=displacement_y,
    rotation_angle=rotation_angle,
    intensity=intensity,
    algorithm=TargetingAlgorithm.GENETIC
)

# Charge all channels to target voltages.
api.send_immediate_charge_or_discharge_to_all_channels(
    target_voltages=target_voltages,
)
api.wait_for_completion()

# Send default pulse to all channels.
time = api.get_time() + 3.0

api.send_timed_default_pulse_to_all_channels(
    reverse_polarities=reverse_polarities,
    time=time,
)
# Do not wait for completion here, as we want to execute the pulse simultaneously with the MEP analysis.

# Analyze MEP on EMG channel 1, coinciding with the pulse.
mep_configuration = MepConfiguration(
    emg_channel=0,  # The EMG channel to analyze, indexing starts from 0

    time_window=TimeWindow(
        start=0.020,  # in ms, after the stimulation pulse
        end=0.040,  # in ms
    ),
    preactivation_check=PreactivationCheck(
        enabled=True,
        time_window=TimeWindow(
            start=-0.040,  # in ms, minus sign indicates that the window starts before the stimulation pulse
            end=-0.020,
        ),
        voltage_range_limit=70.0,  # Maximum allowed voltage range inside the time window, in uV.
    ),
)

mep, errors = api.analyze_mep(
    time=time,
    mep_configuration=mep_configuration,
)

amplitude = mep.amplitude
latency = mep.latency

## Define custom waveform

waveform = Waveform(
    pieces=[
        WaveformPiece(
            waveform_phase=WaveformPhase(
                value=WaveformPhase.RISING
            ),
            duration_in_ticks=2400,
        ),
        WaveformPiece(
            waveform_phase=WaveformPhase(
                value=WaveformPhase.HOLD
            ),
            duration_in_ticks=1200,
        ),
        WaveformPiece(
            waveform_phase=WaveformPhase(
                value=WaveformPhase.FALLING
            ),
            duration_in_ticks=1480,
        ),
    ]
)

## Restart session
api.stop_session()
api.start_session()


## Stop device
api.stop_device()
