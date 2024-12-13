import time

from MTMSApi import MTMSApi

from event_interfaces.msg import (
    ExecutionCondition,
)
from targeting_interfaces.msg import (
    TargetingAlgorithm,
    ElectricTarget
)


api = MTMSApi()

api.start_device()

api.stop_session()
api.start_session()


print("")

# Prompt to ask if the user wants to use electrical targeting.
use_electrical_targeting = input('Use electrical targeting? (Y/n): ')
use_electrical_targeting = use_electrical_targeting.lower() != 'n'

print("")

if use_electrical_targeting:
    # Electrical targeting

    # Prompt the user for x- and y-displacement, rotation angle, and intensity.
    displacement_x = int(input('X-displacement (mm): '))
    displacement_y = int(input('Y-displacement (mm): '))
    rotation_angle = int(input('Rotation angle (degrees): '))
    intensity = int(input('Intensity (V/m): '))

    algorithm_str = input('Algorithm, [g]enetic or [l]east squares: ')

    value = TargetingAlgorithm.GENETIC if algorithm_str.lower() == 'g' else TargetingAlgorithm.LEAST_SQUARES
    algorithm = TargetingAlgorithm(value=value)

    target = ElectricTarget(
        displacement_x=displacement_x,
        displacement_y=displacement_y,
        rotation_angle=rotation_angle,
        intensity=intensity,
        algorithm=algorithm,
    )
    target_voltages, reverse_polarities = api.get_target_voltages(target)
else:
    # Manual voltage setting

    target_voltages = 5 * [None]
    for channel in range(5):
        target_voltages[channel] = int(input('Target voltage for channel {} (-1500 to 1500; negative for reverse polarity): '.format(channel + 1)))

    reverse_polarities = [voltage < 0 for voltage in target_voltages]
    target_voltages = [abs(voltage) for voltage in target_voltages]

# Print target voltages
print("")
print("Target voltages:")
for channel, voltage in enumerate(target_voltages):
    print("  Channel {}: {} V".format(channel + 1, round(voltage)))

print("")

try:
    while True:
        for n in range(2):
            if use_electrical_targeting:
                print("Targeting at ({}, {}, {}) with intensity {} V/m, using {} algorithm".format(
                    displacement_x,
                    displacement_y,
                    rotation_angle,
                    intensity,
                    'genetic' if algorithm.value == TargetingAlgorithm.GENETIC else 'least squares'
                ))

            time_before_charging = time.time()

            execution_condition = ExecutionCondition.IMMEDIATE
            api.send_immediate_charge_or_discharge_to_all_channels(
                target_voltages=target_voltages,
            )

            time_after_charging = time.time()

            # Send trigger out on port 1.
            port = 1
            duration_us = 100

            api.send_trigger_out(
                port=port,
                duration_us=duration_us,
                execution_condition=ExecutionCondition.WAIT_FOR_TRIGGER,
            )

            # Send pulse on the channel, using the default waveform.
            ids = api.send_default_pulse_to_all_channels(
                reverse_polarities=reverse_polarities,
                execution_condition=ExecutionCondition.WAIT_FOR_TRIGGER,
            )
            time_after_preparing_pulse = time.time()

            print("")
            print("Charging time (s): {:.3f}".format(time_after_charging - time_before_charging))
            print("Total time (s) for charging and preparing pulse: {:.3f}".format(time_after_preparing_pulse - time_before_charging))
            print("")

            api.wait_for_completion()

except KeyboardInterrupt:
    api.stop_session()
