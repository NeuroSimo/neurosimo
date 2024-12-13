import math
import time

from event_interfaces.msg import (
    ChargeFeedback,
    ChargeError,
    DischargeFeedback,
    DischargeError,
    PulseError,
    PulseFeedback,
)
from mtms_device_interfaces.msg import ChannelError


class Channel:
    """mTMS simulator coil channel.

    Responsible for simulating the voltage charge and discharge and pulses.
    """

    CLOCK_FREQUENCY_HZ = 4e7

    def __init__(
        self,
        charge_rate: float,
        capacitance: float,
        time_constant: float,
        pulse_voltage_drop_proportion: float,
        max_voltage: int,
        logger,
    ):
        """
        Initialising of the Channel with channel specific properties.

        Args:
            charge_rate: voltage charging rate scaled by capacitance in watts.
            capacitance: capacitance of the coil in farads.
            time_constant: time constant for discharge rate in seconds, tau=RC.
            pulse_voltage_drop_proportion: the proportion which the voltage drops per pulse.
            max_voltage: the maximum voltage of this channel.
            logger: logger to log the events.
        """
        self.logger = logger

        self.charge_rate = charge_rate
        """Charge rate of the voltage in watts."""

        self.capacitance = capacitance
        """Coil capacitance in farads."""

        self.time_constant = time_constant
        """Constant related to voltage discharge in seconds."""

        self.pulse_voltage_drop_proportion = pulse_voltage_drop_proportion
        """Voltage drop proportion when pulse is given."""

        self.max_voltage = max_voltage
        """Maximum voltage of the coil."""

        # Internal state

        self.current_voltage = 0
        """Current voltage in volts."""

        self.temperature = 24
        """Current temperature in C."""

        self.pulse_count = 0
        """Total pulse count of the coil"""

        self.errors = ChannelError()
        """Channel error states."""

        self.is_charging = False
        """True when charging."""

        self.is_discharging = False
        """True when discharging."""

        self.is_pulse_in_progress = False
        """True when giving a pulse."""

    def charge(self, target_voltage: int, event_id: int) -> ChargeFeedback:
        """
        Charge the channel to desired voltage.

        New voltage is not set immediately, and rather models the behaviour of a
        capacitor to change the voltage. Charging is done with constant power and the
        required energy to charge to certain voltage increases quadratically.

        Args:
            target_voltage: the new voltage to charge to in volts.
            event_id: The event id from the event info.

        Returns:
            When charging is ready return ChargeFeedback with event id and errors.
        """

        # Calculate wait time
        charge_rate_constant = self.capacitance / (2 * self.charge_rate)
        t = charge_rate_constant * (target_voltage**2 - self.current_voltage**2)

        self.is_charging = True
        time.sleep(t)
        self.is_charging = False

        # Set new voltage
        self.current_voltage = target_voltage

        return ChargeFeedback(id=event_id)

    def discharge(
        self, target_voltage: int, event_id: int
    ) -> DischargeFeedback:
        """
        Discharges the channel to desired voltage.

        New voltage is not set immediately, and rather models the behaviour of a
        capacitor to change the voltage. Discharge follows exponential decay.

        Args:
            target_voltage: The new voltage to discharge to in volts.
            event_id: The event id from the event info.

        Returns:
            When discharging is ready return DischargeFeedback with event id and errors.
        """

        # To prevent division by zero if discharging the coil back to 0
        target_voltage = max(target_voltage, 3)

        self.logger.info(f"Discharging from {self.current_voltage} to {target_voltage}")

        # Calculate wait time
        t = self.time_constant * math.log(self.current_voltage / target_voltage)

        self.is_discharging = True

        # Note: Due to the system state messages being sent so infrequently, the
        #   discharging sometimes happens from a lower voltage to a higher voltage
        #   (because the event was sent before the voltage was updated). This is a
        #   fix to prevent the 'sleep' call from crashing in that case.
        if t > 0:
            time.sleep(t)

        self.is_discharging = False

        # Set new voltage
        self.current_voltage = target_voltage

        return DischargeFeedback(id=event_id)

    def pulse(self, event_id: int, duration_ticks: int) -> PulseFeedback:
        """
        Simulates the behaviour of giving a pulse.

        Simulates the giving of a pulse by dropping the channel voltage by the amount of
        self.pulse_drop_proportion.

        Args:
            event_id: id of the event.
            duration_ticks: pulse duration in ticks.
        Returns:
            When pulse given return PulseFeedback with event id and errors.
        """
        self.pulse_count += 1
        duration = duration_ticks / Channel.CLOCK_FREQUENCY_HZ

        self.is_pulse_in_progress = True
        time.sleep(duration)
        self.is_pulse_in_progress = False

        after_pulse_voltage_proportion = 1 - self.pulse_voltage_drop_proportion
        self.current_voltage = after_pulse_voltage_proportion * self.current_voltage

        return PulseFeedback(id=event_id)
