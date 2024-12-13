"""
This Python module provides an API for controlling a multi-channel transcranial magnetic stimulation (mTMS) device.
It includes the functionality to start and stop the device, send pulses, charges, and discharges, and to perform
various analyses of the EEG/EMG data.

It uses the Robot Operating System (ROS2) to interact with the device.
"""
import copy
import signal
import time

import rclpy

from system_interfaces.msg import SessionState
from mtms_device_interfaces.msg import DeviceState
from event_interfaces.msg import ExecutionCondition, WaveformsForCoilSet

from MTMSApiNode import MTMSApiNode

class MTMSApi:
    """
    An API for controlling a multi-channel transcranial magnetic stimulation (mTMS) device.
    """
    # TIME_EPSILON is used to implement events that are to be executed immediately but
    # wanting to synchronize them: to do that, get current time, add TIME_EPSILON to it,
    # and execute all the events at that time.
    #
    # Consequently, TIME_EPSILON must be large enough to allow time to send the events to
    # the mTMS device, but not too large so that the events are not executed 'immediately'.
    # Settle for 0.15 s (150 ms) for now, but change if needed.
    #
    TIME_EPSILON = 0.15

    def __init__(self, channel_count=5, verbose=True):
        """
        Initializes the MTMSApi instance, creating a new MTMSApiNode.

        Parameters
        ----------
        channel_count : int, optional
            The number of channels in the device. Default is 5.

            TODO: This should come from .env instead of the API user setting it.
        """
        rclpy.init(args=None)

        self.node = MTMSApiNode(
            channel_count=channel_count,
            verbose=verbose,
        )
        self.interrupted = False

        self.channel_count = channel_count
        self.verbose = verbose

        self.latest_event_id = 0
        self.incomplete_events = []

        print("Configuration:")
        print("")
        print("  Channel count: {}".format(self.channel_count))
        print("  Verbose: {}".format(self.verbose))
        print("")

        signal.signal(signal.SIGINT, self.handle_sigint)

    def handle_sigint(self, signum, frame):
        self.interrupted = True
        raise KeyboardInterrupt

    def is_interrupted(self):
        return self.interrupted

    # General

    def _next_event_id(self):
        """
        Increment the event id and return the new id.

        Does not require any parameters, and does not return any value.
        """
        self.latest_event_id += 1
        self.incomplete_events.append(self.latest_event_id)

        return self.latest_event_id

    # Start and stop

    def start_device(self):
        """
        Start the mTMS device, waiting until the device reports its state as operational or start-up fails.

        Does not require any parameters, and does not return any value.
        """
        self.node.start_device()

        # If start-up fails, device state will end up as 'Not operational'. Hence, check both conditions.
        while self.get_device_state() not in [DeviceState.OPERATIONAL, DeviceState.NOT_OPERATIONAL]:
            if self.verbose:
                self.node.print_state()

    def stop_device(self):
        """
        Stop the mTMS device, waiting until the device reports its state as not operational.

        Does not require any parameters, and does not return any value.
        """
        self.node.stop_device()
        while self.get_device_state() != DeviceState.NOT_OPERATIONAL:
            # For safety reasons, be very explicit when stopping session and/or device, which
            # lowers the capacitor voltages down to zero; we want the operator to know exactly
            # when it is finished - hence do not allow interrupting waiting for the device to
            # be stopped using SIGINT.
            try:
                if self.verbose:
                    self.node.print_state()
            except KeyboardInterrupt:
                pass

    def start_session(self):
        """
        Start an session, waiting until the session state is reported as started.

        Does not require any parameters, and does not return any value.
        """
        self.node.start_session()
        while self.get_session_state() != SessionState.STARTED:
            if self.verbose:
                self.node.print_state()

    def stop_session(self):
        """
        Stop the current session, waiting until the session state is reported as stopped.

        Does not require any parameters, and does not return any value.
        """
        self.node.stop_session()
        while self.get_session_state() != SessionState.STOPPED:
            # For safety reasons, be very explicit when stopping session and/or device, which
            # lowers the capacitor voltages down to zero; we want the operator to know exactly
            # when it is finished - hence do not allow interrupting waiting for the session to
            # be stopped using SIGINT.
            try:
                if self.verbose:
                    self.node.print_state()
            except KeyboardInterrupt:
                pass

    # Functions for waiting

    def wait_for_completion(self, timeout=None):
        """
        Wait until the completion of all remaining events.

        Parameters
        ----------
        timeout : float, optional
            The maximum time to wait for the completion of events. If None, wait indefinitely.
        """
        start_time = self.get_time()

        while len(self.incomplete_events) > 0:
            self.node.wait_for_new_state()

            # Remove completed events.
            self.incomplete_events = [id for id in self.incomplete_events if self.get_event_feedback(id) is None]

            # Check if timeout has been reached.
            if timeout is not None and self.get_time() - start_time > timeout:
                print("Timeout reached while waiting for completion of events.")

                # Remove remaining events.
                self.incomplete_events = []

                break

            if self.verbose:
                self.node.print_state()

        # Always print the state after completion.
        self.node.wait_for_new_state()
        self.node.print_state(force=True)

    def wait_until(self, time):
        """
        Wait until the system time is equal to or greater than the specified time.

        Parameters
        ----------
        time : float
            The time to wait until (as seconds).
        """
        while self.get_time() < time:
            if self.verbose:
                self.node.print_state()

    def wait(self, time):
        """
        Wait for a specified duration of time.

        Parameters
        ----------
        time : float
            The duration to wait for (in seconds).
        """
        start_time = self.get_wallclock_time()

        self.node.wait_for_new_state()
        while self.get_wallclock_time() < start_time + time:
            self.node.wait_for_new_state()
            if self.verbose:
                self.node.print_state()

    # Getters

    def get_device_state(self):
        """
        Return the current state of the device.

        Returns
        -------
        int
            The current state of the device. One of the following:

            * DeviceState.NOT_OPERATIONAL : Device is unoperational.
            * DeviceState.STARTUP : Device is starting up.
            * DeviceState.OPERATIONAL : Device is operational.
            * DeviceState.SHUTDOWN : Device is shutting down.
        """
        self.node.wait_for_new_state()
        return self.node.system_state.device_state.value

    def get_session_state(self):
        """
        Return the current state of the session.

        Returns
        -------
        int
            The current state of the session. One of the following:

            * SessionState.STOPPED : Session is stopped.
            * SessionState.STARTING : Session is starting.
            * SessionState.STARTED : Session is started.
            * SessionState.STOPPING : Session is stopping.
        """
        self.node.wait_for_new_state()
        return self.node.session.state.value

    def get_current_voltage(self, channel):
        """
        Return the current capacitor voltage (V) of the given channel.

        Parameters
        ----------
        channel : int
            The channel id.

        Returns
        -------
        float
            The capacitor voltage (V) of the specified channel.
        """
        self.node.wait_for_new_state()
        return self.node.system_state.channel_states[channel].voltage

    def get_current_voltages(self):
        """
        Return the current capacitor voltages (V) of all channels.

        Returns
        -------
        array-like
            The current channel voltages.
        """
        self.node.wait_for_new_state()

        voltages = [self.node.system_state.channel_states[channel].voltage for channel in range(self.channel_count)]
        return voltages

    def get_temperature(self, channel):
        """
        Return the coil temperature of the given channel if a temperature sensor is present,
        otherwise return None.

        Parameters
        ----------
        channel : int
            The channel id.

        Returns
        -------
        float
            The coil temperature of the specified channel.
        """
        self.node.wait_for_new_state()
        return self.node.system_state.channel_states[channel].temperature

    def get_pulse_count(self, channel):
        """
        Return the total number of pulses generated with the coil connected to the specified channel.

        Parameters
        ----------
        channel : int
            The channel id.

        Returns
        -------
        float
            The total number of pulses generated.
        """
        self.node.wait_for_new_state()
        return self.node.system_state.channel_states[channel].pulse_count

    def get_time(self):
        """
        Return the current time from the start of the session.

        Returns
        -------
        float
            The current time as seconds.
        """
        self.node.wait_for_new_state()
        return self.node.session.time

    def get_event_feedback(self, id):
        """
        Return feedback for a specified event id.

        Parameters
        ----------
        id : int
            The ID of the event.

        Returns
        -------
        int
            The event feedback.
        """
        return self.node.get_event_feedback(id)

    def is_session_started(self):
        return self.get_session_state() == SessionState.STARTED

    # Events
    def allow_stimulation(self, allow_stimulation=False):
        """
        Allow or disallow executing stimulation pulses.

        Parameters
        ----------
        allow_stimulation : bool
            Either True or False. If True, allow executing stimulation pulses.
        """
        self.node.allow_stimulation(
            allow_stimulation=allow_stimulation,
        )

    def send_pulse(self, channel, waveform, execution_condition=ExecutionCondition.TIMED, time=None, reverse_polarity=False):
        """
        Send a pulse event to a specified channel.

        Parameters
        ----------
        channel : int
            The target channel. Range: 1-5
        waveform : list of dicts
            A list of dictionaries with keys `mode` and `duration_in_ticks`:

                * `mode` is one of the following:
                    * PulseMode.RISING
                    * PulseMode.HOLD
                    * PulseMode.FALLING
                    * PulseMode.ALTERNATIVE_HOLD
                * `duration_in_ticks`, range: 0-65535

        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float, optional
            The time at which the pulse is executed. Only needs to be given if execution condition is ExecutionCondition.TIMED.
        reverse_polarity : bool, optional
            Whether to reverse the polarity of the waveform. Default is False.

        Returns
        -------
        int
            The ID of the event.

        Notes
        -----
        The event ID is incremented with each pulse sent.
        """
        assert self.is_session_started(), "Session not started."
        assert not (execution_condition == ExecutionCondition.TIMED and time is None), "Execution condition is ExecutionCondition.TIMED but time not given."

        id = self._next_event_id()

        waveform_ = copy.deepcopy(waveform)
        if reverse_polarity:
            waveform_ = self.reverse_polarity(waveform_)

        self.node.send_pulse(
            id=id,
            execution_condition=execution_condition,
            time=time,
            channel=channel,
            waveform=waveform_,
        )
        return id

    def send_charge(self, channel, target_voltage, execution_condition=ExecutionCondition.TIMED, time=None):
        """
        Send a charge to a specified channel.

        Parameters
        ----------
        channel : int
            The channel for charging. Range: 1-5
        target_voltage : float
            The target voltage for charging. Range: 0-1500
        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float, optional
            The time at which charging is executed. Only needs to be given if execution condition is ExecutionCondition.TIMED.

        Returns
        -------
        int
            The ID of the event.

        Notes
        -----
        The event ID is incremented with each charge sent.
        """
        assert self.is_session_started(), "Session not started."
        assert not (execution_condition == ExecutionCondition.TIMED and time is None), "Execution condition is 'timed' but time not provided."
        assert not (execution_condition != ExecutionCondition.TIMED and time is not None), "Execution condition is not 'timed' but time is provided."
        assert 0 <= target_voltage <= 1500, "Target voltage out of range."

        id = self._next_event_id()

        target_voltage = int(target_voltage)
        self.node.send_charge(
            id=id,
            execution_condition=execution_condition,
            time=time,
            channel=channel,
            target_voltage=target_voltage,
        )
        return id

    def send_discharge(self, channel, target_voltage, execution_condition=ExecutionCondition.TIMED, time=None):
        """
        Send a discharge to a specified channel.

        Parameters
        ----------
        channel : int
            The channel for discharging. Range: 0-5
        target_voltage : float
            The target voltage for the discharge.
        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float, optional
            The time at which discharging is executed. Only needs to be given if execution condition is ExecutionCondition.TIMED.

        Returns
        -------
        int
            The ID of the event.

        Notes
        -----
        The event ID is incremented with each discharge sent.
        """
        assert self.is_session_started(), "Session not started."
        assert not (execution_condition == ExecutionCondition.TIMED and time is None), "Execution condition is 'timed' but time not provided."
        assert not (execution_condition != ExecutionCondition.TIMED and time is not None), "Execution condition is not 'timed' but time is provided."

        id = self._next_event_id()

        target_voltage = int(target_voltage)

        # XXX: Discharging to 0-2 V can take a relatively long time; therefore, set the minimum target voltage to 3.
        #   In the long term, come up with a better solution.
        target_voltage = max(3, target_voltage)

        self.node.send_discharge(
            id=id,
            execution_condition=execution_condition,
            time=time,
            channel=channel,
            target_voltage=target_voltage,
        )
        return id

    def send_trigger_out(self, port, duration_us=1000, execution_condition=ExecutionCondition.TIMED, time=None):
        """
        Sends a trigger output to a specified port.

        Parameters
        ----------
        port : int
            The port number to send the trigger output to.
        duration_us : int
            The duration of the trigger in microseconds. Defaults to 1000 (one millisecond).
        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float, optional
            The time at which the trigger is executed. Only needs to be given if execution condition is ExecutionCondition.TIMED.

        Returns
        -------
        int
            The ID of the event.

        Notes
        -----
        The event ID is incremented with each trigger sent.
        """
        assert self.is_session_started(), "Session not started."
        assert not (execution_condition == ExecutionCondition.TIMED and time is None), "Execution condition is 'timed' but time not provided."
        assert not (execution_condition != ExecutionCondition.TIMED and time is not None), "Execution condition is not 'timed' but time is provided."

        id = self._next_event_id()

        self.node.send_trigger_out(
            id=id,
            execution_condition=execution_condition,
            time=time,
            port=port,
            duration_us=duration_us,
        )
        return id

    def request_trigger(self):
        """
        Request a trigger from the mTMS device, executing events that have execution condition set to
        ExecutionCondition.WAIT_FOR_TRIGGER.

        Does not require any parameters, and does not return any value.
        """
        self.node.request_trigger()

    # Testing and debugging (undocumented)

    def start_device_without_waiting(self):
        self.node.start_device()

    def stop_device_without_waiting(self):
        self.node.stop_device()

    def start_session_without_waiting(self):
        self.node.start_session()

    def stop_session_without_waiting(self):
        self.node.stop_session()

    def wait_until_not_operational(self):
        while self.get_device_state() != DeviceState.NOT_OPERATIONAL:
            pass

    def wait_until_operational(self):
        while self.get_device_state() != DeviceState.OPERATIONAL:
            pass

    # Helpers

    def get_default_waveform(self, channel):
        return self.node.get_default_waveform(channel=channel)

    def get_default_waveforms_for_coil_set(self):
        waveforms = [self.get_default_waveform(channel=channel) for channel in range(self.channel_count)]

        return WaveformsForCoilSet(
            waveforms=waveforms,
        )

    def reverse_polarity(self, waveform):
        return self.node.reverse_polarity(waveform=waveform)

    # Targeting

    def get_target_voltages(self, target):
        """
        Return the target voltages (V), given the target ROS message.

        Parameters
        ----------
        target : ElectricTarget
            The target object, containing the displacement, rotation angle, intensity, and targeting algorithm.

        Returns
        -------
        array-like
            Target voltages.
        """
        return self.node.get_target_voltages(target)

    def get_maximum_intensity(self, displacement_x, displacement_y, rotation_angle, algorithm):
        """
        Return the maximum intensity given the displacements and rotation angle.

        Parameters
        ----------
        displacement_x : float
            Displacement in the x direction.
        displacement_y : float
            Displacement in the y direction.
        rotation_angle : float
            Rotation angle in degrees.
        algorithm : int
            One of the following:
                TargetingAlgorithm.LEAST_SQUARES
                TargetingAlgorithm.GENETIC

        Returns
        -------
        float
            The maximum intensity.
        """

        return self.node.get_maximum_intensity(
            displacement_x=displacement_x,
            displacement_y=displacement_y,
            rotation_angle=rotation_angle,
            algorithm=algorithm,
        )

    def get_multipulse_waveforms(self, targets):
        """
        Return the paired pulse waveforms for the first and second channels.

        Parameters
        ----------
        targets : list of ElectricTarget
            A list of ElectricTarget objects, each containing the displacement, rotation angle, intensity, and algorithm.

        Returns
        -------
        initial_voltages : list of integers
            The initial voltages for each coil.
        approximated_waveforms : list of WaveformsForCoilSet
            A WaveformsForCoilSet object for each target, each defining the waveforms for each coil.
        """
        target_waveforms = [self.get_default_waveforms_for_coil_set() for _ in range(len(targets))]

        return self.node.get_multipulse_waveforms(
            targets=targets,
            target_waveforms=target_waveforms,
        )

    # Stimulation

    def is_stimulation_allowed(self):
        """
        Return True if stimulation is allowed, otherwise False.

        Returns
        -------
        bool
            True if stimulation is allowed, otherwise False.
        """
        return self.node.is_stimulation_allowed()

    # Compound events

    def send_immediate_charge_or_discharge_to_all_channels(self, target_voltages):
        """
        Send immediate charge or discharge commands to all channels.

        Parameters
        ----------
        target_voltages : list of floats
            List of target voltages for each channel.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."
        assert len(target_voltages) == self.channel_count, "Target voltage only defined for {} channels, channel count: {}.".format(
            len(target_voltages), self.channel_count)

        ids = []
        for channel in range(self.channel_count):
            target_voltage = target_voltages[channel]
            id = self.send_charge_or_discharge(
                execution_condition=ExecutionCondition.IMMEDIATE,
                channel=channel,
                target_voltage=target_voltage,
            )
            ids.append(id)

        return ids

    def send_immediate_full_discharge_to_all_channels(self):
        """
        Send immediate full discharge commands to all channels.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."

        target_voltages = self.channel_count * [0]

        ids = self.send_immediate_charge_or_discharge_to_all_channels(
            target_voltages=target_voltages,
        )

        return ids

    def send_timed_default_pulse_to_all_channels(self, reverse_polarities, time):
        """
        Send timed default pulse commands to all channels.

        Parameters
        ----------
        reverse_polarities : list of bools
            List of boolean values indicating whether to reverse polarities for each channel.
        time : float
            The time at which the pulse is executed.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."
        assert len(reverse_polarities) == self.channel_count, "Reverse polarities only defined for {} channels, channel count: {}.".format(
            len(reverse_polarities), self.channel_count)

        ids = []
        for channel in range(self.channel_count):
            reverse_polarity = reverse_polarities[channel]
            waveform = self.get_default_waveform(channel=channel)

            id = self.send_pulse(
                execution_condition=ExecutionCondition.TIMED,
                time=time,
                channel=channel,
                waveform=waveform,
                reverse_polarity=reverse_polarity,
            )
            ids.append(id)

        return ids

    def send_default_pulse_to_all_channels(self, reverse_polarities, time=None, execution_condition=ExecutionCondition.TIMED):
        """
        Send default pulse commands to all channels.

        Parameters
        ----------
        reverse_polarities : list of bools
            List of boolean values indicating whether to reverse polarities for each channel.
        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float
            The time at which the pulse is executed.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."
        assert len(reverse_polarities) == self.channel_count, "Reverse polarities only defined for {} channels, channel count: {}.".format(
            len(reverse_polarities), self.channel_count)

        ids = []
        for channel in range(self.channel_count):
            reverse_polarity = reverse_polarities[channel]
            waveform = self.get_default_waveform(channel=channel)

            id = self.send_pulse(
                execution_condition=execution_condition,
                time=time,
                channel=channel,
                waveform=waveform,
                reverse_polarity=reverse_polarity,
            )
            ids.append(id)

        return ids

    def send_immediate_default_pulse_to_all_channels(self, reverse_polarities):
        """
        Send immediate default pulse commands to all channels.

        Parameters
        ----------
        reverse_polarities : list of bools
            List of boolean values indicating whether to reverse polarities for each channel.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."

        time = self.get_time() + self.TIME_EPSILON

        ids = self.send_timed_default_pulse_to_all_channels(
            reverse_polarities=reverse_polarities,
            time=time,
        )
        return ids

    def send_timed_pulse_to_all_channels(self, waveforms_for_coil_set, time):
        """
        Send pulse command to all channels, using the given waveforms. Assumes that the waveform
        polarities are already as desired.

        Parameters
        ----------
        waveforms_for_coil_set : WaveformsForCoilSet object
        time : float
            The time at which the pulse is executed.

        Returns
        -------
        list
            IDs for each sent command.
        """
        assert self.is_session_started(), "Session not started."

        ids = []
        for channel in range(self.channel_count):
            waveform = waveforms_for_coil_set[channel]

            id = self.send_pulse(
                execution_condition=ExecutionCondition.TIMED,
                time=time,
                channel=channel,
                waveform=waveform,
                reverse_polarity=False,
            )
            ids.append(id)

        return ids

    def send_charge_or_discharge(self, channel, target_voltage, execution_condition=ExecutionCondition.TIMED, time=None):
        """
        Send charge or discharge command to a specified channel based on the current and target voltage.

        Parameters
        ----------
        channel : int
            Channel number.
        target_voltage : float
            Target voltage for the channel.
        execution_condition : ExecutionCondition, optional
            The condition under which the event should be executed. One of the following:

            * ExecutionCondition.IMMEDIATE : Execute the event immediately.
            * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
            * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.

            Default is ExecutionCondition.TIMED
        time : float, optional
            The time at which charging or discharging is executed. Only needs to be given if execution condition is ExecutionCondition.TIMED.

        Returns
        -------
        int
            ID of the sent command.
        """
        assert self.is_session_started(), "Session not started."
        assert not (execution_condition == ExecutionCondition.TIMED and time is None), "Execution condition is 'timed' but time not provided."
        assert not (execution_condition != ExecutionCondition.TIMED and time is not None), "Execution condition is not 'timed' but time is provided."

        voltage = self.get_current_voltage(channel=channel)
        charge_or_discharge = self.send_charge if voltage < target_voltage else self.send_discharge

        id = charge_or_discharge(
            channel=channel,
            target_voltage=target_voltage,
            execution_condition=execution_condition,
            time=time,
        )
        return id

    # MEP analysis

    def analyze_mep(self, time, mep_configuration):
        """
        Analyze an MEP (motor evoked potential) by passing the time, EMG (electromyogram) channel, and MEP configuration.

        Parameters
        ----------
        time : float
            The time at which MEP analysis begins.
        mep_configuration : object
            Configuration object for the MEP.

        Returns
        -------
        tuple
            A tuple containing the MEP message, consisting of the fields amplitude and latency, and any errors encountered during the analysis.
        """
        mep, errors = self.node.analyze_mep(
            time=time,
            mep_configuration=mep_configuration,
        )

        # HACK: ROS2 doesn't support NaN values, therefore they are encoded as 0.0 values. Do the decoding
        #   here. Maybe it would better to use non-zero errors instead as criterion for returning Nones?
        #
        if mep.amplitude == 0.0:
            mep.amplitude = None
        if mep.latency == 0.0:
            mep.latency = None

        return mep, errors

    # Experiment
    def get_experiment_handler(self):
        return self.node.get_experiment_handler()

    # Other

    def print_state(self):
        self.node.wait_for_new_state()
        self.node.print_state()

    def get_wallclock_time(self):
        return time.time()

    def end(self):
        """
        End API use. Call when the API is not used anymore.

        Does not require any parameters, and does not return any value.
        """
        self.node.destroy_node()
        rclpy.shutdown()
