from util.bcolors import bcolors

from mtms_device_interfaces.msg import DeviceState, StartupError
from system_interfaces.msg import SessionState
from event_interfaces.msg import ExecutionCondition, PulseError, DischargeError, ChargeError, TriggerOutError

class MTMSApiEnums():
    DEVICE_STATES = (
        (DeviceState.NOT_OPERATIONAL, "Not operational", ""),
        (DeviceState.STARTUP, "Startup", bcolors.OKBLUE),
        (DeviceState.OPERATIONAL, "Operational", bcolors.OKGREEN),
        (DeviceState.SHUTDOWN, "Shutdown", bcolors.WARNING),
    )
    SESSION_STATES = (
        (SessionState.STOPPED, "Stopped", bcolors.OKBLUE),
        (SessionState.STARTING, "Starting", bcolors.OKBLUE),
        (SessionState.STARTED, "Started", bcolors.OKGREEN),
        (SessionState.STOPPING, "Stopping", bcolors.WARNING),
    )
    STARTUP_ERRORS = (
        (StartupError.NO_ERROR, "No error", bcolors.OKGREEN),
        (StartupError.UART_INITIALIZATION_ERROR, "UART initialization error", bcolors.FAIL),
        (StartupError.BOARD_STARTUP_ERROR, "Board startup error", bcolors.FAIL),
        (StartupError.BOARD_STATUS_MESSAGE_ERROR, "Board status message error", bcolors.FAIL),
        (StartupError.SAFETY_MONITOR_ERROR, "Safety monitor error", bcolors.FAIL),
        (StartupError.DISCHARGE_CONTROLLER_ERROR, "Discharge controller error", bcolors.FAIL),
        (StartupError.CHARGER_ERROR, "Charger error", bcolors.FAIL),
        (StartupError.SENSORBOARD_ERROR, "Sensorboard error", bcolors.FAIL),
        (StartupError.DISCHARGE_CONTROLLER_VOLTAGE_ERROR, "Discharge controller voltage error", bcolors.FAIL),
        (StartupError.CHARGER_VOLTAGE_ERROR, "Charger voltage error", bcolors.FAIL),
        (StartupError.IGBT_FEEDBACK_ERROR, "IGBT feedback error", bcolors.FAIL),
        (StartupError.TEMPERATURE_SENSOR_PRESENCE_ERROR, "Temperature sensor presence error", bcolors.FAIL),
        (StartupError.COIL_MEMORY_PRESENCE_ERROR, "Coil memory presence error", bcolors.FAIL),
    )
    PULSE_ERRORS = (
        (PulseError.NO_ERROR, "No error", bcolors.OKGREEN),
        (PulseError.INVALID_EXECUTION_CONDITION, "Invalid execution condition", bcolors.FAIL),
        (PulseError.INVALID_CHANNEL, "Invalid channel", bcolors.FAIL),
        (PulseError.INVALID_NUMBER_OF_WAVEFORM_PIECES, "Invalid number of pieces", bcolors.FAIL),
        (PulseError.INVALID_MODES, "Invalid modes", bcolors.FAIL),
        (PulseError.INVALID_DURATIONS, "Invalid durations", bcolors.FAIL),
        (PulseError.LATE, "Late", bcolors.FAIL),
        (PulseError.TOO_MANY_PULSES, "Too many pulses", bcolors.FAIL),
        (PulseError.OVERLAPPING_WITH_CHARGING, "Overlapping with charging", bcolors.FAIL),
        (PulseError.OVERLAPPING_WITH_DISCHARGING, "Overlapping with discharging", bcolors.FAIL),
        (PulseError.NOT_ALLOWED, "Not allowed", bcolors.FAIL),
        (PulseError.TRIGGERING_FAILURE, "Triggering failure", bcolors.FAIL),
        (PulseError.UNKNOWN_ERROR, "Unknown error", bcolors.FAIL),
    )
    CHARGE_ERRORS = (
        (ChargeError.NO_ERROR, "No error", bcolors.OKGREEN),
        (ChargeError.INVALID_EXECUTION_CONDITION, "Invalid execution condition", bcolors.FAIL),
        (ChargeError.INVALID_CHANNEL, "Invalid channel", bcolors.FAIL),
        (ChargeError.INVALID_VOLTAGE, "Invalid voltage", bcolors.FAIL),
        (ChargeError.LATE, "Late", bcolors.FAIL),
        (ChargeError.OVERLAPPING_WITH_DISCHARGING, "Overlapping with discharging", bcolors.FAIL),
        (ChargeError.OVERLAPPING_WITH_STIMULATION, "Overlapping with stimulation", bcolors.FAIL),
        (ChargeError.CHARGING_FAILURE, "Charging failure", bcolors.FAIL),
        (ChargeError.UNKNOWN_ERROR, "Unknown error", bcolors.FAIL),
    )
    DISCHARGE_ERRORS = (
        (DischargeError.NO_ERROR, "No error", bcolors.OKGREEN),
        (DischargeError.INVALID_EXECUTION_CONDITION, "Invalid execution condition", bcolors.FAIL),
        (DischargeError.INVALID_CHANNEL, "Invalid channel", bcolors.FAIL),
        (DischargeError.INVALID_VOLTAGE, "Invalid voltage", bcolors.FAIL),
        (DischargeError.LATE, "Late", bcolors.FAIL),
        (DischargeError.OVERLAPPING_WITH_CHARGING, "Overlapping with charging", bcolors.FAIL),
        (DischargeError.OVERLAPPING_WITH_STIMULATION, "Overlapping with stimulation", bcolors.FAIL),
        (DischargeError.DISCHARGING_FAILURE, "Discharging failure", bcolors.FAIL),
        (DischargeError.UNKNOWN_ERROR, "Unknown error", bcolors.FAIL),
    )
    TRIGGER_OUT_ERRORS = (
        (TriggerOutError.NO_ERROR, "No error", bcolors.OKGREEN),
        (TriggerOutError.INVALID_EXECUTION_CONDITION, "Invalid execution condition", bcolors.FAIL),
        (TriggerOutError.LATE, "Late", bcolors.FAIL),
        (TriggerOutError.TRIGGEROUT_FAILURE, "Trigger out failure", bcolors.FAIL),
        (TriggerOutError.UNKNOWN_ERROR, "Unknown error", bcolors.FAIL),
    )


class MTMSApiPrinter():
    TIME_COLOR = bcolors.OKBLUE
    EVENT_COLORS = {
        'Pulse': bcolors.HEADER,
        'Charge': bcolors.OKCYAN,
        'Discharge': bcolors.WARNING,
        'Trigger out': bcolors.OKGREEN,
    }

    # Print the system state every 5th iteration while waiting; this is to avoid spamming the
    # console with state prints, but still have some visibility on the state.
    PRINT_STATE_INTERVAL = 5

    def __init__(self, channel_count):
        self.channel_count = channel_count

        self.support_temperature = True
        self.support_pulse_count = True

        self.state_counter = 0

    def colored_text(self, text, color):
        return "{}{}{}".format(color, text, bcolors.ENDC)

    def enum_to_str(self, value, enums):
        for enum, text, color in enums:
            if value == enum:
                return self.colored_text(text, color)

        assert False, "Invalid value"

    def print_state(self, state, session, force):
        # Print the state occasionally to avoid spamming the console. However,
        # if 'force' argument is set, the state will be printed always.
        self.state_counter += 1
        if not force and self.state_counter < self.PRINT_STATE_INTERVAL:
            return

        self.state_counter = 0

        voltages_str = 'V: '
        temperatures_str = 'T: '
        pulse_counts_str = 'P: '

        for channel in range(self.channel_count):
            channel_state = state.channel_states[channel]

            voltage = '{:5}'.format(channel_state.voltage)
            voltages_str += voltage

            temperature = '{:4}'.format(channel_state.temperature)
            temperatures_str += temperature

            pulse_count = '{:6}'.format(channel_state.pulse_count)
            pulse_counts_str += pulse_count

        startup_error = state.startup_error
        device_state = state.device_state

        session_state = session.state

        time_str = '{}Time (s){}: {:.2f}'.format(self.TIME_COLOR, bcolors.ENDC, session.time)
        state_str = 'Device state: {}'.format(self.enum_to_str(device_state.value, MTMSApiEnums.DEVICE_STATES))
        session_str = 'Session: {}'.format(self.enum_to_str(session_state.value, MTMSApiEnums.SESSION_STATES))
        startup_error_str = 'Startup error: {}'.format(self.enum_to_str(startup_error.value, MTMSApiEnums.STARTUP_ERRORS))

        status_str = ', '.join(filter(None, [
            time_str,
            voltages_str,
            temperatures_str if self.support_temperature else None,
            pulse_counts_str if self.support_pulse_count else None,
            state_str,
            session_str,
            startup_error_str if startup_error != StartupError.NO_ERROR else None,
        ]))
        print(status_str)

    def print_event(self, event_type, event_info, channel=None, port=None):
        assert channel is not None or port is not None

        execution_condition = event_info.execution_condition.value
        id = event_info.id
        time = event_info.execution_time

        if execution_condition == ExecutionCondition.TIMED:
            execution_condition_str = 'Timed at (s): {0:g}'.format(time)
        elif execution_condition == ExecutionCondition.WAIT_FOR_TRIGGER:
            execution_condition_str = 'Waiting for trigger'
        elif execution_condition == ExecutionCondition.IMMEDIATE:
            execution_condition_str = 'Immediate'
        else:
            assert False, "Unknown execution condition"

        print('{}[Sent] {:10.10s} {}  {:7.7s}: {}  Event ID: {}, {}'.format(
            self.EVENT_COLORS[event_type],
            event_type,
            bcolors.ENDC,
            "Channel" if channel is not None else "Port",
            channel if channel is not None else port,
            id,
            execution_condition_str
        ))

    def print_feedback(self, event_type, feedback):
        value = feedback.error.value

        if event_type == "Pulse":
            error_str = self.enum_to_str(value, MTMSApiEnums.PULSE_ERRORS)
        elif event_type == "Charge":
            error_str = self.enum_to_str(value, MTMSApiEnums.CHARGE_ERRORS)
        elif event_type == "Discharge":
            error_str = self.enum_to_str(value, MTMSApiEnums.DISCHARGE_ERRORS)
        elif event_type == "Trigger out":
            error_str = self.enum_to_str(value, MTMSApiEnums.TRIGGER_OUT_ERRORS)

        print('{}[Done] {:10.10s} {}  {:7.7s}     Event ID: {}, Status: {}'.format(
            self.EVENT_COLORS[event_type],
            event_type,
            bcolors.ENDC,
            "",
            feedback.id,
            error_str
        ))

    def print_request_trigger(self):
        # HACK: This should probably be a feedback message that is received from the mTMS device, informing that the
        #   event trigger was successfully generated, similar to the feedback messages from the actual events.
        #
        print('{}Event trigger{}'.format(bcolors.OKGREEN, bcolors.ENDC))

    def print_heading(self, text):
        print('')
        print('{}{}{}{}{}{}'.format("", bcolors.HEADER, "\033[1m", text, "\033[0m", bcolors.ENDC))

    def print_experiment(self, experiment):
        metadata = experiment.metadata
        print('Experiment: {}'.format(metadata.experiment_name))
        print('Subject: {}'.format(metadata.subject_name))

    def print_experiment_feedback(self, feedback):
        attempt_number = feedback.attempt_number
        trial_number = feedback.trial_number
        total_trials = feedback.total_trials
        experiment_state = feedback.experiment_state.value

        print('Trial: {}/{}, Attempt: {}'.format(
            trial_number, total_trials, attempt_number,
        ))
