classdef MTMSApiEnums < handle
    properties
        DEVICE_STATES
        SESSION_STATES
        STARTUP_ERRORS
        PULSE_ERRORS
        CHARGE_ERRORS
        DISCHARGE_ERRORS
        TRIGGER_OUT_ERRORS
    end

    methods
        function obj = MTMSApiEnums()
            device_state = ros2message("mtms_device_interfaces/DeviceState");
            session_state = ros2message("system_interfaces/SessionState");
            startup_error = ros2message("mtms_device_interfaces/StartupError");

            pulse_error = ros2message("event_interfaces/PulseError");
            charge_error = ros2message("event_interfaces/ChargeError");
            discharge_error = ros2message("event_interfaces/DischargeError");
            trigger_out_error = ros2message("event_interfaces/TriggerOutError");

            obj.DEVICE_STATES = {
                {device_state.NOT_OPERATIONAL,  "Not operational", ""},
                {device_state.STARTUP, "Startup", "blue"},
                {device_state.OPERATIONAL, "Operational", "green"},
                {device_state.SHUTDOWN, "Shutdown", "warning"}
            };
            obj.SESSION_STATES = {
                {session_state.STOPPED, "Stopped", "blue"},
                {session_state.STARTING, "Starting", "blue"},
                {session_state.STARTED, "Started", "green"},
                {session_state.STOPPING, "Stopping", "warning"},
            };
            obj.STARTUP_ERRORS = {
                {startup_error.NO_ERROR, "No error", "green"},
                {startup_error.UART_INITIALIZATION_ERROR, "UART initialization error", "fail"},
                {startup_error.BOARD_STARTUP_ERROR, "Board startup error", "fail"},
                {startup_error.BOARD_STATUS_MESSAGE_ERROR, "Board status message error", "fail"},
                {startup_error.SAFETY_MONITOR_ERROR, "Safety monitor error", "fail"},
                {startup_error.DISCHARGE_CONTROLLER_ERROR, "Discharge controller error", "fail"},
                {startup_error.CHARGER_ERROR, "Charger error", "fail"},
                {startup_error.SENSORBOARD_ERROR, "Sensorboard error", "fail"},
                {startup_error.DISCHARGE_CONTROLLER_VOLTAGE_ERROR, "Discharge controller voltage error", "fail"},
                {startup_error.CHARGER_VOLTAGE_ERROR, "Charger voltage error", "fail"},
                {startup_error.IGBT_FEEDBACK_ERROR, "IGBT feedback error", "fail"},
                {startup_error.TEMPERATURE_SENSOR_PRESENCE_ERROR, "Temperature sensor presence error", "fail"},
                {startup_error.COIL_MEMORY_PRESENCE_ERROR, "Coil memory presence error", "fail"},
            };
            obj.PULSE_ERRORS = {
                {pulse_error.NO_ERROR, "No error", "green"},
                {pulse_error.INVALID_EXECUTION_CONDITION, "Invalid execution condition", "fail"},
                {pulse_error.INVALID_CHANNEL, "Invalid channel", "fail"},
                {pulse_error.INVALID_NUMBER_OF_WAVEFORM_PIECES, "Invalid number of pieces", "fail"},
                {pulse_error.INVALID_MODES, "Invalid modes", "fail"},
                {pulse_error.INVALID_DURATIONS, "Invalid durations", "fail"},
                {pulse_error.LATE, "Late", "fail"},
                {pulse_error.TOO_MANY_PULSES, "Too many pulses", "fail"},
                {pulse_error.OVERLAPPING_WITH_CHARGING, "Overlapping with charging", "fail"},
                {pulse_error.OVERLAPPING_WITH_DISCHARGING, "Overlapping with discharging", "fail"},
                {pulse_error.NOT_ALLOWED, "Not allowed", "fail"},
                {pulse_error.TRIGGERING_FAILURE, "Triggering failure", "fail"},
                {pulse_error.UNKNOWN_ERROR, "Unknown error", "fail"},
            };
            obj.CHARGE_ERRORS = {
                {charge_error.NO_ERROR, "No error", "green"},
                {charge_error.INVALID_EXECUTION_CONDITION, "Invalid execution condition", "fail"},
                {charge_error.INVALID_CHANNEL, "Invalid channel", "fail"},
                {charge_error.INVALID_VOLTAGE, "Invalid voltage", "fail"},
                {charge_error.LATE, "Late", "fail"},
                {charge_error.OVERLAPPING_WITH_DISCHARGING, "Overlapping with discharging", "fail"},
                {charge_error.OVERLAPPING_WITH_STIMULATION, "Overlapping with stimulation", "fail"},
                {charge_error.CHARGING_FAILURE, "Charging failure", "fail"},
                {charge_error.UNKNOWN_ERROR, "Unknown error", "fail"},
            };
            obj.DISCHARGE_ERRORS = {
                {discharge_error.NO_ERROR, "No error", "green"},
                {discharge_error.INVALID_EXECUTION_CONDITION, "Invalid execution condition", "fail"},
                {discharge_error.INVALID_CHANNEL, "Invalid channel", "fail"},
                {discharge_error.INVALID_VOLTAGE, "Invalid voltage", "fail"},
                {discharge_error.LATE, "Late", "fail"},
                {discharge_error.OVERLAPPING_WITH_CHARGING, "Overlapping with charging", "fail"},
                {discharge_error.OVERLAPPING_WITH_STIMULATION, "Overlapping with stimulation", "fail"},
                {discharge_error.DISCHARGING_FAILURE, "Discharging failure", "fail"},
                {discharge_error.UNKNOWN_ERROR, "Unknown error", "fail"},
            };
            obj.TRIGGER_OUT_ERRORS = {
                {trigger_out_error.NO_ERROR, "No error", "green"},
                {trigger_out_error.INVALID_EXECUTION_CONDITION, "Invalid execution condition", "fail"},
                {trigger_out_error.LATE, "Late", "fail"},
                {trigger_out_error.TRIGGEROUT_FAILURE, "Trigger out failure", "fail"},
                {trigger_out_error.UNKNOWN_ERROR, "Unknown error", "fail"},
            };
        end
    end
end
