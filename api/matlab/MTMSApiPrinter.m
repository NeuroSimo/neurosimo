classdef MTMSApiPrinter < handle
    properties (Constant)
        TIME_COLOR = "blue"
        EVENT_COLORS = dictionary( ...
            ['Pulse', 'Charge', 'Discharge', 'Trigger out'], ...
            ['header', 'cyan', 'warning', 'green'])
    end

    properties
        channel_count
        support_temperature
        support_pulse_count

        print_every_nth_state
        state_counter

        mtmsapi_enums
    end

    methods
        function obj = MTMSApiPrinter(channel_count)
            obj.channel_count = channel_count;
            obj.support_temperature = true;
            obj.support_pulse_count = true;

            obj.print_every_nth_state = 5;
            obj.state_counter = 0;

            obj.mtmsapi_enums = MTMSApiEnums();
        end

        function str = colored_text(obj, text, color)
            % TODO: Color ignored for now; it is not simple to print
            % colored text in MATLAB.
            str = sprintf("%s", text);
        end

        function str = enum_to_str(obj, value, enums)
            str = "";
            for i = 1:numel(enums)
                % TODO: Using dictionaries for storing different attributes
                % of enums in MTMSApiEnums would be good.
                if enums{i}{1} == value
                    str = obj.colored_text(enums{i}{2}, enums{i}{3});
                end
            end
            assert(str ~= "", "Invalid value")
        end

        function print_state(obj, state, session)
            obj.state_counter = obj.state_counter + 1;

            if obj.state_counter ~= obj.print_every_nth_state
                return
            end

            obj.state_counter = 0;

            voltages_str = "V: ";
            temperatures_str = "T: ";
            pulse_counts_str = "P: ";

            for channel = 1:obj.channel_count
                channel_state = state.channel_states(channel);

                voltage = sprintf("%d ", channel_state.voltage);
                voltages_str = strcat(voltages_str, voltage);

                temperature = sprintf("%d ", channel_state.temperature);
                temperatures_str = strcat(temperatures_str, temperature);

                pulse_count = sprintf("%d ", channel_state.pulse_count);
                pulse_counts_str = strcat(pulse_counts_str, pulse_count);
            end

            startup_error = state.startup_error;
            device_state = state.device_state;

            session_state = session.state;

            % TODO: TIME_COLOR could be used here; colors unused at the moment.
            time_str = sprintf("Time (s): %.2f", session.time);

            state_str = sprintf("Device state: %s", obj.enum_to_str(device_state.value, obj.mtmsapi_enums.DEVICE_STATES));
            session_str = sprintf("Session: %s", obj.enum_to_str(session_state.value, obj.mtmsapi_enums.SESSION_STATES));
            startup_error_str = sprintf("Startup error: %s", obj.enum_to_str(startup_error.value, obj.mtmsapi_enums.STARTUP_ERRORS));

            if ~obj.support_temperature
                temperatures_str = NaN;
            end
            if ~obj.support_pulse_count
                pulse_counts_str = NaN;
            end

            if startup_error.value == startup_error.NO_ERROR
                startup_error_str = NaN;
            end

            strs = [time_str, voltages_str, temperatures_str, pulse_counts_str, state_str, session_str, startup_error_str];
            strs = rmmissing(strs);

            status_str = strjoin(strs, ", ");

            fprintf(strcat(status_str, "\n"));
        end

        function print_event(obj, event_type, event, channel, port)

            execution_condition = event.execution_condition.value;
            id = event.id;
            time = event.time;

            if execution_condition == ExecutionCondition.TIMED
                execution_condition_str = sprintf("Timed at (s): %.2f", time);
            elseif execution_condition == ExecutionCondition.WAIT_FOR_TRIGGER
                execution_condition_str = 'Waiting for trigger';
            elseif execution_condition == ExecutionCondition.IMMEDIATE
                execution_condition_str = 'Instant';
            else
                assert(false, "Unknown execution condition");
            end

            % TODO: EVENT_COLORS[event_type] should color "[Sent]" part of the string.
            port_or_channel_str = sprintf("Port: %d", port);
            if channel > 0
                port_or_channel_str = sprintf("Channel: %d", channel);
            end

            str = sprintf("[Sent] %s  Event ID: %d, %s\n", port_or_channel_str, id, execution_condition_str);
            fprintf(str);
        end

        function print_feedback(obj, event_type, feedback)
            value = feedback.error.value;

            if event_type == "Pulse"
                error_str = obj.enum_to_str(value, obj.mtmsapi_enums.PULSE_ERRORS);
            elseif event_type == "Charge"
                error_str = obj.enum_to_str(value, obj.mtmsapi_enums.CHARGE_ERRORS);
            elseif event_type == "Discharge"
                error_str = obj.enum_to_str(value, obj.mtmsapi_enums.DISCHARGE_ERRORS);
            elseif event_type == "Trigger out"
                error_str = obj.enum_to_str(value, obj.mtmsapi_enums.TRIGGER_OUT_ERRORS);
            end

            % TODO: "[Done]" part of the string should be colored by EVENT_COLORS[event_type].
            str = sprintf("[Done] %s  Event ID: %d, Status: %s\n", event_type, feedback.id, error_str);
            fprintf(str);
        end

        function print_request_trigger(obj)
            % HACK: This should probably be a feedback message that is received from the mtms_device, informing that the
            %   event trigger was successfully generated, similar to the feedback messages from the actual events.
            %
            fprintf("Event trigger\n");
        end

        function print_heading(obj, text)
            fprintf("\n");

            % TODO: Could be bold-faced.
            fprintf("%s", text);
        end
    end
end
