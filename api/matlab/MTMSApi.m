% This Matlab module provides an API for controlling a multi-channel transcranial magnetic stimulation (mTMS) device.
% It includes the functionality to start and stop the device, send pulses, charges, and discharges, and to perform
% various analyses of the EEG/EMG data.
%
% It uses the Robot Operating System (ROS2) to interact with the device.

classdef MTMSApi < handle
    % An API for controlling a multi-channel transcranial magnetic stimulation (mTMS) device.

    properties (Constant)
        % TIME_EPSILON is used to implement events that are to be executed immediately but
        % wanting to synchronize them: to do that, get current time, add TIME_EPSILON to it,
        % and execute all the events at that time.
        %
        % Consequently, TIME_EPSILON must be large enough to allow time to send the events to
        % the mTMS device, but not too large so that the events are not executed 'immediately'.
        %
        % TODO: In MATLAB, 0.1 seconds is too little. In Python, however, 0.1 s works fine.
        %   Settle for 1.5 s for now, but it could be investigated if the code could be optimized
        %   to allow a shorter time interval.
        %
        TIME_EPSILON = 1.5
    end

    properties
        node
        enums

        channel_count

        device_states
        session_states
        execution_conditions

        latest_event_id
        incomplete_events
    end

    methods

        function obj = MTMSApi(channel_count)
        % Initializes the MTMSApi object. Does not require any parameters.
        %
        % :returns: An MTMSApi object with five properties:
        %
        %   * 'node' - An instance of the MTMSApiNode class.
        %   * 'event_id' - A numerical ID, initialized to 0.
        %   * 'incomplete_events' - An empty list.
        %   * 'device_states' - A ROS2 message object, of type "mtms_device_interfaces/DeviceState".
        %   * 'session_states' - A ROS2 message object, of type "system_interfaces/SessionState".
        %   * 'execution_conditions' - A ROS2 message object, of type "event_interfaces/ExecutionCondition".

            % TODO: Should receive channel count automatically from .env so that the user of the API wouldn't have to care.

            % If channel count is not given, default to 5, which is the channel count of the Gen 2 mTMS devices.
            if nargin < 1
                channel_count = 5;
            end

            % Initialize object
            obj.node = MTMSApiNode(channel_count);

            obj.channel_count = channel_count;

            obj.latest_event_id = 0;
            obj.incomplete_events = [];

            obj.device_states = ros2message("mtms_device_interfaces/DeviceState");
            obj.session_states = ros2message("system_interfaces/SessionState");

            obj.execution_conditions = ros2message("event_interfaces/ExecutionCondition");

            % Print configuration
            disp("Configuration:")
            disp(" ")
            disp(sprintf("  Channel count: %d", channel_count))
        end

        % General

        function event_id = next_event_id(obj)
        % Increment the event ID and return the new ID.
        %
        % :returns: The incremented event ID.
        % :rtype: int

            obj.latest_event_id = obj.latest_event_id + 1;
            event_id = obj.latest_event_id;
        end

        function add_to_incomplete_events(obj, event_id)
        % Add given event ID to list of incomplete events.
        %
        % :param event_id: The event ID to add to the list.
        % :type event_id: int

            obj.incomplete_events = [obj.incomplete_events, event_id];
        end

        % Start and stop

        function start_device(obj)
        % Start the mTMS device, waiting until the device reports its state as operational.
        % Does not require any parameters. Does not return any value.

            obj.node.start_device();
            while obj.get_device_state() ~= obj.device_states.OPERATIONAL
                pause(0.1)
            end
        end

        function stop_device(obj)
        % Stop the mTMS device, waiting until the device reports its state as non-operational.
        % Does not require any parameters. Does not return any value.

            obj.node.stop_device();
            while obj.get_device_state() ~= obj.device_states.NOT_OPERATIONAL
                pause(0.1)
            end
        end

        function start_session(obj)
        % Start an session, waiting until the session state is reported as started.
        % Does not require any parameters. Does not return any value.

            obj.node.start_session();
            while obj.get_session_state() ~= obj.session_states.STARTED
                pause(0.1)
            end
        end

        function stop_session(obj)
        % Stop an session, waiting until the session state is reported as stopped.
        % Does not require any parameters. Does not return any value.

            obj.node.stop_session();
            while obj.get_session_state() ~= obj.session_states.STOPPED
                pause(0.1)
            end
        end

        % Wait

        function wait_forever(obj)
        % Continuously wait for a new system state indefinitely.
        % Does not require any parameters. Does not return any value.

            while true
                obj.node.wait_for_new_state();
            end
        end

        function wait_until(obj, time)
        % Wait until the system time is equal to or greater than the specified time.
        %
        % :param time: The time until which the function should wait.
        % :type time: float

            obj.node.wait_for_new_state();
            while obj.get_time() < time
                obj.node.wait_for_new_state();
            end
        end

        function wait_for_completion(obj, timeout)
            % Wait until the completion of all remaining events.
            %
            % :param timeout: The maximum time to wait for the completion of events. If not provided, wait indefinitely.
            % :type timeout: float, optional

            % Check if timeout is provided, otherwise set to Inf for no timeout.
            if nargin <= 1
                timeout = Inf;
            end

            start_time = obj.get_time();

            while ~isempty(obj.incomplete_events)
                obj.node.wait_for_new_state();

                % Pre-allocate a list for completed events.
                completed_events = [];

                % Check if any of the incomplete events are completed.
                for idx = 1:length(obj.incomplete_events)
                    id = obj.incomplete_events(idx);

                    % Event feedback, when available, is a struct, so we can use isstruct to check if it is available.
                    if isstruct(obj.get_event_feedback(id))
                        completed_events = [completed_events, idx];
                    end
                end

                % Remove the completed events.
                obj.incomplete_events(completed_events) = [];

                % Check if timeout has been reached.
                if ~isinf(timeout) && obj.get_time() - start_time > timeout
                    disp("Timeout reached while waiting for completion of events.");

                    % Remove the remaining incomplete events.
                    obj.incomplete_events = [];

                    return;
                end
            end
        end

        function wait(obj, time)
        % Wait for a specified amount of time.
        %
        % :param time: The time to wait.
        % :type time: float

            start_time = obj.get_wallclock_time();

            obj.node.wait_for_new_state();

            while obj.get_wallclock_time() < start_time + seconds(time)
                obj.node.wait_for_new_state();
            end
        end

        % Getters

        function state = get_device_state(obj)
        % Return the current state of the device.
        % Does not require any parameters.
        %
        % :return: The current state of the device. One of the following
        %
        %   * DeviceState.NOT_OPERATIONAL : Device is unoperational.
        %   * DeviceState.STARTUP : Device is starting up.
        %   * DeviceState.OPERATIONAL : Device is operational.
        %   * DeviceState.SHUTDOWN : Device is shutting down.
        % :rtype: int

            obj.node.wait_for_new_state();
            state = obj.node.system_state.device_state.value;
        end

        function state = get_session_state(obj)
        % Return the current state of the session.
        % Does not require any parameters.
        %
        % :return: The current state of the session. One of the following
        %
        %   * SessionState.STOPPED : Session is stopped.
        %   * SessionState.STARTING : Session is starting.
        %   * SessionState.STARTED : Session is started.
        %   * SessionState.STOPPING : Session is stopping.
        % :rtype: int

            state = obj.node.session.state.value;
        end

        function voltage = get_current_voltage(obj, channel)
        % Return the capacitor voltage (V) of the given channel.
        %
        % :param channel: The channel number. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :return: The capacitor voltage (V) of the specified channel.
        % :rtype: float

            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));

            obj.node.wait_for_new_state();

            % Array indexing in MATLAB starts from 1, so we need to add 1 to the channel number.
            voltage = double(obj.node.system_state.channel_states(channel + 1).voltage);
        end

        function voltages = get_current_voltages(obj)
        % Return the capacitor voltages (V) of all channels.

            obj.node.wait_for_new_state();

            voltages = zeros(1, obj.channel_count);
            for channel = 0:obj.channel_count - 1

                % Array indexing in MATLAB starts from 1, so we need to add 1 to the channel number.
                voltages(channel + 1) = obj.node.system_state.channel_states(channel + 1).voltage;
            end
        end

        function temperature = get_temperature(obj, channel)
        % Return the coil temperature of the given channel if a temperature sensor is present,
        % otherwise return None.
        %
        % :param channel: The channel number. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :return: The coil temperature of the specified channel.
        % :rtype: float

            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));

            obj.node.wait_for_new_state();

            % Array indexing in MATLAB starts from 1, so we need to add 1 to the channel number.
            temperature = obj.node.system_state.channel_states(channel + 1).temperature;
        end

        function pulse_count = get_pulse_count(obj, channel)
        % Return the total number of pulses generated with the coil connected to the specified channel.
        %
        % :param channel: The channel number. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :return: The total number of pulses generated.
        % :rtype: float

            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));

            obj.node.wait_for_new_state();

            % Array indexing in MATLAB starts from 1, so we need to add 1 to the channel number.
            pulse_count = obj.node.system_state.channel_states(channel + 1).pulse_count;
        end

        function time = get_time(obj)
        % Return the current time from the start of the session.
        %
        % :return: The current time as seconds.
        % :rtype: float

            obj.node.wait_for_new_state();
            time = obj.node.session.time;
        end

        function feedback = get_event_feedback(obj, id)
        % Return feedback for a specified event id.
        %
        % :param channel: The event ID.
        % :type channel: int
        % :return: The feedback.
        % :rtype: int

            feedback = obj.node.get_event_feedback(id);
        end

        % Events

        function started = is_session_started(obj)
        % Return whether session has started (true or false).
        %
        % :return: Whether session has started.
        % :rtype: bool

            started = obj.get_session_state() == obj.session_states.STARTED;
        end

        function success = allow_stimulation(obj, allow_stimulation)
        % Allow or disallow stimulation.
        %
        % :param allow_stimulation: Either true (allow) or false (disallow).
        %
        % :return: True if allowing or disallowing was successful.
        % :rtype: bool

            success = obj.node.allow_stimulation(allow_stimulation);
        end

        function id = send_pulse(obj, channel, waveform, reverse_polarity, execution_condition, time)
        % Send a pulse event to a specified channel.
        %
        % :param channel: The target channel. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :param waveform: A waveform object, as returned by, e.g., get_default_waveform.
        % :type waveform: ROS message (Waveform)
        % :param reverse_polarity: Whether to reverse the polarity of the waveform. Default is false.
        % :type reverse_polarity: bool, optional
        % :param execution_condition: The condition under which the event should be executed. One of the following:
        %
        %   * ExecutionCondition.IMMEDIATE : Execute the event immediately.
        %   * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
        %   * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.
        %
        %   Default is ExecutionCondition.TIMED
        % :type execution_condition: ExecutionCondition, optional
        % :param time: The time at which the pulse should be sent. Default is 0.0.
        % :type time: float, optional
        %
        % :return: The ID of the event.
        % :rtype: int
        %
        % .. note:: The event ID is incremented with each pulse sent.

            assert(obj.is_session_started(), "Session not started.");
            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));

            % Interpret NaN time as if time was not provided.
            is_time_provided = nargin == 6 && ~isnan(time);

            % Assert that time is provided if execution condition is 'timed'.
            assert(~(execution_condition == obj.execution_conditions.TIMED && ~is_time_provided), "Execution condition is 'timed', but no time provided.");

            % Assert that time is not provided if execution condition is not 'timed'.
            assert(~(execution_condition ~= obj.execution_conditions.TIMED && is_time_provided), "Execution condition is not 'timed', but time was provided.");

            if ~is_time_provided
                time = NaN;
            end

            id = obj.next_event_id();

            waveform_ = waveform;
            if reverse_polarity
                waveform_ = obj.reverse_polarity(waveform);
            end

            obj.node.send_pulse(id, channel, waveform_, execution_condition, time);

            obj.add_to_incomplete_events(id);
        end

        function id = send_charge(obj, channel, target_voltage, execution_condition, time)
        % Send a charge to a specified channel.
        %
        % :param channel: The channel for charging. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :param target_voltage: The target voltage for charging. Range: 0-1500
        % :type target_voltage: float
        % :param execution_condition: The condition under which the event should be executed. One of the following:
        %
        %   * ExecutionCondition.IMMEDIATE : Execute the event immediately.
        %   * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
        %   * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.
        %
        %   Default is ExecutionCondition.TIMED
        % :type execution_condition: ExecutionCondition, optional
        % :param time: The desired time for executing the event. Only used if execution_condition is ExecutionCondition.TIMED. Default is 0.0.
        % :type time: float, optional
        %
        % :return:  The ID of the event.
        % :rtype: int
        %
        % .. note:: The event ID is incremented with each charge sent.

            assert(obj.is_session_started(), "Session not started.");
            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));
            assert(target_voltage >= 0 && target_voltage <= 1500, sprintf("Voltage must be in range 0-1500"))

            % Interpret NaN time as if time was not provided.
            is_time_provided = nargin == 5 && ~isnan(time);

            % Assert that time is provided if execution condition is 'timed'.
            assert(~(execution_condition == obj.execution_conditions.TIMED && ~is_time_provided), "Execution condition is 'timed', but no time provided.");

            % Assert that time is not provided if execution condition is not 'timed'.
            assert(~(execution_condition ~= obj.execution_conditions.TIMED && is_time_provided), "Execution condition is not 'timed', but time was provided.");

            if ~is_time_provided
                time = NaN;
            end

            id = obj.next_event_id();
            obj.node.send_charge(id, channel, target_voltage, execution_condition, time);

            obj.add_to_incomplete_events(id);
        end

        function id = send_discharge(obj, channel, target_voltage, execution_condition, time)
        % Send a discharge to a specified channel.
        %
        % :param channel: The channel for discharging. The indexing starts from 0. Only supports the five first channels of the mTMS device. Range: 0-4
        % :type channel: int
        % :param target_voltage: The target voltage for the discharge.
        % :type target_voltage: float
        % :param execution_condition: The condition under which the event should be executed. One of the following:
        %
        %   * ExecutionCondition.IMMEDIATE : Execute the event immediately.
        %   * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
        %   * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.
        %
        %   Default is ExecutionCondition.TIMED
        % :type execution_condition: ExecutionCondition, optional
        % :param time: The desired time for executing the event. Only used if execution_condition is ExecutionCondition.TIMED. Default is 0.0.
        % :type time: float, optional
        %
        % :return: The ID of the event.
        % :rtype: int
        %
        % .. note:: The event ID is incremented with each discharge sent.

            assert(obj.is_session_started(), "Session not started.");
            assert(channel >= 0 && channel < obj.channel_count, sprintf("Channel must be in range 0-%d.", obj.channel_count - 1));

            % Interpret NaN time as if time was not provided.
            is_time_provided = nargin == 5 && ~isnan(time);

            % Assert that time is provided if execution condition is 'timed'.
            assert(~(execution_condition == obj.execution_conditions.TIMED && ~is_time_provided), "Execution condition is 'timed', but no time provided.");

            % Assert that time is not provided if execution condition is not 'timed'.
            assert(~(execution_condition ~= obj.execution_conditions.TIMED && is_time_provided), "Execution condition is not 'timed', but time was provided.");

            % XXX: If time is not provided, use a dummy value, as ROS2 messages require the field to have a value.
            if ~is_time_provided
                time = NaN;
            end

            % XXX: Discharging to 0-2 V can take a relatively long time; therefore, set the minimum target voltage to 3.
            %   In the long term, come up with a better solution.
            target_voltage = max(target_voltage, 3);

            id = obj.next_event_id();
            obj.node.send_discharge(id, channel, target_voltage, execution_condition, time);

            obj.add_to_incomplete_events(id);
        end

        function id = send_trigger_out(obj, port, duration_us, execution_condition, time)
        % Sends a trigger output to a specified port.
        %
        % :param port: The port number to send the trigger output to.
        % :type port: int
        % :param duration_us: The duration of the trigger in microseconds.
        % :type duration_us: int
        % :param execution_condition: The condition under which the event should be executed. One of the following:
        %
        %   * ExecutionCondition.IMMEDIATE : Execute the event immediately.
        %   * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
        %   * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.
        %
        %   Default is ExecutionCondition.TIMED
        % :type execution_condition: ExecutionCondition, optional
        % :param time: The time at which the trigger should be sent. Default is 0.0.
        % :type time: float, optional
        %
        % :return: The ID of the event.
        % :rtype: int
        %
        % .. note:: The event ID is incremented with each trigger sent.

            assert(obj.is_session_started(), "Session not started.");

            % Interpret NaN time as if time was not provided.
            is_time_provided = nargin == 5 && ~isnan(time);

            % Assert that time is provided if execution condition is 'timed'.
            assert(~(execution_condition == obj.execution_conditions.TIMED && ~is_time_provided), "Execution condition is 'timed', but no time provided.");

            % Assert that time is not provided if execution condition is not 'timed'.
            assert(~(execution_condition ~= obj.execution_conditions.TIMED && is_time_provided), "Execution condition is not 'timed', but time was provided.");

            % XXX: If time is not provided, use a dummy value, as ROS2 messages require the field to have a value.
            if ~is_time_provided
                time = NaN;
            end

            id = obj.next_event_id();
            obj.node.send_trigger_out(id, port, duration_us, execution_condition, time);

            obj.add_to_incomplete_events(id);
        end

        function request_trigger(obj)
        % Execute the events which have execution_condition set to ExecutionCondition.WAIT_FOR_TRIGGER.
        %
        % Does not require any parameters. Does not return any value.

            obj.node.request_trigger();
        end

        % Waveforms and targeting

        function waveform = create_waveform(obj, waveform_struct)
        % Create a waveform from an array of waveform mode structs.
        %
        % :param waveform_struct: A struct array, each struct containing a mode and a duration.
        %
        %   For instance, a single struct could be: struct('mode', 'r', 'duration', 60 * 1e-6).
        %
        %   The struct array can be created, e.g., by: struct('mode', {'r', 'h', 'f'}, 'duration', {60 * 1e-6, 30 * 1e-6, 37 * 1e-6}).
        %
        %   Modes can be either 'r' (rising), 'h' (hold), or 'f' (falling), or alternatively, 'RISING', 'HOLD', 'FALLING'.
        %
        %   The durations are in seconds.
        %
        % :return: A waveform object.
        % :rtype: ROS message (Waveform)

            modes = {waveform_struct.mode};
            durations = {waveform_struct.duration};

            assert(iscell(modes), "Modes must be given as a cell array, e.g., {'RISING', 'HOLD', 'FALLING'} or {'r', 'h', 'f'}.");
            assert(iscell(durations), "Durations must be given as a cell array, e.g., {60 * 1e-6, 30 * 1e-6, 37 * 1e-6}.");

            assert(length(modes) == length(durations), 'Length of modes must be equal to the length of durations.');

            waveform = ros2message('event_interfaces/Waveform');
            for i = 1:length(modes)
                mode = modes{i};
                duration_in_ticks = durations{i} / 25e-9;

                if mode == 'r'
                    mode = 'RISING';
                elseif mode == 'h'
                    mode = 'HOLD';
                elseif mode == 'f'
                    mode = 'FALLING';
                elseif mode == 'a'
                    mode = 'ALTERNATIVE_HOLD';
                end

                allowed_modes = {'NON_CONDUCTIVE', 'RISING', 'HOLD', 'FALLING', 'ALTERNATIVE_HOLD'};
                assert(ismember(mode, allowed_modes), ['Mode must be one of ' strjoin(allowed_modes, ', ')]);

                piece = ros2message('event_interfaces/WaveformPiece');
                piece.duration_in_ticks = uint16(duration_in_ticks);

                % TODO: Unify terms mode and phase.
                piece.waveform_phase.value = piece.waveform_phase.(mode);

                waveform.pieces(i) = piece;
            end
        end

        function waveforms_for_coil_set = create_waveforms_for_coil_set(obj, waveforms)
        % Create a WaveformsForCoilSet object from a list of waveforms.
        %
        % :param waveforms: A list of waveforms, one for each channel.
        % :type waveforms: list of ROS message (Waveform)
        %
        % :return: A WaveformsForCoilSet object.
        % :rtype: ROS message (WaveformsForCoilSet)

            waveforms_for_coil_set = ros2message('event_interfaces/WaveformsForCoilSet');
            for channel = 0:obj.channel_count - 1
                waveforms_for_coil_set.waveforms(channel + 1) = waveforms(channel + 1);
            end
        end

        function waveform = get_default_waveform(obj, channel)
            waveform = obj.node.get_default_waveform(channel);
        end

        function waveforms_for_coil_set = get_default_waveforms_for_coil_set(obj)
            waveforms_for_coil_set = ros2message('event_interfaces/WaveformsForCoilSet');
            for channel = 0:obj.channel_count - 1
                waveforms_for_coil_set.waveforms(channel + 1) = obj.get_default_waveform(channel);
            end
        end

        function waveform = reverse_polarity(obj, waveform)
            waveform = obj.node.reverse_polarity(waveform);
        end

        function [algorithm] = get_targeting_algorithm(obj, algorithm_str)
        % Return the targeting algorithm.
        %
        % :param algorithm_str: Either 'least_squares' or 'genetic'.
        % :type algorithm_str: string
        %
        % :return: Targeting algorithm.
        % :rtype: ROS message (TargetingAlgorithm)

            algorithm = ros2message('targeting_interfaces/TargetingAlgorithm');
            if strcmp(algorithm_str, 'least_squares')
                algorithm.value = algorithm.LEAST_SQUARES;
            elseif strcmp(algorithm_str, 'genetic')
                algorithm.value = algorithm.GENETIC;
            else
                error('Unknown targeting algorithm: %s', algorithm_str);
            end
        end

        function [voltages, reverse_polarities] = get_target_voltages(obj, target)
        % Return the target voltages (V), given a target ROS message.
        %
        % :param target: A target ROS message.
        % :type target: ROS message (ElectricTarget)
        %
        % :return: Target voltages.
        % :rtype: list of floats

            [voltages, reverse_polarities] = obj.node.get_target_voltages(target);
        end

        function maximum_intensity = get_maximum_intensity(obj, displacement_x, displacement_y, rotation_angle, algorithm)
        % Return the maximum intensity given the displacements and rotation angle.
        %
        % :param displacement_x: Displacement in the x direction.
        % :type displacement_x: float
        % :param displacement_y: Displacement in the y direction.
        % :type displacement_y: float
        % :param rotation_angle: Rotation angle in degrees.
        % :type rotation_angle: float
        %
        % :return: The maximum intensity.
        % :rtype: float

            maximum_intensity = obj.node.get_maximum_intensity(displacement_x, displacement_y, rotation_angle, algorithm);
        end

        function [initial_voltages, approximated_waveforms] = get_multipulse_waveforms(obj, targets)
        % Return the paired pulse waveforms for the first and second channels.
        %
        % :param targets: A list of ElectricTarget objects, each containing the displacement, rotation angle, intensity, and algorithm.
        % :type targets: list of ElectricTarget
        %
        % :return: Initial voltages for each coil.
        % :rtype: list of integers
        % :return: A WaveformsForCoilSet object for each target, each defining the waveforms for each coil.
        % :rtype: list of WaveformsForCoilSet

            target_waveforms = cell(1, length(targets));
            for i = 1:length(targets)
                target_waveforms{i} = obj.get_default_waveforms_for_coil_set();
            end

            [initial_voltages, approximated_waveforms] = obj.node.get_multipulse_waveforms(targets, target_waveforms);
        end

        % Other

        function [mep, errors] = analyze_mep(obj, time, mep_configuration)
        % Analyze an MEP (motor evoked potential) by passing the time, EMG (electromyogram) channel, and MEP configuration.
        %
        % :param time: Time point to analyze the MEP.
        % :type time: float
        % :param mep_configuration: Configuration object for the MEP.
        % :type mep_configuration: object
        %
        % :return: A MEP object containing amplitude and latency, and an errors object containing any errors encountered during the analysis.
        % :rtype: list

            [mep, errors] = obj.node.analyze_mep(time, mep_configuration);

            % HACK: ROS2 does not support NaN in float64 type, work around by using 0.0 instead of NaN in message; map to NaN here.
            if mep.amplitude == 0.0
                mep.amplitude = NaN;
            end
            if mep.latency == 0.0
                mep.amplitude = NaN;
            end
        end

        function mep_configuration = create_mep_configuration(obj, emg_channel, mep_start_time, mep_end_time, preactivation_check_enabled, preactivation_start_time, preactivation_end_time, preactivation_voltage_range_limit)
            % Create a MEP configuration object, given the EMG channel for the analysis, the start and end of time window, and
            % parameters defining the preactivation check.
            %
            % :param emg_channel: The EMG channel number.
            % :type emg_channel: int
            % :param mep_start_time: Start of the time window for MEP analysis, relative to the event of interest (e.g., pulse).
            % :type mep_start_time: float
            % :param mep_end_time: End of the time window for MEP analysis, relative to the event of interest (e.g., pulse).
            % :type mep_end_time: float
            % :param preactivation_check_enabled: Whether to enable the preactivation check.
            % :type preactivation_check_enabled: bool
            % :param preactivation_start_time: Start of the preactivation time window, relative to the event of interest (e.g., pulse).
            % :type mep_start_time: float
            % :param preactivation_end_time: End of the preactivation time window, relative to the event of interest (e.g., pulse).
            % :type mep_end_time: float
            % :param preactivation_voltage_range_limit: Voltage range limit; if the range of voltages within the preactivation time window
            %    exceeds this limit, the preactivation check will fail.
            % :type preactivation_voltage_range_limit: float
            %
            % :return: MEP configuration
            % :rtype: ROS message (MepConfiguration)

            mep_configuration = ros2message('mep_interfaces/MepConfiguration');

            mep_configuration.emg_channel = emg_channel;

            mep_configuration.time_window.start = mep_start_time;
            mep_configuration.time_window.end = mep_end_time;

            preactivation_check = ros2message('mep_interfaces/PreactivationCheck');

            preactivation_check.enabled = preactivation_check_enabled;
            preactivation_check.time_window.start = preactivation_start_time;
            preactivation_check.time_window.end = preactivation_end_time;
            preactivation_check.voltage_range_limit = preactivation_voltage_range_limit;

            mep_configuration.preactivation_check = preactivation_check;
        end

        function [target] = create_target(obj, displacement_x, displacement_y, rotation_angle, intensity, algorithm)
            % Create a target ROS message.
            %
            % :param displacement_x: Displacement in the x direction.
            % :type displacement_x: int
            % :param displacement_y: Displacement in the y direction.
            % :type displacement_y: int
            % :param rotation_angle: Rotation angle in degrees.
            % :type rotation_angle: int
            % :param intensity: Intensity value.
            % :type intensity: int
            % :param algorithm: Targeting algorithm.
            % :type algorithm: ROS message (TargetingAlgorithm)
            %
            % :return: Target message.
            % :rtype: ROS message (ElectricTarget)

            target = ros2message('targeting_interfaces/ElectricTarget');

            target.displacement_x = int8(displacement_x);
            target.displacement_y = int8(displacement_y);
            target.rotation_angle = uint16(rotation_angle);
            target.intensity = uint8(intensity);
            target.algorithm = algorithm;
        end

        % Compound events

        function id = send_charge_or_discharge(obj, channel, target_voltage, execution_condition, time)
        % Send charge or discharge command to a specified channel based on the current and target voltage.
        %
        % :param channel: Channel number.
        % :type channel: int
        % :param target_voltage: Target voltage for the channel.
        % :type target_voltage: float
        % :param execution_condition: The condition under which the event should be executed. One of the following:
        %
        %   * ExecutionCondition.IMMEDIATE : Execute the event immediately.
        %   * ExecutionCondition.TIMED : Execute the event when the desired time is reached.
        %   * ExecutionCondition.WAIT_FOR_TRIGGER : Execute the event when an external trigger is sent or a trigger command is sent.
        %
        %   Default is ExecutionCondition.TIMED
        % :type execution_condition: ExecutionCondition, optional
        % :param time: Time at when  before sending the pulse, by default 0.0.
        % :type time: float, optional
        %
        % :return: ID of the sent command.
        % :rtype: int

            assert(obj.is_session_started(), "Session not started.");

            % Interpret NaN time as if time was not provided.
            is_time_provided = nargin == 5 && ~isnan(time);

            % Assert that time is provided if execution condition is 'timed'.
            assert(~(execution_condition == obj.execution_conditions.TIMED && ~is_time_provided), "Execution condition is 'timed', but no time provided.");

            % Assert that time is not provided if execution condition is not 'timed'.
            assert(~(execution_condition ~= obj.execution_conditions.TIMED && is_time_provided), "Execution condition is not 'timed', but time was provided.");

            if ~is_time_provided
                time = NaN;
            end

            voltage = obj.get_current_voltage(channel);
            if voltage < target_voltage
                id = obj.send_charge(channel, target_voltage, execution_condition, time);
            else
                id = obj.send_discharge(channel, target_voltage, execution_condition, time);
            end
        end

        function ids = send_immediate_charge_or_discharge_to_all_channels(obj, target_voltages)
        % Send immediate charge or discharge commands to all channels.
        %
        % :param target_voltages: List of target voltages for each channel.
        % :type target_voltages: list of floats
        %
        % :return: list of event IDs for each sent command
        % :return type: list of ints

            assert(obj.is_session_started(), "Session not started.");

            assert(length(target_voltages) == obj.channel_count, sprintf("Target voltage defined for %d channels, but channel count is %d.", ...
                length(target_voltages), obj.channel_count));

            ids = [];

            % Channel indexing starts from 0, hence start the loop from 0.
            for channel = 0:obj.channel_count - 1

                % MATLAB indexing starts from 1, so we need to add 1 to the channel number, as we are indexing a MATLAB array.
                target_voltage = target_voltages(channel + 1);

                new_id = obj.send_charge_or_discharge(channel, target_voltage, obj.execution_conditions.IMMEDIATE);
                ids = [ids new_id];
            end
        end

        function ids = send_immediate_full_discharge_to_all_channels(obj)
        % Send immediate full discharge commands to all channels.
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            assert(obj.is_session_started(), "Session not started.");

            target_voltages = zeros(1, obj.channel_count);
            ids = obj.send_immediate_charge_or_discharge_to_all_channels(target_voltages);
        end

        function ids = send_timed_custom_pulse_to_all_channels(obj, waveforms_for_coil_set, time)
        % Send timed default pulse commands to all channels. Assumes that the waveform polarities are already as desired.
        %
        % :param waveforms_for_coil_set: Waveforms for each coil.
        % :type waveforms_for_coil_set: ROS message (WaveformsForCoilSet)
        % :param time: The time at which to execute the pulse.
        % :type time: float
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            waveforms = waveforms_for_coil_set.waveforms;

            assert(obj.is_session_started(), "Session not started.");

            assert(length(waveforms) == obj.channel_count, ...
                sprintf("Waveforms defined for %d channels, but channel count is %d.", length(waveforms), obj.channel_count));

            ids = [];

            % Channel indexing starts from 0, hence start the loop from 0.
            for channel = 0:obj.channel_count - 1

                % MATLAB indexing starts from 1, so we need to add 1 to the channel number, as we are indexing a MATLAB array.
                waveform = waveforms(channel + 1);

                new_id = obj.send_pulse(channel, waveform, false, obj.execution_conditions.TIMED, time);
                ids = [ids new_id];
            end
        end

        function ids = send_timed_default_pulse_to_all_channels(obj, reverse_polarities, time)
        % Send timed default pulse commands to all channels.
        %
        % :param reverse_polarities: List of boolean values indicating whether to reverse polarities for each channel.
        % :type reverse_polarities: list of bools
        % :param time: The time at which to execute the pulse. Default is 0.0.
        % :type time: float, optional
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            assert(obj.is_session_started(), "Session not started.");

            assert(length(reverse_polarities) == obj.channel_count, ...
                sprintf("Reverse polarities defined for %d channels, but channel count is %d.", length(reverse_polarities), obj.channel_count));

            ids = [];

            % Channel indexing starts from 0, hence start the loop from 0.
            for channel = 0:obj.channel_count - 1

                % MATLAB indexing starts from 1, so we need to add 1 to the channel number, as we are indexing a MATLAB array.
                reverse_polarity = reverse_polarities(channel + 1);

                % get_default_waveform is a ROS service call that uses 0-based indexing, hence no need to add 1 here.
                waveform = obj.get_default_waveform(channel);

                new_id = obj.send_pulse(channel, waveform, reverse_polarity, obj.execution_conditions.TIMED, time);
                ids = [ids new_id];
            end
        end

        function ids = send_immediate_default_pulse_to_all_channels(obj, reverse_polarities)
        % Send immediate default pulse commands to all channels.
        %
        % :param reverse_polarities: List of boolean values indicating whether to reverse polarities for each channel.
        % :type reverse_polarities: list of bools
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            assert(obj.is_session_started(), "Session not started.");

            time = obj.get_time() + obj.TIME_EPSILON;
            ids = obj.send_timed_default_pulse_to_all_channels(reverse_polarities, time);
        end

        function ids = send_timed_pulse_to_all_channels(obj, waveforms_for_coil_set, time)
        % Send pulse command to all channels, using the given waveforms. Assumes that the waveform
        % polarities are already as desired.
        %
        % :param waveforms_for_coil_set: Waveforms for each coil.
        % :type waveforms_for_coil_set: WaveformsForCoilSet
        % :param time: The time at which the pulse is executed.
        % :type time: float
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            assert(obj.is_session_started(), "Session not started.");

            ids = [];
            for channel = 0:obj.channel_count - 1
                waveform = waveforms_for_coil_set.waveforms(channel + 1);

                id = obj.send_pulse(channel, waveform, false, obj.execution_conditions.TIMED, time);
                ids = [ids id];
            end
        end

        function ids = send_immediate_custom_pulse_to_all_channels(obj, waveforms_for_coil_set)
        % Send immediate default pulse commands to all channels. Assumes that the waveform polarities are already as desired.
        %
        % :param waveforms_for_coil_set: Waveforms for each coil.
        % :type waveforms_for_coil_set: ROS message (WaveformsForCoilSet)
        %
        % :return: IDs for each sent command.
        % :rtype: list of ints

            assert(obj.is_session_started(), "Session not started.");

            time = obj.get_time() + obj.TIME_EPSILON;
            ids = obj.send_timed_custom_pulse_to_all_channels(waveforms_for_coil_set, time);
        end

        % Other

        function print_state(obj)
            obj.node.wait_for_new_state()
        end

        function time = get_wallclock_time(obj)
            time = datetime("now");
        end
    end
end
