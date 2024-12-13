classdef WaveformApproximatorWrapper < handle
    %WaveformApproximator A wrapper class for approximating waveforms.

    properties
        node
        target
        approximator

        approximate_waveform_server
        estimate_voltage_after_pulse_server
    end

    methods
        function obj = WaveformApproximatorWrapper()
            setenv("RMW_IMPLEMENTATION","rmw_cyclonedds_cpp")

            obj.node = ros2node("waveform_approximator");

            % Create ROS servers
            obj.approximate_waveform_server = ros2svcserver(obj.node, "/targeting/approximate_waveform", ...
                "targeting_interfaces/ApproximateWaveform", ...
                @obj.approximate_waveform_wrapper);

            obj.estimate_voltage_after_pulse_server = ros2svcserver(obj.node, "/targeting/estimate_voltage_after_pulse", ...
                "targeting_interfaces/EstimateVoltageAfterPulse", ...
                @obj.estimate_voltage_after_pulse);

            % Create the approximator object.
            time_resolution = 0.01e-6;
            solutions_filename = 'solutions_five_coil_set.mat';

            obj.approximator = WaveformApproximator(solutions_filename, time_resolution);
        end

        % Convert the ROS message to a MATLAB struct
        %
        % Example output:
        %
        % struct( ...
        %    'mode', {'r', 'h', 'f'}, ...
        %    'duration', {60 * 1e-6, 30 * 1e-6, 37 * 1e-6}, ...
        % );

        function waveform_struct = convert_ros_waveform_to_matlab_waveform(obj, waveform)
            pieces = waveform.pieces;

            % Loop through the pieces and extract the modes and durations
            modes = cell(1, length(pieces));
            durations = cell(1, length(pieces));

            for i = 1:length(pieces)
                piece = pieces(i);
                phase = piece.waveform_phase;

                % Map the waveform phase in ROS message to a character
                switch phase.value
                    case phase.RISING
                        mode = 'r';
                    case phase.HOLD
                        mode = 'h';
                    case phase.FALLING
                        mode = 'f';
                    case phase.ALTERNATIVE_HOLD
                        mode = 'a';
                    otherwise
                        assert (false, 'Invalid phase');
                end
                modes{i} = mode;

                % Convert ticks to seconds (1 tick = 25 ns)
                durations{i} = double(piece.duration_in_ticks) * 25 / 1e9;
            end

            % Create the MATLAB struct
            waveform_struct = struct( ...
                'mode', modes, ...
                'duration', durations ...
            );
        end

        % Convert the MATLAB struct to a ROS message
        function waveform = convert_matlab_waveform_to_ros_waveform(obj, waveform_struct)
            modes = [waveform_struct.mode];
            durations = [waveform_struct.duration];

            % Create the waveform ROS message
            waveform = ros2message("event_interfaces/Waveform");

            % Loop through the modes and durations
            waveform.pieces = [];
            for i = 1:length(modes)
                piece = ros2message("event_interfaces/WaveformPiece");
                phase = piece.waveform_phase;

                % Map the character to the waveform phase in ROS message
                switch modes(i)
                    case 'r'
                        value = phase.RISING;
                    case 'h'
                        value = phase.HOLD;
                    case 'f'
                        value = phase.FALLING;
                    case 'a'
                        value = phase.ALTERNATIVE_HOLD;
                    otherwise
                        assert (false, 'Invalid phase');
                end
                piece.waveform_phase.value = value;

                % Convert seconds to ticks (1 tick = 25 ns)
                piece.duration_in_ticks = uint16(durations(i) / 25 * 1e9);

                % Append the piece to the waveform ROS message
                waveform.pieces = [waveform.pieces piece];
            end
        end

        function response = approximate_waveform_wrapper(obj, request, response)
            actual_voltage = double(request.actual_voltage);
            target_voltage = double(request.target_voltage);
            target_waveform = request.target_waveform;
            coil_number = request.coil_number;

            % Convert the ROS message to a MATLAB struct
            target_waveform = obj.convert_ros_waveform_to_matlab_waveform(target_waveform);

            % Print the target waveform
            disp(' ');
            disp('Received target waveform:');
            disp(' ');
            disp('Modes:');
            disp([target_waveform.mode]);
            disp('Durations (microseconds):');
            disp([target_waveform.duration] * 1e6);

            % Select the coil.
            obj.approximator.select_coil(coil_number);

            % Approximate the waveform.
            [approximated_waveform, ~, success] = obj.approximator.approximate_iteratively(actual_voltage, target_voltage, target_waveform);

            if ~success
                disp('Approximation failed.');
            end

            % Populate the response message
            if success
                response.approximated_waveform = obj.convert_matlab_waveform_to_ros_waveform(approximated_waveform);
            end
            response.success = success;
        end

        function response = estimate_voltage_after_pulse(obj, request, response)
            voltage_before = double(request.voltage_before);
            waveform = request.waveform;
            coil_number = request.coil_number;

            % Convert the ROS message to a MATLAB struct
            target_waveform = obj.convert_ros_waveform_to_matlab_waveform(waveform);

            % Select the coil
            obj.approximator.select_coil(coil_number);

            % Calculate the voltage after pulse for both waveforms
            voltage_after = obj.approximator.estimate_voltage_after_pulse(voltage_before, target_waveform);

            % Populate the response message
            response.voltage_after = uint16(voltage_after);
            response.success = true;
        end
    end
end
