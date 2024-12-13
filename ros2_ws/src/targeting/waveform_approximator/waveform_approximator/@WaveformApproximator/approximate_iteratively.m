function [approximated_waveform, sampling_points, success] = approximate_iteratively(obj, actual_voltage, target_voltage, target_waveform)

    % Check that the target voltage does not exceed the actual voltage.
    if target_voltage > actual_voltage
        error('The target voltage exceeds the actual voltage.');
    end

    relative_error_threshold = 0.02;

    % Generate the state trajectory for the target waveform.
    state_trajectory = obj.generate_state_trajectory_from_waveform(target_voltage, target_waveform);

    % Define the algorithms and corresponding intermediate points
    algorithms = {};

    % Start from 3 intermediate points and decrease to 0.
    for i = 3:-1:0
        algorithm = struct( ...
            'algorithm', @obj.algorithm_alternating_hold_start_with_non_hold, ...
            'algorithm_name', 'alternating hold start with non-hold', ...
            'intermediate_points', i, ...
            'check_duration', true, ...
            'check_relative_error', true);

        algorithms{end+1} = algorithm;

        algorithm = struct( ...
            'algorithm', @obj.algorithm_alternating_hold_start_with_hold, ...
            'algorithm_name', 'alternating hold start with hold', ...
            'intermediate_points', i, ...
            'check_duration', true, ...
            'check_relative_error', true);

        algorithms{end+1} = algorithm;
    end

    % Add the micropulse and constant hold algorithms with 0 intermediate points
    algorithms{end+1} = struct( ...
        'algorithm', @obj.algorithm_micropulse, ...
        'algorithm_name', 'micropulse', ...
        'intermediate_points', 0, ...
        'check_duration', true, ...
        'check_relative_error', true);

    % The constant hold is only used for very low target voltages, where the approximation
    % might contain large relative errors. For that reason, the relative errors are not checked.
    algorithms{end+1} = struct( ...
        'algorithm', @obj.algorithm_constant_hold, ...
        'algorithm_name', 'constant hold', ...
        'intermediate_points', 0, ...
        'check_duration', true, ...
        'check_relative_error', false);

    success = false;
    for i = 1:length(algorithms)
        intermediate_points = algorithms{i}.intermediate_points;
        algorithm = algorithms{i}.algorithm;
        algorithm_name = algorithms{i}.algorithm_name;
        check_duration = algorithms{i}.check_duration;
        check_relative_error = algorithms{i}.check_relative_error;

        disp(['Trying with ', num2str(intermediate_points), ' intermediate points per mode using ''', algorithm_name, ''' algorithm.']);

        % Calculate the number of intermediate points for each mode.
        num_of_intermediate_points_per_mode = calculate_num_of_intermediate_points_per_mode(target_waveform, intermediate_points);

        % Sample the state trajectory using the waveform.
        sampling_points = obj.sample_state_trajectory_by_waveform(state_trajectory, target_waveform, num_of_intermediate_points_per_mode);

        % Approximate the waveform.
        [approximated_waveform, relative_errors] = obj.approximate(actual_voltage, sampling_points, algorithm);

        % Check that all waveform phases are ok, that is, they have a long enough duration.
        waveform_durations_ok = are_waveform_durations_ok(approximated_waveform, relative_errors);

        % Check that all waveform phases are ok, that is, they have a small enough relative error.
        relative_errors_ok = are_relative_errors_ok(relative_errors);

        if (~check_duration || waveform_durations_ok) && (~check_relative_error || relative_errors_ok)
            success = true;
            disp('  The approximation was successful.');

            break;
        end
        disp('  The approximation failed.');
    end

    function num_of_intermediate_points_per_mode = calculate_num_of_intermediate_points_per_mode(target_waveform, num_of_intermediate_points_for_rising_and_falling_modes)
        % Create number of intermediate points for each mode by using as many as possible for falling and rising modes,
        % and always zero for hold modes.
        num_of_intermediate_points_per_mode = zeros(1, length(target_waveform));
        for i = 1:length(target_waveform)
            if target_waveform(i).mode == 'f' || target_waveform(i).mode == 'r'
                num_of_intermediate_points_per_mode(i) = num_of_intermediate_points_for_rising_and_falling_modes;
            end
        end
    end

    % Define waveform durations to be ok if all phases have a duration longer than the threshold,
    % except initial and final phases if they are 'hold'.
    function is_ok = are_waveform_durations_ok(waveform, relative_errors)
        is_ok = true;
        for i = 1:length(waveform)
            % For first and last modes, check that if they are not 'hold', they have a duration longer than the threshold.
            first_and_last_mode_not_ok = (i == 1 || i == length(waveform)) && waveform(i).mode ~= 'h' && waveform(i).duration < obj.minimum_mode_duration;

            % For other modes, check that they have a duration longer than the threshold.
            other_mode_not_ok = i ~= 1 && i ~= length(waveform) && waveform(i).duration < obj.minimum_mode_duration;

            if first_and_last_mode_not_ok || other_mode_not_ok
                disp(['  Duration ', num2str(waveform(i).duration * 1e6), ' us of mode ', waveform(i).mode, ' at sampling point ', num2str(i), ' is too short.']);
                is_ok = false;
                break;
            end
        end
    end

    % Define waveform phases to be ok if all relative errors are smaller than the threshold.
    function is_ok = are_relative_errors_ok(relative_errors)
        is_ok = true;
        for i = 1:length(relative_errors)
            if relative_errors(i) > relative_error_threshold
                disp(['  Relative error ', num2str(100 * relative_errors(i)), '% of current at sampling point ', num2str(i), ' is too high.']);
                is_ok = false;
                break;
            end
        end
    end
end
