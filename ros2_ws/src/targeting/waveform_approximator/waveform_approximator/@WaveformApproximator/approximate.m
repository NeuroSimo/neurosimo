function [approximated_waveform relative_errors] = approximate(obj, actual_voltage, sampling_points, algorithm)

    states = [sampling_points.state];
    I_coils = [states.I_coil];
    max_I_coil = max(abs(I_coils));

    % Generate initial conditions.
    initial_state = obj.generate_initial_state(actual_voltage);

    %% Loop through the sampling points and approximate the waveform.
    approximated_waveform = struct('mode', {}, 'duration', {});

    relative_errors = zeros(1, length(sampling_points) - 1);

    for i = 1:length(sampling_points) - 1
        mode_info = sampling_points(i).mode_info;

        % Calculate total duration between two consecutive points
        total_duration = sampling_points(i + 1).time - sampling_points(i).time;

        % Initialize search bounds
        lower_bound = 0;
        upper_bound = total_duration;

        target_state = sampling_points(i + 1).state;

        error_function = @(parameter) obj.calculate_error(parameter, algorithm, initial_state, target_state, total_duration, mode_info);

        [parameter, error] = obj.golden_section_search( ...
            memoize(error_function), ...
            lower_bound, ...
            upper_bound);

        relative_errors(i) = error / max_I_coil;

        % Approximate the mode with the calculated parameter
        waveform = algorithm(parameter, total_duration, mode_info);

        % Update the initial state for the next iteration
        initial_state = obj.apply_waveform_to_state(initial_state, waveform);

        % Append the approximated waveform
        for i = 1:length(waveform)
            mode = waveform(i).mode;
            duration = waveform(i).duration;

            approximated_waveform = [approximated_waveform, struct('mode', mode, 'duration', duration)];
        end
    end
    approximated_waveform = obj.merge_consecutive_modes(approximated_waveform);
end
