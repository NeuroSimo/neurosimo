function sampling_points = sample_state_trajectory_by_waveform(obj, state_trajectory, target_waveform, num_of_intermediate_points_per_mode)

    % Generate times for sampling points, based on the durations and the number of sampling points
    sampling_points = struct('time', {}, 'index', {}, 'state', {}, 'mode_info', {});

    assert(length(target_waveform) == length(num_of_intermediate_points_per_mode), ...
        'The number of intermediate points per mode must be equal to the number of modes in the target waveform.');

    start_time = 0;
    for i = 1:length(target_waveform)
        duration = target_waveform(i).duration;
        num_of_intermediate_points = num_of_intermediate_points_per_mode(i);

        sampling_times = start_time + linspace(0, duration, 2 + num_of_intermediate_points);

        if i ~= length(target_waveform)
            sampling_times = sampling_times(1:end - 1);
        end
        for j = 1:length(sampling_times)
            time = sampling_times(j);
            index = obj.get_index_from_time(time);

            current_mode = target_waveform(i).mode;
            is_last_segment = j == length(sampling_times);

            mode_info = struct('current_mode', current_mode, 'mode_index', i, 'segment_index', j, 'is_last_segment', is_last_segment);

            state = state_trajectory(index);

            sampling_point = struct( ...
                'time', time, ...
                'index', index, ...
                'state', state, ...
                'mode_info', mode_info);

            sampling_points = [sampling_points, sampling_point];
        end
        start_time = start_time + duration;
    end
end
