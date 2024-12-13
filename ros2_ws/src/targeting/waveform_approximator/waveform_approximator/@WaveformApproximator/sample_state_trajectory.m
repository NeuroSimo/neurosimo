function sampling_points = sample_state_trajectory(obj, state_trajectory, sampling_times)

    % Generate times for sampling points, based on the durations and the number of sampling points
    sampling_points = struct('time', {}, 'index', {}, 'state', {}, 'mode_info', {});

    for j = 1:length(sampling_times)
        time = sampling_times(j);
        index = obj.get_index_from_time(time);
        state = state_trajectory(index);

        if j ~= length(sampling_times)
            next_time = sampling_times(j + 1);
            next_index = obj.get_index_from_time(next_time);
            next_state = state_trajectory(next_index);

            time_difference = next_time - time;
            next_state_when_holding = obj.apply_mode_to_state(state, 'h', time_difference);

            if next_state_when_holding.I_coil > next_state.I_coil
                current_mode = 'r';
            else
                current_mode = 'f';
            end
        else
            current_mode = 'h';
        end

        mode_info = struct('current_mode', current_mode, 'mode_index', j, 'segment_index', 1, 'is_last_segment', true);

        sampling_point = struct( ...
            'time', time, ...
            'index', index, ...
            'state', state, ...
            'mode_info', mode_info);

        sampling_points = [sampling_points, sampling_point];
    end
end
