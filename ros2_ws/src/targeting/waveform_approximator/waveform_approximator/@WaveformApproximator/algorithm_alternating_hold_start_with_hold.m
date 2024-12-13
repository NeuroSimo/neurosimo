function waveform = algorithm_alternating_hold(obj, parameter, total_duration, mode_info)
    current_mode = mode_info.current_mode;
    is_last_segment = mode_info.is_last_segment;
    segment_index = mode_info.segment_index;
    mode_index = mode_info.mode_index;

    is_even_segment = mod(segment_index, 2) == 0;

    if current_mode == 'r'
        if is_even_segment
            waveform = struct( ...
                'mode', {'r', 'h'}, ...
                'duration', {parameter, total_duration - parameter});
        else
            waveform = struct( ...
                'mode', {'h', 'r'}, ...
                'duration', {parameter, total_duration - parameter});
        end

        % Always approximate with hold-rise if it is the last mode.
        if is_last_segment
            waveform = struct( ...
                'mode', {'h', 'r'}, ...
                'duration', {parameter, total_duration - parameter});
        end

    elseif current_mode == 'f'
        if is_even_segment || is_last_segment
            waveform = struct( ...
                'mode', {'f', 'h'}, ...
                'duration', {parameter, total_duration - parameter});
        else
            waveform = struct( ...
                'mode', {'h', 'f'}, ...
                'duration', {parameter, total_duration - parameter});
        end

        % Always approximate with hold-fall if it is the last mode.
        if is_last_segment
            waveform = struct( ...
                'mode', {'h', 'f'}, ...
                'duration', {parameter, total_duration - parameter});
        end

    elseif current_mode == 'h'
        waveform = struct( ...
            'mode', {'h'}, ...
            'duration', {total_duration});

    elseif current_mode == 'a'
        waveform = struct( ...
            'mode', {'a'}, ...
            'duration', {total_duration});

    else
        error('Invalid mode: %s', current_mode);
    end
end
