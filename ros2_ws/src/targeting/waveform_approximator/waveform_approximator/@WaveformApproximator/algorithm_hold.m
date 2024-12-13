function waveform = algorithm_hold(obj, parameter, total_duration, mode_info)
    current_mode = mode_info.current_mode;
    is_last_segment = mode_info.is_last_segment;
    segment_index = mode_info.segment_index;
    mode_index = mode_info.mode_index;

    if current_mode == 'r'
        waveform = struct( ...
            'mode', {'r', 'h'}, ...
            'duration', {parameter, total_duration - parameter});

    elseif current_mode == 'f'
        waveform = struct( ...
            'mode', {'h', 'f'}, ...
            'duration', {parameter, total_duration - parameter});

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
