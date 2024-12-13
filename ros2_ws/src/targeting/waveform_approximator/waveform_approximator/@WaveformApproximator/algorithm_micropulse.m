function waveform = algorithm_micropulse(obj, parameter, total_duration, mode_info)
    current_mode = mode_info.current_mode;
    is_last_segment = mode_info.is_last_segment;
    segment_index = mode_info.segment_index;
    mode_index = mode_info.mode_index;

    assert (mode_index <= 3, 'Micropulse algorithm only supports waveforms with up to 3 modes.');

    if current_mode == 'r' && mode_index == 1
        waveform = struct( ...
            'mode', {'h', 'f', 'r'}, ...
            'duration', {parameter, obj.minimum_mode_duration, total_duration - parameter - obj.minimum_mode_duration});

    elseif current_mode == 'r' && mode_index > 1
        waveform = struct( ...
            'mode', {'r', 'f', 'h'}, ...
            'duration', {parameter, obj.minimum_mode_duration, total_duration - parameter - obj.minimum_mode_duration});

    elseif current_mode == 'f' && mode_index == 1
        waveform = struct( ...
            'mode', {'h', 'r', 'f'}, ...
            'duration', {parameter, obj.minimum_mode_duration, total_duration - parameter - obj.minimum_mode_duration});

    elseif current_mode == 'f' && mode_index > 1
        waveform = struct( ...
            'mode', {'f', 'r', 'h'}, ...
            'duration', {parameter, obj.minimum_mode_duration, total_duration - parameter - obj.minimum_mode_duration});

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
