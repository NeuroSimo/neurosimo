function waveform = algorithm_constant_hold(obj, parameter, total_duration, mode_info)
    current_mode = mode_info.current_mode;
    is_last_segment = mode_info.is_last_segment;
    segment_index = mode_info.segment_index;
    mode_index = mode_info.mode_index;

    waveform = struct( ...
        'mode', {'h'}, ...
        'duration', {total_duration});
end
