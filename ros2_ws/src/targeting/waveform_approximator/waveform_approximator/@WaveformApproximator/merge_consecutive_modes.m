function waveform = merge_consecutive_modes(obj, waveform)
    % Go through the waveform; if there are consecutive modes with the same mode (e.g., 'h'),
    % merge them.
    i = 1;
    while i < length(waveform)
        if waveform(i).mode == waveform(i + 1).mode
            waveform(i).duration = waveform(i).duration + waveform(i + 1).duration;
            waveform(i + 1) = [];
        else
            i = i + 1;
        end
    end
end
