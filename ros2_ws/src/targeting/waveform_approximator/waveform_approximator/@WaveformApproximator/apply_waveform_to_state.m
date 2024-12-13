function final_state = apply_waveform_to_state(obj, initial_state, waveform)
    intermediate_state = initial_state;

    for i = 1:length(waveform)
        duration = waveform(i).duration;
        mode = waveform(i).mode;
        intermediate_state = obj.apply_mode_to_state(intermediate_state, mode, duration);
    end

    final_state = intermediate_state;
end
