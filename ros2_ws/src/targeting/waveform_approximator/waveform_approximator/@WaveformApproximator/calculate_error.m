function error = calculate_error(obj, param, algorithm, initial_state, target_state, total_duration, mode_info)
    waveform = algorithm(param, total_duration, mode_info);
    final_state = obj.apply_waveform_to_state(initial_state, waveform);
    error = abs(final_state.I_coil - target_state.I_coil);
end
