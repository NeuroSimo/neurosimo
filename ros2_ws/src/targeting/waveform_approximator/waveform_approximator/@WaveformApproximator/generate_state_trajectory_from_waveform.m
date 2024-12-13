% Calculate the resulting state_trajectory of the circuit with the given waveform and initial voltage.
function state_trajectory = generate_state_trajectory_from_waveform(obj, voltage, waveform)
    state = obj.generate_initial_state(voltage);

    total_duration = sum([waveform.duration]);

    state_trajectory_length = floor(total_duration / obj.time_resolution) + 1;
    state_trajectory = repmat(state, 1, state_trajectory_length);

    j = 1;
    for i = 1:length(waveform)
        mode = waveform(i).mode;
        duration = waveform(i).duration;

        state_trajectory_for_mode = obj.generate_state_trajectory_from_mode(state, mode, duration);

        state_trajectory(j:j+length(state_trajectory_for_mode)-1) = state_trajectory_for_mode;
        j = j + length(state_trajectory_for_mode) - 1;

        state = state_trajectory_for_mode(end);
    end
end
