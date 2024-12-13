function final_state = apply_mode_to_state(obj, initial_state, mode, duration)

    % Give 'true' as the last argument so that it only returns the final state; this speeds up the
    % function markedly, as it does not have to form the structure array of states.
    final_state = obj.generate_state_trajectory_from_mode(initial_state, mode, duration, true);
end
