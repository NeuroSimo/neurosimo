% Generate the state trajectory of the current based on the given function.
function state_trajectory = generate_state_trajectory_from_function(obj, coil_current_function, duration)

    state = obj.generate_initial_state(0);

    state_trajectory_length = fix(duration / obj.time_resolution) + 1;
    state_trajectory = repmat(state, 1, state_trajectory_length);

    time = 0;
    for i = 1:length(state_trajectory)
        state_trajectory(i) = state;
        state_trajectory(i).I_coil = coil_current_function(time);

        time = time + obj.time_resolution;
    end
end
