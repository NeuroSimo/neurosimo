function end_time = get_state_trajectory_end_time(obj, state_trajectory)
    end_time = (length(state_trajectory) - 1) * obj.time_resolution;
end