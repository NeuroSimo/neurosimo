function sampling_points = sample_state_trajectory_uniformly(obj, state_trajectory, num_of_intermediate_points)
    end_time = (length(state_trajectory) - 1) * obj.time_resolution;
    sampling_times = linspace(0, end_time, 2 + num_of_intermediate_points);

    sampling_points = obj.sample_state_trajectory(state_trajectory, sampling_times);
end
