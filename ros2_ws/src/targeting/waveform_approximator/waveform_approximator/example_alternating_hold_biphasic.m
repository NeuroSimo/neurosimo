% Example: Approximate a waveform using the 'alternating hold' algorithm.
%
% This script demonstrates how to approximate a waveform using the alternating hold algorithm.

clear all

solutions_filename = 'solutions_five_coil_set.mat';

time_resolution = 0.01e-6;

% Create the approximator object.
approximator = WaveformApproximator(solutions_filename, time_resolution);

% Select the coil.
approximator.select_coil(1);

% Select the algorithm.
algorithm = @approximator.algorithm_alternating_hold_start_with_hold;

% Set the actual and target voltages.
actual_voltage = 1500;

% Note: If target voltage is very low, the approximation might contain modes with too short a duration;
%       checking the durations is the responsibility of the user.
target_voltage = 1000;

% Create a simple target waveform.
target_waveform = struct( ...
    'mode', {'f', 'h', 'r', 'h', 'f', 'h'}, ...
    'duration', {60e-6, 30e-6, 120e-6, 30e-6, 45e-6, 5e-6});

num_of_intermediate_points_per_mode = [2, 0, 4, 0, 1, 0];

% Generate the state trajectory for the target waveform.
state_trajectory = approximator.generate_state_trajectory_from_waveform(target_voltage, target_waveform);

% Sample the state trajectory using the waveform.
sampling_points = approximator.sample_state_trajectory_by_waveform(state_trajectory, target_waveform, num_of_intermediate_points_per_mode);

% Approximate the waveform.
[approximated_waveform, relative_errors] = approximator.approximate(actual_voltage, sampling_points, algorithm);

% Note that it is the responsibility of the user to check that the durations of the modes are not too short
% and that the relative errors are acceptable (e.g., smaller than 0.02). For more automated checking,
% see example_approximate_iteratively.m.

% Generate the state trajectory for the approximated waveform.
approximated_state_trajectory = approximator.generate_state_trajectory_from_waveform(actual_voltage, approximated_waveform);

% Plot the state trajectories.
figure
approximator.plot_state_trajectories(state_trajectory, approximated_state_trajectory, sampling_points)


% Optionally, perform the pulse.
inp = input('Do you want to perform the pulse? (y/n): ', 's');
if ~strcmp(inp, 'y')
    return
end

api = MTMSApi();

api.start_device();
api.start_session();

waveform = api.create_waveform(approximated_waveform);
reverse_polarity = false;

channel = 0;
execution_condition = api.execution_conditions.IMMEDIATE;

api.send_pulse(channel, waveform, reverse_polarity, execution_condition);
api.wait_for_completion();
