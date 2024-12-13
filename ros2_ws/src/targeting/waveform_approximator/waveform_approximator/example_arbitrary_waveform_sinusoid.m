% Example: Approximate a sinusoidal coil current function using the hold algorithm.

clear all

solutions_filename = 'solutions_five_coil_set.mat';

time_resolution = 0.01e-6;

% Create the approximator object.
approximator = WaveformApproximator(solutions_filename, time_resolution);

% Select the coil.
approximator.select_coil(1);

% Select the algorithm.
algorithm = @approximator.algorithm_hold_both_ends;

% Set the actual and target voltages.
actual_voltage = 1500;

% Note: If target voltage is very low, the approximation might contain modes with too short a duration;
%       checking the durations is the responsibility of the user.
target_voltage = 1000;


% Generate a sinusoidal coil current function.
duration = 100 * 1e-6;
omega = 2 * pi / duration;
I_max = 1000;
coil_current_function = @(t) I_max * sin(omega * t);
num_of_intermediate_points = 10;

% Generate the state trajectory based on the coil current function.
state_trajectory = approximator.generate_state_trajectory_from_function(coil_current_function, duration);

% Sample the state trajectory uniformly.
sampling_points = approximator.sample_state_trajectory_uniformly(state_trajectory, num_of_intermediate_points);

% Approximate the waveform.
[approximated_waveform, relative_errors] = approximator.approximate(actual_voltage, sampling_points, algorithm);

% Note that it is the responsibility of the user to check that the durations of the modes are not too short
% and that the relative errors are acceptable (e.g., smaller than 0.02). For more automated checking,
% see example_approximate_iteratively.m.

% Generate the state trajectory for the approximated waveform
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
