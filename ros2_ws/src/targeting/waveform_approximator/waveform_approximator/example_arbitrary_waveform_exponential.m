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


% Generate a function that consists of these parts:
%   - Exponential rise to a maximum of 3000 A (150 us)
%   - Linear fall into -3000 A (120 us)
%   - Constant current at -3000 A (30 us)
%   - Linear rise into 0 A (50 us)
%
% The function needs to be continuous; each part starts from the end of the previous part.

% Parameters for the exponential rise.
exponential_duration = 150 * 1e-6;

I_max = 3000;
tau = 200e-6;

% Parameters for the linear fall.
fall_duration = 120 * 1e-6;

I_after_fall = -3000;

% Parameters for the constant phase.
constant_duration = 30 * 1e-6;

% Parameters for the linear rise.
rise_duration = 50 * 1e-6;

I_after_rise = 0;

% Define the function segments.
exponential_segment = @(t) I_max * (1 - exp(-t / tau));
fall_segment = @(t) exponential_segment(exponential_duration) - (exponential_segment(exponential_duration) - I_after_fall) * t / fall_duration;
constant_segment = @(t) fall_segment(fall_duration);
rise_segment = @(t) constant_segment(constant_duration) + (I_after_rise - constant_segment(constant_duration)) * t / rise_duration;

% Define the function.
I = @(t) ...
    (t <= exponential_duration) .* exponential_segment(t) + ...
    (t > exponential_duration & t <= exponential_duration + fall_duration) .* fall_segment(t - exponential_duration) + ...
    (t > exponential_duration + fall_duration & t <= exponential_duration + fall_duration + constant_duration) .* constant_segment(t - exponential_duration - fall_duration) + ...
    (t > exponential_duration + fall_duration + constant_duration & t <= exponential_duration + fall_duration + constant_duration + rise_duration) .* rise_segment(t - exponential_duration - fall_duration - constant_duration);


% Generate the state trajectory based on the coil current function.
total_duration = exponential_duration + fall_duration + constant_duration + rise_duration;
num_of_intermediate_points = 10;

state_trajectory = approximator.generate_state_trajectory_from_function(I, total_duration);

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
