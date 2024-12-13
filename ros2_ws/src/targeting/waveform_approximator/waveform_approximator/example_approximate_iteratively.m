% Example: Approximate a waveform using the micropulse algorithm.
%
% This script demonstrates how to approximate a waveform using the micropulse algorithm,
% suitable for approximating waveforms of small target voltage using a high actual voltage.

clear all

inp = input('Do you want to perform the pulses? (y/n): ', 's');
disp(' ');

if ~strcmp(inp, 'y')
    perform_pulses = false;
    disp('Pulses will not be performed.');
else
    perform_pulses = true;
    disp('Pulses will be performed. Initializing the API...');

    api = MTMSApi();

    api.start_device();
    api.start_session();
end


solutions_filename = 'solutions_five_coil_set.mat';

time_resolution = 0.01e-6;

% Create the approximator object.
approximator = WaveformApproximator(solutions_filename, time_resolution);

% Select the coil.
approximator.select_coil(1);

% Set the actual and target voltages.
actual_voltage = 1500;

% Test with different target voltages. Ensure that the relevant corner cases (e.g., 0 V, 1500 V) are covered.
target_voltages = [0, 1, 5, 10, 20, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1490, 1500];

% Create simple target waveforms
target_waveform_rising_first = struct( ...
    'mode', {'r', 'h', 'f'}, ...
    'duration', {60 * 1e-6, 30 * 1e-6, 37 * 1e-6} ...
);

target_waveform_falling_first = struct( ...
    'mode', {'f', 'h', 'r'}, ...
    'duration', {60 * 1e-6, 30 * 1e-6, 37 * 1e-6} ...
);

target_waveforms = {target_waveform_rising_first, target_waveform_falling_first};

for j = 1:length(target_waveforms)
    target_waveform = target_waveforms{j};
    disp(' ');
    disp(['Approximating target waveform with first mode: ', target_waveform(1).mode]);
    figure
    for i = 1:length(target_voltages)
        target_voltage = target_voltages(i);

        disp(' ');
        disp(['Approximating waveform with target voltage ', num2str(target_voltage), ' V.']);

        % Approximate the waveform.
        [approximated_waveform, sampling_points, success] = approximator.approximate_iteratively(actual_voltage, target_voltage, target_waveform);

        if ~success
            disp('Approximation failed.');
            continue
        end

        % For plotting, generate the state trajectory for the original and approximated waveforms.
        state_trajectory = approximator.generate_state_trajectory_from_waveform(target_voltage, target_waveform);
        approximated_state_trajectory = approximator.generate_state_trajectory_from_waveform(actual_voltage, approximated_waveform);

        % Plot the state trajectories.
        subplot(5, 5, i)

        approximator.plot_state_trajectories(state_trajectory, approximated_state_trajectory, sampling_points)

        % Add title with the target voltagge.
        title('Target voltage: ' + string(target_voltage) + ' V')

        if perform_pulses

            % Charge channel 0 to the target voltage.
            channel = 0;
            execution_condition = api.execution_conditions.IMMEDIATE;

            disp(['Charging channel ', num2str(channel), ' to ', num2str(actual_voltage), ' V.']);

            api.send_charge(channel, actual_voltage, execution_condition);
            api.wait_for_completion();

            voltage_before_pulse = api.get_current_voltage(channel);

            disp('Press any key to perform the pulse.')
            pause;

            % Perform the pulse.
            waveform = api.create_waveform(approximated_waveform);
            reverse_polarity = false;

            api.send_pulse(channel, waveform, reverse_polarity, api.execution_conditions.IMMEDIATE);
            api.wait_for_completion();

            % Print the voltage after the pulse.
            voltage_after_pulse = api.get_current_voltage(channel);
            disp(' ');
            disp(['Actual voltage before pulse: ', num2str(voltage_before_pulse), ' V.']);
            disp(['Actual voltage after pulse: ', num2str(voltage_after_pulse), ' V.']);

            estimated_voltage = approximator.estimate_voltage_after_pulse(actual_voltage, approximated_waveform);
            disp(['Estimated voltage after pulse: ', num2str(estimated_voltage), ' V.']);

            disp(' ');
            disp('Press any key to continue.')
            pause;
        end
    end
end

if perform_pulses
    api.stop_session();
    api.stop_device();
end
disp('Done.');
