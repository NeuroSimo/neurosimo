api = MTMSApi();

api.start_device();
api.start_session();

%%     Single events

%% Charge channel 0 to 20 V.

% Note that the TMS channel indexing starts from 0.
channel = 0;
target_voltage = 20;
execution_condition = api.execution_conditions.IMMEDIATE;

api.send_charge(channel, target_voltage, execution_condition);
api.wait_for_completion();

%% Send pulse on channel 0, using the default waveform.

waveform = api.get_default_waveform(channel);
reverse_polarity = false;

channel = 0;
execution_condition = api.execution_conditions.IMMEDIATE;

api.send_pulse(channel, waveform, reverse_polarity, execution_condition);
api.wait_for_completion();


%% Discharge channel 0 completely.

target_voltage = 0;

channel = 0;
execution_condition = api.execution_conditions.IMMEDIATE;

api.send_discharge(channel, target_voltage, execution_condition);
api.wait_for_completion();


%% Send trigger out on port 1.

duration_us = 1000;

port = 1;
execution_condition = api.execution_conditions.IMMEDIATE;

api.send_trigger_out(port, duration_us, execution_condition);
api.wait_for_completion();


%% Send pulse on channel 0 and analyze MEP.

% Use default waveform for the pulse.

waveform = api.get_default_waveform(channel);
reverse_polarity = false;

% Generate a timed pulse.

execution_condition = api.execution_conditions.TIMED;
time = api.get_time() + 3.0;
channel = 0;

api.send_pulse(channel, waveform, reverse_polarity, execution_condition, time);
% Do not wait for completion here, as we want to execute the pulse simultaneously with the MEP analysis.

% Analyze MEP on EMG channel 0, coinciding with the pulse.

% Note that the EMG channel indexing starts from 0.
emg_channel = 0;

mep_start_time = 0.02;  % in ms, after the stimulation pulse
mep_end_time = 0.04;  % in ms
preactivation_check_enabled = true;
preactivation_start_time = -0.02;  % in ms, minus sign indicates that the window starts before the stimulation pulse
preactivation_end_time = -0.01;
preactivation_voltage_range_limit = 70;  % Maximum allowed voltage range inside the time window, in uV.

mep_configuration = api.create_mep_configuration(emg_channel, mep_start_time, mep_end_time, preactivation_check_enabled, preactivation_start_time, preactivation_end_time, preactivation_voltage_range_limit);

[mep, errors] = api.analyze_mep(time, mep_configuration);

amplitude = mep.amplitude;
latency = mep.latency;

%% Custom waveforms

% Create a custom waveform.

custom_waveform_struct = struct( ...
    'mode', {'r', 'h', 'f'}, ...                        % 'r' for rising, 'h' for hold, 'f' for falling
    'duration', {60 * 1e-6, 30 * 1e-6, 37 * 1e-6} ...   % in seconds
);

custom_waveform = api.create_waveform(custom_waveform_struct);

% Send pulse on all channels, using the custom waveform created above.

% Note that in this example, the same waveform is used on all channels, but in reality, different waveforms
% should be used on different channels due to different coil characteristics.

waveforms_for_coil_set = api.create_waveforms_for_coil_set([custom_waveform, custom_waveform, custom_waveform, custom_waveform, custom_waveform]);

api.send_immediate_custom_pulse_to_all_channels(waveforms_for_coil_set);

%% Paired pulse targeting

algorithm = api.get_targeting_algorithm('genetic');

displacement_x = 0;  % mm
displacement_y = 0;  % mm
rotation_angle = 45;  % deg
intensity = 10;  % V/m

first_target = api.create_target(displacement_x, displacement_y, rotation_angle, intensity, algorithm);

displacement_x = 5;  % mm
displacement_y = 5;  % mm
rotation_angle = 90;  % deg
intensity = 5;  % V/m

second_target = api.create_target(displacement_x, displacement_y, rotation_angle, intensity, algorithm);

targets = [first_target, second_target];

[initial_voltages, approximated_waveforms] = api.get_multipulse_waveforms(targets)

% Charge all channels to initial voltages.
api.send_immediate_charge_or_discharge_to_all_channels(initial_voltages);

api.wait_for_completion()

% Set the times (in seconds).
time = api.get_time() + 1.0;
time_between_pulses = 0.003;

% Send the first pulse.
api.send_timed_pulse_to_all_channels(approximated_waveforms(1), time);

% Send the second pulse.
api.send_timed_pulse_to_all_channels(approximated_waveforms(2), time + time_between_pulses);

% Wait for completion of both pulses.
api.wait_for_completion()


%% Targeting

displacement_x = 5;  % mm
displacement_y = 5;  % mm
rotation_angle = 90;  % deg
intensity = 5;  % V/m
algorithm = api.get_targeting_algorithm('genetic');

target = api.create_target(displacement_x, displacement_y, rotation_angle, intensity, algorithm);

[target_voltages, reverse_polarities] = api.get_target_voltages(target);

% Get maximum intensity

maximum_intensity = api.get_maximum_intensity(displacement_x, displacement_y, rotation_angle, algorithm);

% Charge all channels to target voltages.

api.send_immediate_charge_or_discharge_to_all_channels(target_voltages);
api.wait_for_completion();

% Send default pulse to all channels.

api.send_immediate_default_pulse_to_all_channels(reverse_polarities);
api.wait_for_completion();


%% Targeting and MEP analysis

displacement_x = 5;  % mm
displacement_y = 5;  % mm
rotation_angle = 90;  % deg
intensity = 5;  % V/m
algorithm = api.get_targeting_algorithm('genetic');

target = api.create_target(displacement_x, displacement_y, rotation_angle, intensity, algorithm);

[target_voltages, reverse_polarities] = api.get_target_voltages(target);

% Charge all channels to target voltages.

api.send_immediate_charge_or_discharge_to_all_channels(target_voltages);
api.wait_for_completion();

% Send default pulse to all channels.

time = api.get_time() + 3.0;

api.send_timed_default_pulse_to_all_channels(reverse_polarities, time);
% Do not wait for completion here, as we want to execute the pulse simultaneously with the MEP analysis.

% Analyze MEP on EMG channel 0, coinciding with the pulse.

% Note that the EMG channel indexing starts from 0.
emg_channel = 0;

mep_start_time = 0.02;  % in ms, after the stimulation pulse
mep_end_time = 0.04;  % in ms
preactivation_check_enabled = true;
preactivation_start_time = -0.02;  % in ms, minus sign indicates that the window starts before the stimulation pulse
preactivation_end_time = -0.01;
preactivation_voltage_range_limit = 70;  % Maximum allowed voltage range inside the time window, in uV.

mep_configuration = api.create_mep_configuration(emg_channel, mep_start_time, mep_end_time, preactivation_check_enabled, preactivation_start_time, preactivation_end_time, preactivation_voltage_range_limit);

[mep, errors] = api.analyze_mep(time, mep_configuration);

amplitude = mep.amplitude;
latency = mep.latency;

%% Restart session

api.stop_session()
api.start_session()


%% Stop device

api.stop_device()
