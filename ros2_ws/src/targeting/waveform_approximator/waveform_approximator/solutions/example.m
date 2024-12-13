%% Example: Find solutions and store them in a .mat file.

% Load pulse data
load('1_10_5ch_coil_1.mat');

% Find system resistance and inductance during rising phase
[R_fit, L_fit] = characterize_coil(pulse_data, 'true', 'true');

% Generate small system parameter matrix
system_matrix = generate_matrix(R_fit, L_fit, 'true', 'true');

% Calculate circuit solutions
sols = calculate_solutions(system_matrix);

% Save solutions file
save('solutions.mat', 'sols');
