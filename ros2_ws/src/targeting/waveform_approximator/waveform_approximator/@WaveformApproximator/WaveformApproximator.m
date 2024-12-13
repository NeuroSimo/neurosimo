classdef WaveformApproximator < handle

    properties (SetAccess = public, GetAccess = public)
        solutions;
        functions;
        metadata;

        time_resolution;
        minimum_mode_duration;
    end

    methods (Access = public)

        % Constructor for the waveform approximator.
        % input:
        %   @solutions_filename: filename of the file containing phase functions
        %   @time_resolution: time resolution for computing the state trajectories
        function obj = WaveformApproximator(solutions_file, time_resolution)
            obj.time_resolution = time_resolution;
            obj.solutions = load(solutions_file);

            % The minimum mode duration on the FPGA (as of bitfile version 0.5.12) is 160 ticks; however, this is
            % incorrect and should be fixed, as feedback is requested 200 ticks after mode change because
            % channel interference causes artifacts in the feedback signal for as long as 160 us after mode change.
            % The correct minimum mode duration is therefore 200 ticks, corresponding to 5 us
            % (1 tick is 25 ns). Use the correct value here. TODO: Fix on the FPGA side.
            obj.minimum_mode_duration = 200 * 25e-9;
        end

        function select_coil(obj, index)
            assert (index > 0, "Index must be greater than 0");
            assert (index <= length(obj.solutions.sols), "Index must be less than or equal to the number of solutions");

            obj.functions = obj.solutions.sols(index).f;
            obj.metadata = obj.solutions.sols(index).metadata;
        end

        % Generate a state trajectory from an initial voltage and a waveform.
        %
        % input:
        %   @voltage: initial voltage
        %   @waveform: an array of structs, each with the following fields:
        %       @mode: the mode of the waveform phase as a character, e.g., 'f' for falling
        %       @duration: the mode duration in seconds
        % output:
        %   @state_trajectory: an array of states, each state a struct, including the fields V_c, V_s1, V_s2, ..., I_z_R.
        state_trajectory = generate_state_trajectory_from_waveform(obj, voltage, waveform)

        % Generate a state trajectory from an arbitrary function and a duration.
        %
        % input:
        %   @coil_current_function: function that returns the coil current at a given time in seconds
        %   @duration: duration for the state trajectory
        % output:
        %   @state_trajectory: an array of states, each state a struct, including the fields V_c, V_s1, V_s2, ..., I_z_R.
        state_trajectory = generate_state_trajectory_from_function(obj, coil_current_function, duration);

        % Generate sampling points from a state trajectory and a target waveform.
        %
        % input:
        %   @state_trajectory: an array of states, each state a struct, including the fields V_c, V_s1, V_s2, ..., I_z_R.
        %   @target_waveform: an array of structs, each with the following fields:
        %       @mode: the mode of the waveform phase as a character, e.g., 'f' for falling
        %       @duration: the mode duration in seconds
        %   @num_of_intermediate_points_per_mode: the number of intermediate points between two consecutive modes
        % output:
        %   @sampling_points: an array of structs, each with the following fields:
        %       @time: the time in seconds
        %       @index: the index of the state in the state trajectory
        %       @state: the state at the index
        %       @mode_info: a struct with the following fields:
        %           @current_mode: the mode of the next phase as a character, e.g., 'f' for falling
        %           @is_last_segment: a boolean indicating if the mode is the last one
        %           @index: the index of the mode in the target waveform
        sampling_points = sample_state_trajectory_by_waveform(obj, state_trajectory, target_waveform, num_of_intermediate_points_per_mode)

        % Generate sampling points from a state trajectory and a number of intermediate points.
        %
        % input:
        %   @state_trajectory: an array of states, each state a struct, including the fields V_c, V_s1, V_s2, ..., I_z_R.
        %   @num_of_sampling_points: the number of intermediate points between two consecutive modes
        % output:
        %   @sampling_points: an array of structs, each with the following fields:
        %       @time: the time in seconds
        %       @index: the index of the state in the state trajectory
        %       @state: the state at the index
        %       @mode_info: a struct with the following fields:
        %           @current_mode: the mode of the next phase as a character, e.g., 'f' for falling
        %           @is_last_segment: a boolean indicating if the mode is the last one
        %           @index: the index of the mode in the target waveform
        sampling_points = sample_state_trajectory_uniformly(obj, state_trajectory, num_of_sampling_points)

        % Generate sampling points from a state trajectory and an array of sampling times.
        %
        % input:
        %   @state_trajectory: an array of states, each state a struct, including the fields V_c, V_s1, V_s2, ..., I_z_R.
        %   @sampling_times: an array of times in seconds
        % output:
        %   @sampling_points: an array of structs, each with the following fields:
        %       @time: the time in seconds
        %       @index: the index of the state in the state trajectory
        %       @state: the state at the index
        %       @mode_info: a struct with the following fields:
        %           @current_mode: the mode of the next phase as a character, e.g., 'f' for falling
        %           @is_last_segment: a boolean indicating if the mode is the last one
        %           @index: the index of the mode in the target waveform
        sampling_points = sample_state_trajectory(obj, state_trajectory, sampling_times)

        % Approximate a waveform using a given algorithm. Note that it is the responsibility
        % of the user to check that the approximated waveform phases have a long enough duration and that
        % the relative errors are small enough.
        %
        % input:
        %   @actual_voltage: initial voltage
        %   @sampling_points: an array of structs, generated by the function sample_state_trajectory_uniformly or
        %      sample_state_trajectory_by_waveform
        %   @approximator: an object of the class WaveformApproximator
        %   @algorithm: a function handle to the algorithm for approximating the waveform
        % output:
        %   @approximated_waveform: an array of structs, each with the following fields:
        %       @mode: the mode of the waveform phase as a character, e.g., 'f' for falling
        %       @duration: the mode duration in seconds
        %   @relative_errors: an array of relative errors at each sampling point
        [approximated_waveform, relative_errors] = approximate(obj, actual_voltage, sampling_points, approximator, algorithm)

        [approximated_waveform, sampling_points, success] = approximate_iteratively(obj, actual_voltage, target_voltage, target_waveform)

        % Estimate the final voltage after a waveform is applied.
        %
        % input:
        %   @voltage_before: initial voltage before the pulse
        %   @waveform: an array of structs, each with the following fields:
        %       @mode: the mode of the waveform phase as a character, e.g., 'f' for falling
        %       @duration: the mode duration in seconds
        % output:
        %   @voltage_after: the final voltage after the waveform is applied
        voltage_after = estimate_voltage_after_pulse(obj, voltage_before, waveform);

        plot_state_trajectories(obj, target_state_trajectory, approximated_state_trajectory, sampling_points)

        % Algorithms for approximating waveforms
        %
        % Usage, e.g.:
        %   algorithm = @approximator.algorithm_hold;
        %   approximated_waveform = approximator.approximate(actual_voltage, sampling_points, approximator, algorithm);

        % Approximate each sampling interval with a hold and either a falling or a rising mode.
        waveform = algorithm_hold(obj, parameter, total_duration, mode_info)

        % Approximate each sampling interval with a hold mode at the beginning and the end, and a falling or a rising
        % mode in between. The hold modes are of the same duration.
        waveform = algorithm_hold_both_ends(obj, parameter, total_duration, mode_info)

        % Approximate each sampling interval with a hold and either a falling or a rising mode. The order of the hold and
        % the falling/rising mode is alternating.
        waveform = algorithm_alternating_hold_start_with_hold(obj, parameter, total_duration, mode_info)
        waveform = algorithm_alternating_hold_start_with_non_hold(obj, parameter, total_duration, mode_info)

        % Approximate each sampling interval with a hold, a falling mode, and a rising mode. The falling mode precedes
        % the rising mode if the target current is increasing, and vice versa.
        waveform = algorithm_micropulse(obj, parameter, total_duration, mode_info)

        % Approximate each sampling interval with only a hold mode. This is a dummy algorithm, only useful when
        % approximating a waveform with very low target voltage.
        waveform = algorithm_constant_hold(obj, parameter, total_duration, mode_info)
    end

    methods (Access = private)
        initial_state = generate_initial_state(obj, voltage);

        final_state = apply_mode_to_state(obj, initial_state, mode, duration)
        final_state = apply_waveform_to_state(obj, initial_state, waveform)

        state_trajectory = generate_state_trajectory_from_mode(obj, initial_state, mode, duration, varargin)

        [parameter, error] = golden_section_search(obj, error_function, lower_bound, upper_bound)

        error = calculate_error(obj, parameter, algorithm, initial_state, target_state, total_duration, mode_info)

        index = get_index_from_time(obj, time)
        time = get_state_trajectory_end_time(obj, state_trajectory)

        waveform = merge_consecutive_modes(obj, waveform)
    end
end
