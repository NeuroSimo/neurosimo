function state_trajectory = generate_state_trajectory_from_mode(obj, initial_state, mode, duration, varargin)
    if nargin == 5
        return_only_final_state = varargin{1};
    elseif nargin > 5
        error('Too many input arguments.');
    else
        return_only_final_state = false;
    end

    % Preallocate coil current vector
    rounding_factor = fix(abs(log10(abs(1e-6/obj.time_resolution))));
    duration = round(duration/1e-6, rounding_factor)*1e-6;
    num_elements = round(duration/obj.time_resolution + 1);

    V_c = zeros(1, num_elements);       V_c(1) = initial_state.V_c;
    V_s1 = zeros(1, num_elements);      V_s1(1) = initial_state.V_s1;
    V_s2 = zeros(1, num_elements);      V_s2(1) = initial_state.V_s2;
    V_s3 = zeros(1, num_elements);      V_s3(1) = initial_state.V_s3;
    V_s4 = zeros(1, num_elements);      V_s4(1) = initial_state.V_s4;
    I_coil = zeros(1, num_elements);    I_coil(1) = initial_state.I_coil;
    I_tw = zeros(1, num_elements);      I_tw(1) = initial_state.I_tw;
    I_bar_U = zeros(1, num_elements);   I_bar_U(1) = initial_state.I_bar_U;
    I_bar_L = zeros(1, num_elements);   I_bar_L(1) = initial_state.I_bar_L;
    I_z_L = zeros(1, num_elements);     I_z_L(1) = initial_state.I_z_L;
    I_z_R = zeros(1, num_elements);     I_z_R(1) = initial_state.I_z_R;

    % Maximum diode conduction duration: 5e-6
    cond_max = 5e-6;
    I_d = zeros(1, round(cond_max/obj.time_resolution));

    % Calculate rising step
    idx = 1;
    if mode == 'f'
        % rise, diode conduction
        conduction_dur = cond_max;
        t = obj.time_resolution:obj.time_resolution:conduction_dur;

        I_d = obj.functions.i_rise_diode_simp(initial_state.I_bar_L, initial_state.I_bar_U, initial_state.I_coil, ...
                                              initial_state.I_tw, initial_state.I_z_L, initial_state.I_z_R, ...
                                              initial_state.V_c, initial_state.V_s1, initial_state.V_s2, ...
                                              initial_state.V_s3, initial_state.V_s4, t);
        rev_index = find(I_d < 0, 1);
        conduction_dur = obj.time_resolution*(rev_index-1);

        if conduction_dur ~= 0
            t = obj.time_resolution:obj.time_resolution:conduction_dur;
            size = length(t);
            V_c(idx+1:idx+size)     = obj.functions.v_rise_cap_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            V_s1(idx+1:idx+size)    = obj.functions.v_rise_sn1_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            V_s2(idx+1:idx+size)    = obj.functions.v_rise_sn2_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            V_s3(idx+1:idx+size)    = obj.functions.v_rise_sn3_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            V_s4(idx+1:idx+size)    = obj.functions.v_rise_sn4_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

            I_coil(idx+1:idx+size)  = obj.functions.i_rise_coil_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            I_tw(idx+1:idx+size)    = obj.functions.i_rise_tw_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            I_bar_U(idx+1:idx+size) = obj.functions.i_rise_barU_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            I_bar_L(idx+1:idx+size) = obj.functions.i_rise_barL_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            I_z_L(idx+1:idx+size)   = obj.functions.i_rise_zL_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
            I_z_R(idx+1:idx+size)   = obj.functions.i_rise_zR_d_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

            idx = idx+size;
        end

        % rise
        t = obj.time_resolution:obj.time_resolution:duration-conduction_dur;
        size = length(t);

        V_c(idx+1:idx+size)     = obj.functions.v_rise_cap_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s1(idx+1:idx+size)    = obj.functions.v_rise_sn1_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s2(idx+1:idx+size)    = obj.functions.v_rise_sn2_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s3(idx+1:idx+size)    = obj.functions.v_rise_sn3_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s4(idx+1:idx+size)    = obj.functions.v_rise_sn4_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

        I_coil(idx+1:idx+size)  = obj.functions.i_rise_coil_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_tw(idx+1:idx+size)    = obj.functions.i_rise_tw_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_U(idx+1:idx+size) = obj.functions.i_rise_barU_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_L(idx+1:idx+size) = obj.functions.i_rise_barL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_L(idx+1:idx+size)   = obj.functions.i_rise_zL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_R(idx+1:idx+size)   = obj.functions.i_rise_zR_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

    elseif mode == 'h'
        % top hold
        t = obj.time_resolution:obj.time_resolution:duration;
        size = length(t);
        V_c(idx+1:idx+size)     = obj.functions.v_top_cap_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s1(idx+1:idx+size)    = obj.functions.v_top_sn1_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s2(idx+1:idx+size)    = obj.functions.v_top_sn2_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s3(idx+1:idx+size)    = obj.functions.v_top_sn3_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s4(idx+1:idx+size)    = obj.functions.v_top_sn4_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

        I_coil(idx+1:idx+size)  = obj.functions.i_top_coil_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_tw(idx+1:idx+size)    = obj.functions.i_top_tw_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_U(idx+1:idx+size) = obj.functions.i_top_barU_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_L(idx+1:idx+size) = obj.functions.i_top_barL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_L(idx+1:idx+size)   = obj.functions.i_top_zL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_R(idx+1:idx+size)   = obj.functions.i_top_zR_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

    elseif mode == 'a'
        % bottom hold
        t = obj.time_resolution:obj.time_resolution:duration;
        size = length(t);
        V_c(idx+1:idx+size)     = obj.functions.v_bot_cap_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s1(idx+1:idx+size)    = obj.functions.v_bot_sn1_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s2(idx+1:idx+size)    = obj.functions.v_bot_sn2_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s3(idx+1:idx+size)    = obj.functions.v_bot_sn3_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s4(idx+1:idx+size)    = obj.functions.v_bot_sn4_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

        I_coil(idx+1:idx+size)  = obj.functions.i_bot_coil_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_tw(idx+1:idx+size)    = obj.functions.i_bot_tw_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_U(idx+1:idx+size) = obj.functions.i_bot_barU_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_L(idx+1:idx+size) = obj.functions.i_bot_barL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_L(idx+1:idx+size)   = obj.functions.i_bot_zL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_R(idx+1:idx+size)   = obj.functions.i_bot_zR_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

    elseif mode == 'r'
        % fall
        t = obj.time_resolution:obj.time_resolution:duration;
        size = length(t);
        V_c(idx+1:idx+size)     = obj.functions.v_fall_cap_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s1(idx+1:idx+size)    = obj.functions.v_fall_sn1_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s2(idx+1:idx+size)    = obj.functions.v_fall_sn2_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s3(idx+1:idx+size)    = obj.functions.v_fall_sn3_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        V_s4(idx+1:idx+size)    = obj.functions.v_fall_sn4_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);

        I_coil(idx+1:idx+size)  = obj.functions.i_fall_coil_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_tw(idx+1:idx+size)    = obj.functions.i_fall_tw_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_U(idx+1:idx+size) = obj.functions.i_fall_barU_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_bar_L(idx+1:idx+size) = obj.functions.i_fall_barL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_L(idx+1:idx+size)   = obj.functions.i_fall_zL_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
        I_z_R(idx+1:idx+size)   = obj.functions.i_fall_zR_simp(I_bar_L(idx), I_bar_U(idx), I_coil(idx), I_tw(idx), I_z_L(idx), I_z_R(idx), V_c(idx), V_s1(idx), V_s2(idx), V_s3(idx), V_s4(idx), t);
    end

    if return_only_final_state
        state_trajectory.V_c = V_c(end);
        state_trajectory.V_s1 = V_s1(end);
        state_trajectory.V_s2 = V_s2(end);
        state_trajectory.V_s3 = V_s3(end);
        state_trajectory.V_s4 = V_s4(end);
        state_trajectory.I_coil = I_coil(end);
        state_trajectory.I_tw = I_tw(end);
        state_trajectory.I_bar_U = I_bar_U(end);
        state_trajectory.I_bar_L = I_bar_L(end);
        state_trajectory.I_z_L = I_z_L(end);
        state_trajectory.I_z_R = I_z_R(end);
    else
        state_trajectory = repmat(initial_state, 1, length(V_c));
        for i = 1:length(V_c)
            state_trajectory(i).V_c = V_c(i);
            state_trajectory(i).V_s1 = V_s1(i);
            state_trajectory(i).V_s2 = V_s2(i);
            state_trajectory(i).V_s3 = V_s3(i);
            state_trajectory(i).V_s4 = V_s4(i);
            state_trajectory(i).I_coil = I_coil(i);
            state_trajectory(i).I_tw = I_tw(i);
            state_trajectory(i).I_bar_U = I_bar_U(i);
            state_trajectory(i).I_bar_L = I_bar_L(i);
            state_trajectory(i).I_z_L = I_z_L(i);
            state_trajectory(i).I_z_R = I_z_R(i);
        end
    end
end
