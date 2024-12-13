function sols = solve_phase_functions(C_pulse, R_coil, L_coil, R_sn, C_sn, R_tw, L_tw, R_bar, L_bar, R_z, L_z, R_ch, R_d)
    syms s
    syms V_cap V_sn1 V_sn2 V_sn3 V_sn4 I_coil I_tw I_barU I_barL I_zL I_zR real
    V_d = 2;        % Diode forward voltage
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Rising phase solutions
    %%%%%%%%%%%%%%%%%%%%%%%%
    % Define mesh impedances
    Z_rise = [  1/(s*C_pulse)+s*L_tw+R_tw+R_ch+R_z+s*L_z+1/(s*C_sn)+R_sn,  -R_ch,  0,  -R_z-s*L_z-1/(s*C_sn)-R_sn,   0;
                -R_ch,   R_ch+R_sn+1/(s*C_sn),   -R_sn-1/(s*C_sn),  0,  0;
                0,   -R_sn-1/(s*C_sn),  2*R_sn+2/(s*C_sn)+s*L_bar+R_bar+s*L_coil+R_coil,  -s*L_coil-R_coil,   0;
                -R_sn-1/(s*C_sn)-R_z-s*L_z,    0,  -s*L_coil-R_coil,   R_sn+1/(s*C_sn)+2*s*L_z+2*R_z+R_ch+R_coil+s*L_coil+R_bar+s*L_bar, -R_ch;
                0,   0,  0,  -R_ch,  R_ch+R_sn+1/(s*C_sn) ];

	% Define mesh voltages
    V_rise = [  V_cap/s-V_sn2/s+L_tw*I_tw+L_z*I_zL;
                -V_sn1/s;
                V_sn1/s-V_sn3/s-L_coil*I_coil+L_bar*I_barU;
                V_sn2/s+L_coil*I_coil-L_z*I_zL+L_z*I_zR+L_bar*I_barL;
                -V_sn4/s ];
    
	% Solve for unknown (laplace domain) currents
	I_rise = linsolve(Z_rise, V_rise);
    
    % Solve for reactive element currents and voltages (in time domain) using inverse laplace transform and
    % variable precision arithmetic - turn into a function handle for speed
    sols.i_rise_coil_simp       = matlabFunction(ilaplace(vpa(I_rise(4)-I_rise(3))));
    sols.i_rise_tw_simp         = matlabFunction(ilaplace(vpa(I_rise(1))));
    sols.i_rise_barU_simp       = matlabFunction(ilaplace(vpa(I_rise(3))));
    sols.i_rise_barL_simp       = matlabFunction(ilaplace(vpa(I_rise(4))));
    sols.i_rise_zL_simp         = matlabFunction(ilaplace(vpa(I_rise(1)-I_rise(4))));
    sols.i_rise_zR_simp         = matlabFunction(ilaplace(vpa(I_rise(4))));
    
    sols.v_rise_cap_simp        = matlabFunction(ilaplace(vpa(-I_rise(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_rise_sn1_simp        = matlabFunction(ilaplace(vpa((I_rise(2)-I_rise(3))*(1/(s*C_sn)) + V_sn1/s)));
    sols.v_rise_sn2_simp        = matlabFunction(ilaplace(vpa((I_rise(1)-I_rise(4))*(1/(s*C_sn)) + V_sn2/s)));
    sols.v_rise_sn3_simp        = matlabFunction(ilaplace(vpa(I_rise(3)*1/(s*C_sn) + V_sn3/s)));
    sols.v_rise_sn4_simp        = matlabFunction(ilaplace(vpa(I_rise(5)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Rising phase calculations done.\n");
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Rising phase (diode conduction) solutions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define mesh impedances
    Z_rise_d = [    1/(s*C_pulse)+s*L_tw+R_tw+R_ch+R_z+s*L_z+1/(s*C_sn)+R_sn,  -R_ch,  0,  -R_z-s*L_z-1/(s*C_sn)-R_sn,   0,  0;
                    -R_ch,   R_ch+R_sn+1/(s*C_sn),   -R_sn-1/(s*C_sn),  0,  0,   0;
                    0,   -R_sn-1/(s*C_sn),  2*R_sn+2/(s*C_sn)+s*L_bar+R_bar+s*L_coil+R_coil,  -s*L_coil-R_coil,   0,     -1/(s*C_sn)-R_sn;
                    -R_sn-1/(s*C_sn)-R_z-s*L_z,    0,  -s*L_coil-R_coil,   R_sn+1/(s*C_sn)+2*s*L_z+2*R_z+R_ch+R_coil+s*L_coil+R_bar+s*L_bar, -R_ch,  0;
                    0,   0,  0,  -R_ch,  R_ch+R_sn+1/(s*C_sn),   0;
                    0,   0,  -R_sn-1/(s*C_sn),   0,  0,  R_sn+1/(s*C_sn)+R_d ];
    
    % Define mesh voltages
    V_rise_d = [    V_cap/s-V_sn2/s+L_tw*I_tw+L_z*I_zL;
                    -V_sn1/s;
                    V_sn1/s-V_sn3/s-L_coil*I_coil+L_bar*I_barU;
                    V_sn2/s+L_coil*I_coil-L_z*I_zL+L_z*I_zR+L_bar*I_barL;
                    -V_sn4/s;
                    V_sn3/s+V_d/s ]; % remove: +V_d/s
	
    % Solve for laplace domain currents
	I_rise_d = linsolve(Z_rise_d, V_rise_d);
    
    % Solve for reactive element currents and voltages
    sols.i_rise_coil_d_simp     = matlabFunction(ilaplace(vpa(I_rise_d(4)-I_rise_d(3))));
    sols.i_rise_tw_d_simp       = matlabFunction(ilaplace(vpa(I_rise_d(1))));
    sols.i_rise_barU_d_simp     = matlabFunction(ilaplace(vpa(I_rise_d(3))));
    sols.i_rise_barL_d_simp     = matlabFunction(ilaplace(vpa(I_rise_d(4))));
    sols.i_rise_zL_d_simp       = matlabFunction(ilaplace(vpa(I_rise_d(1)-I_rise_d(4))));
    sols.i_rise_zR_d_simp       = matlabFunction(ilaplace(vpa(I_rise_d(4))));
    sols.i_rise_diode_simp      = matlabFunction(ilaplace(vpa(-I_rise_d(6))));
    
    sols.v_rise_cap_d_simp      = matlabFunction(ilaplace(vpa(-I_rise_d(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_rise_sn1_d_simp      = matlabFunction(ilaplace(vpa((I_rise_d(2)-I_rise_d(3))*(1/(s*C_sn)) + V_sn1/s)));
    sols.v_rise_sn2_d_simp      = matlabFunction(ilaplace(vpa((I_rise_d(1)-I_rise_d(4))*(1/(s*C_sn)) + V_sn2/s)));
    sols.v_rise_sn3_d_simp      = matlabFunction(ilaplace(vpa((I_rise_d(3)-I_rise_d(6))*1/(s*C_sn) + V_sn3/s)));
    sols.v_rise_sn4_d_simp      = matlabFunction(ilaplace(vpa(I_rise_d(5)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Rising phase (dcon) calculations done.\n");
    
    %%%%%%%%%%%%%%%%%%%%
    % Top loop solutions
    %%%%%%%%%%%%%%%%%%%%
    % Define mesh impedances
    Z_top = [   1/(s*C_pulse)+s*L_tw+R_tw+R_ch+R_z+s*L_z+1/(s*C_sn)+R_sn,  -R_ch,  0,  -R_z-s*L_z-1/(s*C_sn)-R_sn,   0;
                -R_ch,   R_ch+R_sn+1/(s*C_sn),   -R_sn-1/(s*C_sn),  0,  0;
                0,   -R_sn-1/(s*C_sn),  R_sn+1/(s*C_sn)+s*L_bar+R_bar+R_d+s*L_coil+R_coil,  -s*L_coil-R_coil,   -R_d;
                -R_sn-1/(s*C_sn)-R_z-s*L_z,    0,  -s*L_coil-R_coil,   2*R_sn+2/(s*C_sn)+2*s*L_z+2*R_z+R_coil+s*L_coil+R_bar+s*L_bar, 0;
                0,   0,  -R_d,  0,  R_d+R_sn+1/(s*C_sn) ];
    
    % Define mesh voltages
    V_top = [   V_cap/s+L_tw*I_tw+L_z*I_zL-V_sn2/s;
                -V_sn1/s;
                V_sn1/s+L_bar*I_barU-L_coil*I_coil;
                V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL;
                -V_sn3/s ];
    
    % Solve for laplace domain currents
    I_top = linsolve(Z_top, V_top);
    
    % Solve for reactive element currents and voltages
    sols.i_top_coil_simp        = matlabFunction(ilaplace(vpa(I_top(4)-I_top(3))));
    sols.i_top_tw_simp          = matlabFunction(ilaplace(vpa(I_top(1))));
    sols.i_top_barU_simp        = matlabFunction(ilaplace(vpa(I_top(3))));
    sols.i_top_barL_simp        = matlabFunction(ilaplace(vpa(I_top(4))));
    sols.i_top_zL_simp          = matlabFunction(ilaplace(vpa(I_top(1)-I_top(4))));
    sols.i_top_zR_simp          = matlabFunction(ilaplace(vpa(I_top(4))));
    
    sols.v_top_cap_simp         = matlabFunction(ilaplace(vpa(-I_top(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_top_sn1_simp         = matlabFunction(ilaplace(vpa((I_top(2)-I_top(3))*(1/(s*C_sn)) + V_sn1/s)));
    sols.v_top_sn2_simp         = matlabFunction(ilaplace(vpa((I_top(1)-I_top(4))*(1/(s*C_sn)) + V_sn2/s)));
    sols.v_top_sn3_simp         = matlabFunction(ilaplace(vpa(I_top(5)*1/(s*C_sn) + V_sn3/s)));
    sols.v_top_sn4_simp         = matlabFunction(ilaplace(vpa(I_top(4)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Top loop calculations done.\n");
    
    %%%%%%%%%%%%%%%%%%%%%%%
    % Bottom loop solutions
    %%%%%%%%%%%%%%%%%%%%%%%
    % Define mesh impedances
    Z_bot = [   1/(s*C_pulse)+s*L_tw+R_tw+R_sn+1/(s*C_sn)+R_z+s*L_z+R_d, -R_ch, -R_sn-1/(s*C_sn), -R_z-s*L_z, 0;
                -R_d, R_d+R_sn+1/(s*C_sn), 0, -R_sn-1/(s*C_sn), 0;
                -R_sn-1/(s*C_sn), 0, 2/(s*C_sn)+2*R_sn+R_bar+s*L_bar+s*L_coil+R_coil, -R_coil-s*L_coil, 0;
                -R_z-s*L_z, -R_sn-1/(s*C_sn), -R_coil-s*L_coil, 1/(s*C_sn)+R_sn+2*s*L_z+2*R_z+R_coil+s*L_coil+R_ch+R_bar+s*L_bar, -R_ch;
                0, 0, 0, -R_ch, R_ch+R_sn+1/(s*C_sn)    ];

    V_bot = [   V_cap/s+L_tw*I_tw-V_sn1/s+L_z*I_zL;
                -V_sn2/s;
                V_sn1/s+L_bar*I_barU-V_sn3/s-L_coil*I_coil;
                V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR+L_bar*I_barL;
                -V_sn4/s    ];

	% Solve for laplace domain currents
    I_bot = linsolve(Z_bot, V_bot);
    
    % Solve for reactive element currents and voltages
    sols.i_bot_coil_simp        = matlabFunction(ilaplace(vpa(I_bot(4)-I_bot(3))));
    sols.i_bot_tw_simp          = matlabFunction(ilaplace(vpa(I_bot(1))));
    sols.i_bot_barU_simp        = matlabFunction(ilaplace(vpa(I_bot(3))));
    sols.i_bot_barL_simp        = matlabFunction(ilaplace(vpa(I_bot(4))));
    sols.i_bot_zL_simp          = matlabFunction(ilaplace(vpa(I_bot(1)-I_bot(4))));
    sols.i_bot_zR_simp          = matlabFunction(ilaplace(vpa(I_bot(4))));
    
    sols.v_bot_cap_simp         = matlabFunction(ilaplace(vpa(-I_bot(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_bot_sn1_simp         = matlabFunction(ilaplace(vpa((I_bot(1)-I_bot(3))*(1/(s*C_sn)) + V_sn1/s)));
    sols.v_bot_sn2_simp         = matlabFunction(ilaplace(vpa((I_bot(2)-I_bot(4))*(1/(s*C_sn)) + V_sn2/s)));
    sols.v_bot_sn3_simp         = matlabFunction(ilaplace(vpa(I_bot(3)*1/(s*C_sn) + V_sn3/s)));
    sols.v_bot_sn4_simp         = matlabFunction(ilaplace(vpa(I_bot(5)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Bottom loop calculations done.\n");
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Falling phase solutions
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Define mesh impedances
    Z_fall = [  1/(s*C_pulse)+s*L_tw+R_tw+1/(s*C_sn)+R_sn+R_z+s*L_z+R_d,  -R_d,  -1/(s*C_sn)-R_sn,  -R_z-s*L_z,   0;
                -R_d,   R_d+1/(s*C_sn)+R_sn,   0,  -1/(s*C_sn)-R_sn,  0;
                -1/(s*C_sn)-R_sn,   0,  R_sn+1/(s*C_sn)+R_bar+s*L_bar+R_d+s*L_coil+R_coil,  -R_coil-s*L_coil,   -R_d;
                -R_z-s*L_z,    -R_sn-1/(s*C_sn), -R_coil-s*L_coil,   2*R_sn+2/(s*C_sn)+2*s*L_z+2*R_z+R_coil+s*L_coil+s*L_bar+R_bar, 0;
                0,   0,  -R_d,  0,  R_d+R_sn+1/(s*C_sn) ];
    
    V_fall = [  V_cap/s+L_tw*I_tw-V_sn1/s+L_z*I_zL;
                -V_sn2/s;
                V_sn1/s+L_bar*I_barU-L_coil*I_coil;
                V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL;
                -V_sn3/s ];
    
    % Solve for laplace domain currents
    I_fall = linsolve(Z_fall, V_fall);
    
    % Solve for reactive element currents and voltages
    sols.i_fall_coil_simp       = matlabFunction(ilaplace(vpa(I_fall(4)-I_fall(3))));
    sols.i_fall_tw_simp         = matlabFunction(ilaplace(vpa(I_fall(1))));
    sols.i_fall_barU_simp       = matlabFunction(ilaplace(vpa(I_fall(3))));
    sols.i_fall_barL_simp       = matlabFunction(ilaplace(vpa(I_fall(4))));
    sols.i_fall_zL_simp         = matlabFunction(ilaplace(vpa(I_fall(1)-I_fall(4))));
    sols.i_fall_zR_simp         = matlabFunction(ilaplace(vpa(I_fall(4))));
    
    sols.v_fall_cap_simp        = matlabFunction(ilaplace(vpa(-I_fall(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_fall_sn1_simp        = matlabFunction(ilaplace(vpa((I_fall(1)-I_fall(3))*(1/(s*C_sn)) + V_sn1/s)));
    sols.v_fall_sn2_simp        = matlabFunction(ilaplace(vpa((I_fall(2)-I_fall(4))*(1/(s*C_sn)) + V_sn2/s)));
    sols.v_fall_sn3_simp        = matlabFunction(ilaplace(vpa(I_fall(5)*1/(s*C_sn) + V_sn3/s)));
    sols.v_fall_sn4_simp        = matlabFunction(ilaplace(vpa(I_fall(4)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Falling phase calculations done.\n");
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Falling phase (diode conduction) solutions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    Z_fall_d = [    1/(s*C_pulse)+R_tw+s*L_tw+R_sn+1/(s*C_sn)+R_z+s*L_z+R_ch,   -R_sn-1/(s*C_sn),   0,  -R_z-s*L_z, -R_ch,  0;
                    -R_sn-1/(s*C_sn),   1/(s*C_sn)+R_sn+R_d,    -R_d,   0,  0,  0;
                    0,  -R_d,   R_d+R_bar+s*L_bar+R_ch+s*L_coil+R_coil, -R_coil-s*L_coil,   0,  -R_ch;
                    -R_z-s*L_z, 0,  -R_coil-s*L_coil,   2/(s*C_sn)+2*R_sn+2*s*L_z+2*R_z+R_coil+s*L_coil+s*L_bar+R_bar,  -R_sn-1/(s*C_sn),   0;
                    -R_ch,  0,  0,  -R_sn-1/(s*C_sn),   R_ch+R_sn+1/(s*C_sn),   0;
                    0,  0,  -R_ch,  0,  0,  R_ch+R_sn+1/(s*C_sn) ];
	
	V_fall_d = [    V_cap/s+L_tw*I_tw-V_sn1/s+L_z*I_zL;
                    V_sn1/s+V_d/s;
                    -V_d/s+L_bar*I_barU-L_coil*I_coil;
                    V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL;
                    -V_sn2/s;
                    -V_sn3/s ];
	
	I_fall_d = linsolve(Z_fall_d, V_fall_d);
    
    sols.i_fall_coil_d_simp     = matlabFunction(ilaplace(vpa(I_fall_d(4)-I_fall_d(3))));
    sols.i_fall_tw_d_simp       = matlabFunction(ilaplace(vpa(I_fall_d(1))));
    sols.i_fall_barU_d_simp     = matlabFunction(ilaplace(vpa(I_fall_d(3))));
    sols.i_fall_barL_d_simp     = matlabFunction(ilaplace(vpa(I_fall_d(4))));
    sols.i_fall_zL_d_simp       = matlabFunction(ilaplace(vpa(I_fall_d(1)-I_fall_d(4))));
    sols.i_fall_zR_d_simp       = matlabFunction(ilaplace(vpa(I_fall_d(4))));
    sols.i_fall_diode_simp      = matlabFunction(ilaplace(vpa(I_fall_d(3)-I_fall_d(2))));
    
    sols.v_fall_cap_d_simp      = matlabFunction(ilaplace(vpa(-I_fall_d(1)*1/(s*C_pulse) + V_cap/s)));
    sols.v_fall_sn1_d_simp      = matlabFunction(ilaplace(vpa((I_fall_d(1)-I_fall_d(2))*1/(s*C_sn) + V_sn1/s)));
    sols.v_fall_sn2_d_simp      = matlabFunction(ilaplace(vpa((I_fall_d(5)-I_fall_d(4))*1/(s*C_sn) + V_sn2/s)));
    sols.v_fall_sn3_d_simp      = matlabFunction(ilaplace(vpa(I_fall_d(6)*1/(s*C_sn) + V_sn3/s)));
    sols.v_fall_sn4_d_simp      = matlabFunction(ilaplace(vpa(I_fall_d(4)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Falling phase (dcon) calculations done.\n");
	%}
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Interleaved circuit
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %
    Z_int = [   1/(s*C_pulse)+R_tw+s*L_tw+R_ch+R_z+s*L_z+R_sn+1/(s*C_sn),   -R_ch,  0,  -R_z-s*L_z-R_sn-1/(s*C_sn);
                -R_ch,  R_ch+R_sn+1/(s*C_sn),   -R_sn-1/(s*C_sn), 0;
                0,  -R_sn-1/(s*C_sn),   2/(s*C_sn)+2*R_sn+R_bar+s*L_bar+R_coil+s*L_coil,    -R_coil-s*L_coil;
                -R_z-s*L_z-R_sn-1/(s*C_sn), 0,  -R_coil-s*L_coil,   2/(s*C_sn)+2*R_sn+2*R_z+2*s*L_z+R_coil+s*L_coil+R_bar+s*L_bar ];
    
	V_int = [   V_cap/s+L_tw*I_tw+L_z*I_zL-V_sn2/s;
                -V_sn1/s;
                V_sn1/s+L_bar*I_barU-V_sn3/s-L_coil*I_coil;
                V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL ];
    
	I_int = linsolve(Z_int, V_int);
    
    sols.i_int_coil_simp        = matlabFunction(ilaplace(vpa(I_int(4)-I_int(3))));
	sols.i_int_tw_simp          = matlabFunction(ilaplace(vpa(I_int(1))));
    sols.i_int_barU_simp        = matlabFunction(ilaplace(vpa(I_int(3))));
	sols.i_int_barL_simp        = matlabFunction(ilaplace(vpa(I_int(4))));
	sols.i_int_zL_simp          = matlabFunction(ilaplace(vpa(I_int(1)-I_int(4))));
	sols.i_int_zR_simp          = matlabFunction(ilaplace(vpa(I_int(4))));

    sols.v_int_cap_simp         = matlabFunction(ilaplace(vpa((-I_int(1)*1/(s*C_pulse) + V_cap/s))));
	sols.v_int_sn1_simp         = matlabFunction(ilaplace(vpa((I_int(2)-I_int(3))*(1/(s*C_sn)) + V_sn1/s)));
	sols.v_int_sn2_simp         = matlabFunction(ilaplace(vpa((I_int(1)-I_int(4))*(1/(s*C_sn)) + V_sn2/s)));
	sols.v_int_sn3_simp         = matlabFunction(ilaplace(vpa(I_int(3)*1/(s*C_sn) + V_sn3/s)));
    sols.v_int_sn4_simp         = matlabFunction(ilaplace(vpa(I_int(4)*1/(s*C_sn) + V_sn4/s)));
    
    Z_int_rev = [   1/(s*C_pulse)+R_tw+s*L_tw+2*R_sn+2/(s*C_sn)+R_z+s*L_z,   -R_sn-1/(s*C_sn),   0,  -R_z-s*L_z-R_sn-1/(s*C_sn);
                    -R_sn-1/(s*C_sn),   1/(s*C_sn)+R_sn+R_bar+s*L_bar+R_ch+s*L_coil+R_coil, -R_ch,  -R_coil-s*L_coil;
                    0,  -R_ch,  R_ch+R_sn+1/(s*C_sn),   0;
                    -R_z-s*L_z-R_sn-1/(s*C_sn), -R_coil-s*L_coil,   0,  2/(s*C_sn)+2*R_sn+2*s*L_z+2*R_z+R_coil+s*L_coil+R_bar+s*L_bar ];
    
	V_int_rev = [   V_cap/s+L_tw*I_tw-V_sn1/s+L_z*I_zL-V_sn2/s;
                    V_sn1/s+L_bar*I_barU-L_coil*I_coil;
                    -V_sn3/s;
                    V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL ];
	
	I_int_rev = linsolve(Z_int_rev, V_int_rev);
    
    sols.i_int_rev_coil_simp    = matlabFunction(ilaplace(vpa(I_int_rev(4)-I_int_rev(2))));
    sols.i_int_rev_tw_simp      = matlabFunction(ilaplace(vpa(I_int_rev(1))));
    sols.i_int_rev_barU_simp    = matlabFunction(ilaplace(vpa(I_int_rev(2))));
    sols.i_int_rev_barL_simp    = matlabFunction(ilaplace(vpa(I_int_rev(4))));
    sols.i_int_rev_zL_simp      = matlabFunction(ilaplace(vpa(I_int_rev(1)-I_int_rev(4))));
    sols.i_int_rev_zR_simp      = matlabFunction(ilaplace(vpa(I_int_rev(4))));
    
    sols.v_int_rev_cap_simp     = matlabFunction(ilaplace(vpa((-I_int_rev(1)*1/(s*C_pulse) + V_cap/s))));
    sols.v_int_rev_sn1_simp     = matlabFunction(ilaplace(vpa((I_int_rev(1)-I_int_rev(2))*1/(s*C_sn) + V_sn1/s)));
    sols.v_int_rev_sn2_simp     = matlabFunction(ilaplace(vpa((I_int_rev(1)-I_int_rev(4))*1/(s*C_sn) + V_sn2/s)));
    sols.v_int_rev_sn3_simp     = matlabFunction(ilaplace(vpa(I_int_rev(3)*1/(s*C_sn) + V_sn3/s)));
    sols.v_int_rev_sn4_simp     = matlabFunction(ilaplace(vpa(I_int_rev(4)*1/(s*C_sn) + V_sn4/s)));
	
    fprintf("Interleaved phase calculations done.\n");
    %}
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Disabled circuit
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %
    
    Z_off = [   1/(s*C_pulse)+R_tw+s*L_tw+2*R_sn+2/(s*C_sn)+R_z+s*L_z,  -R_sn-1/(s*C_sn),   -R_z-s*L_z-R_sn-1/(s*C_sn);
                -R_sn-1/(s*C_sn),   2/(s*C_sn)+2*R_sn+R_bar+s*L_bar+s*L_coil+R_coil,    -R_coil-s*L_coil;
                -R_z-s*L_z-R_sn-1/(s*C_sn), -R_coil-s*L_coil,   2/(s*C_sn)+2*R_sn+2*s*L_z+2*R_z+R_coil+s*L_coil+s*L_bar+R_bar ];
	
	V_off = [   V_cap/s+L_tw*I_tw-V_sn1/s+L_z*I_zL-V_sn2/s;
                V_sn1/s+L_bar*I_barU-V_sn3/s-L_coil*I_coil;
                V_sn2/s-L_z*I_zL+L_coil*I_coil+L_z*I_zR-V_sn4/s+L_bar*I_barL ];
	
	I_off = linsolve(Z_off, V_off);
    
    sols.i_off_coil_simp    = matlabFunction(ilaplace(vpa(I_off(3)-I_off(2))));
    sols.i_off_tw_simp      = matlabFunction(ilaplace(vpa(I_off(1))));
    sols.i_off_barU_simp    = matlabFunction(ilaplace(vpa(I_off(2))));
    sols.i_off_barL_simp    = matlabFunction(ilaplace(vpa(I_off(3))));
    sols.i_off_zL_simp      = matlabFunction(ilaplace(vpa(I_off(1)-I_off(3))));
    sols.i_off_zR_simp      = matlabFunction(ilaplace(vpa(I_off(3))));
    
    sols.v_off_cap_simp     = matlabFunction(ilaplace(vpa((-I_off(1)*1/(s*C_pulse) + V_cap/s))));
    sols.v_off_sn1_simp     = matlabFunction(ilaplace(vpa((I_off(1)-I_off(2))*1/(s*C_sn) + V_sn1/s)));
    sols.v_off_sn2_simp     = matlabFunction(ilaplace(vpa((I_off(1)-I_off(3))*1/(s*C_sn) + V_sn2/s)));
    sols.v_off_sn3_simp     = matlabFunction(ilaplace(vpa(I_off(2)*1/(s*C_sn) + V_sn3/s)));
    sols.v_off_sn4_simp     = matlabFunction(ilaplace(vpa(I_off(3)*1/(s*C_sn) + V_sn4/s)));
    
    fprintf("Disabled phase calculations done.\n");
    %}
    
end