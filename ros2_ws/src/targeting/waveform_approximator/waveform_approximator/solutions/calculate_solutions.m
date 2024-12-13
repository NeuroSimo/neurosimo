function sols = calculate_solutions(system_matrix)

    if length(system_matrix) < 4
         for i = 1:length(system_matrix)
            % Calculate solutions
            tic
            sols(i).f = solve_phase_functions(system_matrix(i).params(1), system_matrix(i).params(2), system_matrix(i).params(3), ...
                                              system_matrix(i).params(4), system_matrix(i).params(5), system_matrix(i).params(6), ...
                                              system_matrix(i).params(7), system_matrix(i).params(8), system_matrix(i).params(9), ...
                                              system_matrix(i).params(10), system_matrix(i).params(11), system_matrix(i).params(12), ...
                                              system_matrix(i).params(13));
            % Add metadata
            sols(i).metadata.C_pulse = system_matrix(i).params(1);
            sols(i).metadata.R_coil = system_matrix(i).params(2);
            sols(i).metadata.L_coil = system_matrix(i).params(3);
            sols(i).metadata.R_sn = system_matrix(i).params(4);
            sols(i).metadata.C_sn = system_matrix(i).params(5);
            sols(i).metadata.R_tw = system_matrix(i).params(6);
            sols(i).metadata.L_tw = system_matrix(i).params(7);
            sols(i).metadata.R_bar = system_matrix(i).params(8);
            sols(i).metadata.L_bar = system_matrix(i).params(9);
            sols(i).metadata.R_z = system_matrix(i).params(10);
            sols(i).metadata.L_z = system_matrix(i).params(11);
            sols(i).metadata.R_ch = system_matrix(i).params(12);
            sols(i).metadata.R_d = system_matrix(i).params(13);
            fprintf("Iteration round %d finished!\n", i);
            toc
         end
    else
        parfor i = 1:length(system_matrix)
            % Calculate solutions
            tic
            sols(i).f = solve_phase_functions(system_matrix(i).params(1), system_matrix(i).params(2), system_matrix(i).params(3), ...
                                              system_matrix(i).params(4), system_matrix(i).params(5), system_matrix(i).params(6), ...
                                              system_matrix(i).params(7), system_matrix(i).params(8), system_matrix(i).params(9), ...
                                              system_matrix(i).params(10), system_matrix(i).params(11), system_matrix(i).params(12), ...
                                              system_matrix(i).params(13));
            % Add metadata
            sols(i).metadata.C_pulse = system_matrix(i).params(1);
            sols(i).metadata.R_coil = system_matrix(i).params(2);
            sols(i).metadata.L_coil = system_matrix(i).params(3);
            sols(i).metadata.R_sn = system_matrix(i).params(4);
            sols(i).metadata.C_sn = system_matrix(i).params(5);
            sols(i).metadata.R_tw = system_matrix(i).params(6);
            sols(i).metadata.L_tw = system_matrix(i).params(7);
            sols(i).metadata.R_bar = system_matrix(i).params(8);
            sols(i).metadata.L_bar = system_matrix(i).params(9);
            sols(i).metadata.R_z = system_matrix(i).params(10);
            sols(i).metadata.L_z = system_matrix(i).params(11);
            sols(i).metadata.R_ch = system_matrix(i).params(12);
            sols(i).metadata.R_d = system_matrix(i).params(13);
            fprintf("Iteration round %d finished!\n", i);
            toc
        end
    end
end