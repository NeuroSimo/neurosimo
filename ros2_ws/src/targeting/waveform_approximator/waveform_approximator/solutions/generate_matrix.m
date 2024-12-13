% Function for generating a parameter matrix based on 
function system_matrix = generate_matrix(R_avg, L_avg, single, verbose)
    
    C_pulse_list = [1.02]*1e-3;
    R_sn = 1;
    C_sn_list = [1]*1e-6;
    R_ch_list = [2]*1e-3;
    R_d_list = [2]*1e-3;
    R_tw_list = [3]*1e-3;
    L_tw_list = [0.15]*1e-6;
    R_bar_list = [3]*1e-3;
    L_bar_list = [0.1]*1e-6;
    R_z_list = [3]*1e-3;
    L_z_list = [0.13]*1e-6;
    
    if strcmp(single, 'true')
        R_coil_list = R_avg - 11.44e-3;
        L_coil_list = L_avg - 0.37e-6;
    else
        R_c_eff = R_avg - mean(R_tw_list) - 2*mean(R_ch_list) - mean(R_z_list) - mean(R_bar_list);
        L_c_eff = L_avg - mean(L_tw_list) - mean(L_z_list) - mean(L_bar_list);

        R_coil_list = [R_c_eff-3e-3, R_c_eff-2e-3, R_c_eff-1e-3, R_c_eff, R_c_eff+1e-3, R_c_eff+2e-3, R_c_eff+3e-3];
        L_coil_list = [L_c_eff-0.3e-6, L_c_eff-0.2e-6, L_c_eff-0.1e-6, L_c_eff, L_c_eff+0.1e-6, L_c_eff+0.2e-6, L_c_eff+0.3e-6];
    end

    idx = 1;
    for RCOIL = 1:length(R_coil_list)
        for LCOIL = 1:length(L_coil_list)
            for CPULSE = 1:length(C_pulse_list)
                for CSN = 1:length(C_sn_list)
                    for RCH = 1:length(R_ch_list)
                        for RD = 1:length(R_d_list)
                            for RTW = 1:length(R_tw_list)
                                for LTW = 1:length(L_tw_list)
                                    for RBAR = 1:length(R_bar_list)
                                        for LBAR = 1:length(L_bar_list)
                                            for RZ = 1:length(R_z_list)
                                                for LZ = 1:length(L_z_list)
                                                    system_matrix(idx).params = [C_pulse_list(CPULSE), R_coil_list(RCOIL), L_coil_list(LCOIL), R_sn, ...
                                                                                 C_sn_list(CSN), R_tw_list(RTW), L_tw_list(LTW), R_bar_list(RBAR), ...
                                                                                 L_bar_list(LBAR), R_z_list(RZ), L_z_list(LZ), R_ch_list(RCH), R_d_list(RD)];
                                                    idx = idx+1;
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
    
    if strcmp(verbose, 'true')
        fprintf("Parameter matrix generated, number of entries: %d\n", length(system_matrix));
    end
    
end