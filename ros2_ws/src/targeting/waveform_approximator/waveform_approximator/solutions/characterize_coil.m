% Function for finding out the system inductance for the rising phase of a
% monophasic pulse waveform

function [R_avg, L_avg] = characterize_coil(pulse_data, show_plots, verbose)
    % define constants
    rogowski_ratio = 0.5e-3;
    L_guess = 15e-6;
    R_guess = 60e-3;

    roi_delay = 10e-6;
    pulse_length = 60e-6;

    th_tol = 70;    % threshold tolerance, adjust this to better find the starting location of pulses

    R_list = zeros(1, length(pulse_data));
    L_list = zeros(1, length(pulse_data));

    tile_count = ceil(sqrt(length(pulse_data)));
    
    if strcmp(show_plots, 'true')
        tiledlayout(tile_count, tile_count);
    end

    for i = 1:length(pulse_data)
        % load data, offset correction
        t_meas = pulse_data(i).t_meas;
        i_offset = mean(pulse_data(i).i_meas(1:100)/rogowski_ratio);
        i_meas = abs(pulse_data(i).i_meas/rogowski_ratio - i_offset);

        % detect beginning of pulse (>2% of maximum amplitude)
        v_th = max(abs(i_meas))*0.02;
        t_index = find(i_meas >= v_th, 1) - th_tol;
        t_offset = t_meas(t_index);

        % determine areas of investigation for fit
        res = t_meas(2)-t_meas(1);
        start_idx = t_index + round(roi_delay/res);
        stop_idx = t_index + round((pulse_length-2e-6)/res);
        i_fit = i_meas(start_idx:stop_idx);
        t_fit = t_meas(start_idx:stop_idx)-t_offset;

        % define fitting model
        ft = fittype('(2*Vc*exp(-(R*t)/(2*L))*sin((t*(-R^2 + 4000*L)^(1/2))/(2*L)))/(-R^2 + 4000*L)^(1/2)', 'dependent', 'I', 'independent', 't', 'coefficients', {'R', 'L'}, 'problem', 'Vc');

        voltage = pulse_data(i).voltage.before_pulse;
        fm = fit(t_fit, i_fit, ft, 'StartPoint', [R_guess, L_guess], 'problem', voltage);

        % plot fit vs real
        syms Vc R L t
        i_coil = matlabFunction((2*Vc*exp(-(R*t)/(2*L))*sin((t*(-R^2 + 4000*L)^(1/2))/(2*L)))/(-R^2 + 4000*L)^(1/2));
        time = 0:0.01e-6:60e-6;
        
        coeff_values = coeffvalues(fm);
        R_coil = coeff_values(1);
        L_coil = coeff_values(2);

        if strcmp(show_plots, 'true')
            nexttile;
            plot(time/1e-6, i_coil(L_coil, R_coil, voltage, time));
            hold on
            plot(t_fit/1e-6, i_fit);
        end
        
        if strcmp(verbose, 'true')
            fprintf("L = %.2f uH, R = %.1f mOhm\n", L_coil/1e-6, R_coil/1e-3);
        end
        
        L_list(i) = L_coil;
        R_list(i) = R_coil;

    end
    
    R_avg = mean(R_list);
    L_avg = mean(L_list);
    
    if strcmp(verbose, 'true')
        fprintf("L_mean = %.2f uH\nR_mean = %.2f mOhm\n", mean(L_list)/1e-6, mean(R_list)/1e-3);
    end
end