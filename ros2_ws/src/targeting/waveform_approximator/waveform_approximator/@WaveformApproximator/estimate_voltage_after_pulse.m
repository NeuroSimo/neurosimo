function voltage_after = estimate_voltage_after_pulse(obj, voltage_before, waveform)
    state_trajectory = obj.generate_state_trajectory_from_waveform(voltage_before, waveform);

    % TODO: Ensure that the voltages computed this way match with the actual voltages
    %   after a pulse; if not, use an alternative method to compute the voltage after pulse.
    voltage_after = state_trajectory(end).V_c;
end
