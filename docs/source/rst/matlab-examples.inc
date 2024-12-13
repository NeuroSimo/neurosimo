Initialize device and session
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: matlab

    api = MTMSApi();

    api.start_device();
    api.start_session();

Execute single events
~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Charge channel 1 to 20 V
.. code-block:: matlab

    channel = 1;
    target_voltage = 20;
    execution_condition = api.execution_conditions.IMMEDIATE;
    time = 10.0;
    wait_for_completion = true;

    api.send_charge(channel, target_voltage, execution_condition, time, wait_for_completion);


.. rubric:: Send pulse on channel 1, using the default waveform
.. code-block:: matlab

    waveform = api.get_default_waveform(channel);
    reverse_polarity = false;

    api.send_pulse(channel, waveform, execution_condition, time, reverse_polarity, wait_for_completion);

.. rubric:: Discharge channel 1 completely
.. code-block:: matlab

    api.send_discharge(channel, 0, execution_condition, time, wait_for_completion);


.. rubric:: Send trigger out on port 1.
.. code-block:: matlab

    port = 1;
    duration_us = 1000;

    api.send_trigger_out(port, duration_us, execution_condition, time, wait_for_completion);


Send pulse on channel 1 and analyze MEP.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Use default waveform for the pulse.
.. code-block:: matlab

    waveform = api.get_default_waveform(channel);
    reverse_polarity = false;


.. rubric:: Generate a timed pulse.
.. code-block:: matlab

    channel = 1;
    execution_condition = api.execution_conditions.TIMED;
    time = api.get_time() + 3.0;
    wait_for_completion = false;  % Note that this needs to be false so that MEP can be queried for before the pulse is executed.

    api.send_pulse(channel, waveform, execution_condition, time, reverse_polarity, wait_for_completion);

.. rubric:: Analyze MEP on EMG channel 1, coinciding with the pulse.
.. code-block:: matlab

    emg_channel = 1;

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


Targeting
~~~~~~~~~

.. rubric:: Get channel voltages
.. code-block:: matlab

    displacement_x = 5;  % mm
    displacement_y = 5;  % mm
    rotation_angle = 90;  % deg
    intensity = 20;  % V/m

    [target_voltages, reverse_polarities] = api.get_target_voltages(displacement_x, displacement_y, rotation_angle, intensity);

.. rubric:: Get maximum intensity
.. code-block:: matlab

    maximum_intensity = api.get_maximum_intensity(displacement_x, displacement_y, rotation_angle);


.. rubric:: Charge all channels to target voltages.
.. code-block:: matlab

    wait_for_completion = true;
    api.send_immediate_charge_or_discharge_to_all_channels(target_voltages, wait_for_completion);

.. rubric:: Send default pulse to all channels.
.. code-block:: matlab

    api.send_immediate_default_pulse_to_all_channels(reverse_polarities, wait_for_completion);


Targeting combined with MEP analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Get channel voltages
.. code-block:: matlab

    displacement_x = 5;  % mm
    displacement_y = 5;  % mm
    rotation_angle = 90;  % deg
    intensity = 20;  % V/m

    [target_voltages, reverse_polarities] = api.get_target_voltages(displacement_x, displacement_y, rotation_angle, intensity);

.. rubric:: Charge all channels to target voltages.
.. code-block:: matlab

    wait_for_completion = true;
    api.send_immediate_charge_or_discharge_to_all_channels(target_voltages, wait_for_completion);


.. rubric:: Send default pulse to all channels.
.. code-block:: matlab

    wait_for_completion = false;
    time = api.get_time() + 3.0;

    api.send_timed_default_pulse_to_all_channels(reverse_polarities, time, wait_for_completion);

.. rubric:: Analyze MEP on EMG channel 1, coinciding with the pulse.
.. code-block:: matlab

    emg_channel = 1;

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


Restart session
~~~~~~~~~~~~~~~~~~

.. code-block:: matlab

    api.stop_session()
    api.start_session()


Stop device
~~~~~~~~~~~

.. code-block:: matlab

    api.stop_device()


End-to-end experiment
~~~~~~~~~~~~~~~~~~~~~

    TODO: Nothing here at the moment.
