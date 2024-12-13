Initialize device and session
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block::

    from event_interfaces.msg import ExecutionCondition, WaveformPhase

    from mep_interfaces.msg import MepConfiguration, PreactivationCheck
    from eeg_interfaces.msg import TimeWindow

    from MTMSApi import MTMSApi

    api = MTMSApi()

    api.start_device()
    api.start_session()


Execute single events
~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Charge channel 1 to 20 V
.. code-block::

    channel = 1
    target_voltage = 20
    execution_condition = ExecutionCondition.IMMEDIATE
    time = 10.0

    api.send_charge(
        channel=channel,
        target_voltage=target_voltage,
        execution_condition=execution_condition,
        time=time,
    )


.. rubric:: Send pulse on channel 1, using the default waveform
.. code-block::

    waveform = api.get_default_waveform(channel)
    reverse_polarity = False

    api.send_pulse(
        channel=channel,
        waveform=waveform,
        execution_condition=execution_condition,
        time=time,
        reverse_polarity=reverse_polarity,
    )

.. rubric:: Discharge channel 1 completely
.. code-block::

    api.send_discharge(
        channel=channel,
        target_voltage=0,
        execution_condition=execution_condition,
        time=time,
    )

.. rubric:: Send trigger out on port 1.
.. code-block::

    port = 1
    duration_us = 1000

    api.send_trigger_out(
        port=port,
        duration_us=duration_us,
        execution_condition=execution_condition,
        time=time,
    )


Send pulse on channel 1 and analyze MEP.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Use default waveform for the pulse.
.. code-block::

    waveform = api.get_default_waveform(channel)
    reverse_polarity = False

.. rubric:: Generate a timed pulse.
.. code-block::

    channel = 1
    execution_condition = ExecutionCondition.TIMED
    time = api.get_time() + 3.0
    wait_for_completion = False  # Note that this needs to be false so that MEP can be queried for before the pulse is executed.

    api.send_pulse(
        channel=channel,
        waveform=waveform,
        execution_condition=execution_condition,
        time=time,
        reverse_polarity=reverse_polarity,
        wait_for_completion=wait_for_completion,
    )

.. rubric:: Analyze MEP on EMG channel 1, coinciding with the pulse.
.. code-block::

    mep_configuration = MepConfiguration(
        emg_channel=1,

        time_window=TimeWindow(
            start=0.020,  # in ms, after the stimulation pulse
            end=0.040,  # in ms
        ),
        preactivation_check=PreactivationCheck(
            enabled=True,
            time_window=TimeWindow(
                start=-0.040,  # in ms, minus sign indicates that the window starts before the stimulation pulse
                end=-0.020,
            ),
            voltage_range_limit=70.0,  # Maximum allowed voltage range inside the time window, in uV.
        ),
    )

    mep, errors = api.analyze_mep(
        time=time,
        mep_configuration=mep_configuration,
    )

    amplitude = mep.amplitude
    latency = mep.latency


Targeting
~~~~~~~~~

.. rubric:: Get channel voltages
.. code-block::

    displacement_x = 5  # mm
    displacement_y = 5  # mm
    rotation_angle = 90  # deg
    intensity = 20  # V/m

    target_voltages, reverse_polarities = api.get_target_voltages(
        displacement_x=displacement_x,
        displacement_y=displacement_y,
        rotation_angle=rotation_angle,
        intensity=intensity,
    )

.. rubric:: Get maximum intensity
.. code-block::

    maximum_intensity = api.get_maximum_intensity(
        displacement_x=displacement_x,
        displacement_y=displacement_y,
        rotation_angle=rotation_angle,
    )

.. rubric:: Charge all channels to target voltages.
.. code-block::

    api.send_immediate_charge_or_discharge_to_all_channels(
        target_voltages=target_voltages,
    )

.. rubric:: Send default pulse to all channels.
.. code-block::

    api.send_immediate_default_pulse_to_all_channels(
        reverse_polarities=reverse_polarities,
    )


Targeting combined with MEP analysis
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. rubric:: Get channel voltages
.. code-block::

    displacement_x = 5  # mm
    displacement_y = 5  # mm
    rotation_angle = 90  # deg
    intensity = 20  # V/m

    target_voltages, reverse_polarities = api.get_target_voltages(
        displacement_x=displacement_x,
        displacement_y=displacement_y,
        rotation_angle=rotation_angle,
        intensity=intensity,
    )

.. rubric:: Charge all channels to target voltages.
.. code-block::

    api.send_immediate_charge_or_discharge_to_all_channels(
        target_voltages=target_voltages,
    )

    # Send default pulse to all channels.
    wait_for_completion = False  # Note that this needs to be false so that MEP can be queried for before the pulse is executed.
    time = api.get_time() + 3.0

    api.send_timed_default_pulse_to_all_channels(
        reverse_polarities=reverse_polarities,
        time=time,
        wait_for_completion=wait_for_completion,
    )

.. rubric:: Analyze MEP on EMG channel 1, coinciding with the pulse.
.. code-block::

    mep_configuration = MepConfiguration(
        emg_channel=1,

        time_window=TimeWindow(
            start=0.020,  # in ms, after the stimulation pulse
            end=0.040,  # in ms
        ),
        preactivation_check=PreactivationCheck(
            enabled=True,
            time_window=TimeWindow(
                start=-0.040,  # in ms, minus sign indicates that the window starts before the stimulation pulse
                end=-0.020,
            ),
            voltage_range_limit=70.0,  # Maximum allowed voltage range inside the time window, in uV.
        ),
    )

    mep, errors = api.analyze_mep(
        time=time,
        mep_configuration=mep_configuration,
    )

    amplitude = mep.amplitude
    latency = mep.latency


Restart session
~~~~~~~~~~~~~~~~~~

.. code-block::

    api.stop_session()
    api.start_session()


Stop device
~~~~~~~~~~~

.. code-block::

    api.stop_device()


End-to-end experiment
~~~~~~~~~~~~~~~~~~~~~

Implementation of a simple experiment, consisting of charging the coil on channel 1 to 200 V and generating a single pulse with a monophasic waveform.

.. code-block::

    from event_interfaces.msg import ExecutionCondition

    from MTMSApi import MTMSApi

    api = MTMSApi()

    api.start_device()
    api.start_session()

    channel = 1
    target_voltage = 200
    execution_condition = ExecutionCondition.TIMED
    time = 1.0

    api.send_charge(
        channel=channel,
        target_voltage=target_voltage,
        execution_condition=execution_condition,
        time=time,
    )

    waveform = api.get_default_waveform(channel)

    time = 2.0

    api.send_pulse(
        channel=channel,
        waveform=waveform,
        execution_condition=execution_condition,
        time=time,
    )

    time = 2.5

    api.wait_until(time)

    api.stop_session()
    api.stop_device()

    api.end()
