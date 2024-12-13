The functions provided by the API are broadly divided into the following categories:

Functions controlling the system state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autosummary::
    MTMSApi.MTMSApi.start_device
    MTMSApi.MTMSApi.stop_device
    MTMSApi.MTMSApi.start_session
    MTMSApi.MTMSApi.stop_session
    MTMSApi.MTMSApi.end

Functions querying the system state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autosummary::
    MTMSApi.MTMSApi.get_device_state
    MTMSApi.MTMSApi.get_session_state
    MTMSApi.MTMSApi.get_voltage
    MTMSApi.MTMSApi.get_temperature
    MTMSApi.MTMSApi.get_pulse_count
    MTMSApi.MTMSApi.get_time
    MTMSApi.MTMSApi.get_target_voltages
    MTMSApi.MTMSApi.get_maximum_intensity

Functions related to event requests
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. autosummary::
    MTMSApi.MTMSApi.send_pulse
    MTMSApi.MTMSApi.send_charge
    MTMSApi.MTMSApi.send_discharge
    MTMSApi.MTMSApi.send_trigger_out
    MTMSApi.MTMSApi.send_immediate_charge_or_discharge_to_all_channels
    MTMSApi.MTMSApi.send_immediate_full_discharge_to_all_channels

Helper functions
~~~~~~~~~~~~~~~~

.. autosummary::
    MTMSApi.MTMSApi.wait_forever
    MTMSApi.MTMSApi.analyze_mep


.. autoclass:: MTMSApi.MTMSApi
