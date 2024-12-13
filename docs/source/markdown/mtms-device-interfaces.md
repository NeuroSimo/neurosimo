# mTMS device interface specification document
## Introduction
This specification document gives overview of the mTMS device control ROS2 interface and
its functions, and gives the description of the required interfaces to be
implemented and their expected behaviour for any device to be used.

### Operating environment
TODO: Add information on the mTMS device and system requirements

### System overview
TODO: Describe the system state behaviour and general usage

## Interface Diagram
TODO: Add interface diagram

## Usage specification
The mTMS device interface includes ROS2 services, publishers and subscribers. Some
topics are linked in behaviour or requires specific system state for usage. This document
will not show every message type implementation and rather will showcase them when relevant,
and exact implementation details can be looked in the interface definitions.

### Topic overview
The table below, lists all the ROS2 topics used by the mTMS device interface, and their
types and packages locations for their message definitions. Quality of Service (QoS)
policies and other behaviour will be described in their own sections.

| Topic                            | Type         | Message definition                            |
|----------------------------------|--------------|-----------------------------------------------|
| `/mtms_device/allow_stimulation` | Service      | `mtms_device_interfaces.srv.AllowStimulation` |
| `/mtms_device/send_settings`     | Service      | `mtms_device_interfaces.srv.SendSettings`     |
| `/mtms_device/start_device`      | Service      | `mtms_device_interfaces.srv.StartDevice`      |
| `/mtms_device/stop_device`       | Service      | `mtms_device_interfaces.srv.StopDevice`       |
| `/mtms_device/start_session`     | Service      | `mtms_device_interfaces.srv.StartSession`     |
| `/mtms_device/stop_session`      | Service      | `mtms_device_interfaces.srv.StopSession`      |
| `/mtms_device/request_events`    | Service      | `mtms_device_interfaces.srv.RequestEvents`    |
| `/event/charge_feedback`         | Publisher    | `event_interfaces.msg.ChargeFeedback`         |
| `/event/discharge_feedback`      | Publisher    | `event_interfaces.msg.DischargeFeedback`      |
| `/event/pulse_feedback`          | Publisher    | `event_interfaces.msg.PulseFeedback`          |
| `/event/trigger_out_feedback`    | Publisher    | `event_interfaces.msg.TriggerOutFeedback`     |
| `/node/message`                  | Publisher    | `std_msgs.msg.String`                         |
| `/mtms_device/system_state`      | Publisher    | `mtms_device_interfaces.msg.SystemState`      |

### Interface description
The mTMS device interface functions can be divided to follow sections, which can
contain 1 or more different ROS2 topics:
- [System state Publishing](#system-state)
- [Device state management](#device-state-management)
- [Session state management](#session-state-management)
- [Charging](#charging)
- [Discharging](#discharging)
- [Device configuration and settings](#device-configuration-and-settings)
- [Pulses](#pulses)
- [Outgoing triggers](#trigger-out)
- [FPGA status](#fpga-status)

TODO: Add descriptions for error conditions.

#### System state publishing
The mTMS device publishes its system state on a regular intervals to the `/mtms_device/system_state` topic

    ChannelState[] channel_states

    SystemError system_error_cumulative
    SystemError system_error_current
    SystemError system_error_emergency

    StartupError startup_error

    DeviceState device_state
    SessionState session_state

    float64 time

where each of the coil channels in the mTMS device has its own `ChannelState` object in the
channel_states object. `DeviceState` and `SessionState` describe the device and session
states, which are described in their own sections.

System state is published with 20ms interval tolerance for message delay is 5ms.
System state is published with following QoS policy (further details on the properties in
ROS2 Documentation):


| QoS property   | value               |
|----------------|---------------------|
| History        | KEEP_LAST           |
| Depth          | 1                   |
| Reliability    | RELIABLE            |
| Durability     | TRANSIENT_LOCAL     |
| Deadline       | 25ms                |
| Lifespan       | 25ms                |
| Liveliness     | Default (automatic) |
| Lease duration | Default             |

#### Device state management
The device state is management by service topics `/mtms_device/start_device`
and `/mtms_device/stop_device`.

Current status of device state is published by the system_state in `device_state`
session, which is defined by `DeviceState`:

    uint8 NOT_OPERATIONAL=0
    uint8 STARTUP=1
    uint8 OPERATIONAL=2
    uint8 SHUTDOWN=3

    uint8 value

when the device is started with `/mtms_device/start_device` request the response returns
`true` if startup was successful. Once the startup is finished the system state property
`device_state.value=DeviceState.OPERATIONAL`

when the device is stopped with `/mtms_device/stop_device` request the response returns
`true` if shutdown was successful.  Once the shutdown process is finished the system
state property `device_state.value=DeviceState.NOT_OPERATIONAL`

#### Session state management
Session state management is similar to device state management by having its own topics
for starting session with `/mtms_device/start_session` and stopping session
`/mtms_device/stop_session`.

In addition to modifying session state, when session is
started system state `time` (in seconds) property starts counting up from the moment
session was started. When session is stopped `time` is reset back to 0.

Current status of session state is published by the system_state in `session_state`
session, which is defined by `SessionState`:

    uint8 STOPPED=0
    uint8 STARTING=1
    uint8 STARTED=2
    uint8 STOPPING=3

    uint8 value

When the session starting is finished the system state property
`session_state.value=SessionState.STARTED` and the service request will return `true`,
likewise when session stop request is made and complete, the session state will be
`session_state.value=SessionState.STOPPED`

#### Charging
Charging works by requesting a service `/mtms_device/request_events` with a message of type `Charge`
in `charges` field. The message type `Charge` consists of the following fields:

    uint8 channel
    uint16 target_voltage # in volts
    EventInfo event_info

This event will start the charging process once the given execution condition in
`event_info` is met. EventInfo:

    uint16 id  # Contains information about the event.

    ExecutionCondition execution_condition  # The condition on which the event will be executed; see ExecutionCondition.msg.

    float64 execution_time  # Time in seconds when the event will be executed.

    float64 decision_time   # Time in seconds when the decision on the event was made.


When the execution condition is met the charging process of a given `channel` to the
`target_voltage` will start. Once the charging process is finished or disrupted by some
error a message to the topic `/event/charge_feedback` is sent, containing the event id,
given in the charge message and possible error raised.

    uint16 id
    ChargeError error

#### Discharging
The discharging process is similar to the charging process by having the service
`mtms_device/request_events` for updating the channel voltage and will return
feedback message `/event/discharge_feedback` once complete or disrupted by error.

#### Device configuration and settings
The device needs to have setting sent to it with topic `/mtms_device/send_settings` that
control the pulse durations and frequencies.

For the device to be able to give pulses, the stimulation has to be allowed. This is
controlled with topic `/mtms_device/allow_stimulation`.

#### Pulses
Pulses, also work similarly to the charging and discharging processes in the regard that
pulses are requested in service `/mtms_device/request_events` and once finished or disrupted
by error return with `/event/pulse_feedback`.

    uint8 channel
    WaveformPiece[] waveform
    EventInfo event_info

The pulse is given to certain channel, with given waveform and event_info property that
works similarly to the charging and discharging properties.

#### Trigger out
Trigger out messages work also similarly to the charging and once the trigger out message
is completed message to the topic `/event/trigger_out_feedback` is sent:

    uint8 port  # The index of the signal port.
    uint32 duration_us  # Duration of the pulse in microseconds.
    EventInfo event_info

The trigger out message will send a trigger to physical `port` with given `duration_us`
when the `event_info` condition it met.

#### FPGA status
The FPGA status is published to the topic `/node/message`, which tells the current
status of the device in human-readable format.

## Interface specification
This section goes over the implementation details of different interface messages.

### Topic: `/mtms_device/allow_stimulation`
#### Service: `mtms_device_interfaces.srv.AllowStimulation`
QoS: ROS2 Default

Allow or disallow stimulation.
Response: Boolean indicating if the service call was successful.

    bool allow_stimulation
    ---
    bool success

### Topic: `/mtms_device/send_settings`
#### Service: `mtms_device_interfaces.srv.SendSettings`
QoS: ROS2 Default

    Settings settings
    ---
    bool success

### Topic: `/mtms_device/start_device`
#### Service: `mtms_device_interfaces.srv.StartDevice`
QoS: ROS2 Default

Start device. Response: Boolean indicating if starting was successful.


    ---
    bool success

### Topic: `/mtms_device/stop_device`
#### Service: `mtms_device_interfaces.srv.StopDevice`
QoS: ROS2 Default

Stop device. Response: Boolean indicating if stopping was successful.


    ---
    bool success

### Topic: `/mtms_device/request_events`
#### Service: `mtms_device_interfaces.srv.RequestEvents`
QoS: ROS2 Default

Request events. Response: Boolean indicating if request was successful.

    event_interfaces/Pulse[] pulses
    event_interfaces/Charge[] charges
    event_interfaces/Discharge[] discharges
    event_interfaces/TriggerOut[] trigger_outs
    ---
    bool success

## Publishers

### Topic: `/event/pulse_feedback`
#### Message: `event_interfaces.msg.PulseFeedback`
QoS: ROS2 Defaults with KEEP_LAST with depth 10

Contains feedback of an event

    uint16 id
    PulseError error

### Topic: `/event/charge_feedback`
#### Message: `event_interfaces.msg.ChargeFeedback`
QoS: ROS2 Defaults with KEEP_LAST with depth 10

Contains feedback of an event

    uint16 id
    ChargeError error

### Topic: `/event/discharge_feedback`
#### Message: `event_interfaces.msg.DisChargeFeedback`
QoS: ROS2 Defaults with KEEP_LAST with depth 10

Contains feedback of an event

uint16 id
DischargeError error

### Topic: `/event/trigger_out_feedback`
#### Message: `event_interfaces.msg.TriggerOutFeedback`
QoS: ROS2 Defaults with KEEP_LAST with depth 10

    uint8 port # The index of the signal port.
    uint32 duration_us # Duration of the pulse in microseconds.
    EventInfo event_info

### Topic: `/node/message`
#### Message: `std_msgs.msg.String`
QoS: ROS2 Defaults with KEEP_LAST with depth 10

Standard message string

### Topic: `/mtms_device/system_state`
#### Message: `mtms_device_interfaces.msg.SystemState
QoS:
- History: KEEP_LAST with depth 1.
- Reliability: RELIABLE,
- Durability: TRANSIENT_LOCAL
- Deadline: 25ms,
- Lifespan: 25ms


    ChannelState[] channel_states

    SystemError system_error_cumulative
    SystemError system_error_current
    SystemError system_error_emergency

    StartupError startup_error

    DeviceState device_state
