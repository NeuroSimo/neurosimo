## Version history

### 0.5.13
- MOD: Disable IGBT feedback check on Gen 2 devices due to issues with cross-channel interference.

### 0.5.12
- MOD: Increase the maximum number of pulse pieces from 16 to 17.

### 0.5.11
- MOD: Set minimum mode duration to 160 ticks for both Gen 1 and 2 devices.
- ADD: Event aggregation lock, allowing to aggregate events before performing them.

### 0.5.10
- MOD: Allow also last phase to be shorter than minimum duration if it is "Hold" phase.
- MOD: Set default maximum number of pulse pieces to 16.

### 0.5.9
- MOD: Change maximum number of pulse pieces to 16 to save space on FPGA.
- FIX: Maximum voltage in charging/discharging was not inclusive, thus excluding 1500 V.
- MOD: Allow first pulse phase to be shorter than minimum duration if it is "Hold" phase.

### 0.5.8
- ADD: Configurable number pulse of pieces with a maximum of 32.
- FIX: Attempt to fix occasionally missing discharge feedback message.

### 0.5.7
- MOD: Increase time before checking IGBT feedback to 200 ticks.

### 0.5.6
- MOD: Remove 20 ms update interval from session time.

### 0.5.5
- ADD: Send event time as feedback.

### 0.5.4
- ADD: 'Delay (ticks)' field to event info.
- ADD: Charging time to charge feedback.

### 0.5.3
- ADD: 'Allow trigger out' functionality, used to externally prevent trigger outs.

### 0.5.2
- FIX: Explicitly allow stimulation at the start of a session.

### 0.5.1
- FIX: Do not disallow stimulation at the start of a session.

### 0.5.0
- MOD: Change pulse polarity on Tubingen device to the original polarity (before fix in v0.4.6).
- MOD: Change 'Experiment' to 'Session' throughout the FPGA.
- ADD: Tools to use in installing the device
- DEL: Outdated installation tools
- ADD: 'Ready to stimulate' control, used to externally prevent pulse.

### 0.4.9
- FIX: Implementation of pulse got stuck because FIFO got full.
- MOD: Change SignalOut to TriggerOut.
- DEL: 'Disable checks' functionality, used for debugging.

### 0.4.8
- MOD: Change sync interval from 10 s to 1 s.
- FIX: Several charging-related bugs in Aalto device.
- ADD: Affine correction for voltage setpoint for charging in Aalto device.
- ADD: Linear correction for discharge controllers in Aalto device.

### 0.4.7
- FIX: Time counter was reset at the wrong time, before session started.

### 0.4.6
- MOD: Send sync out signal at periodic intervals.
- FIX: Pulse polarity on Tubingen device.

### 0.4.5
- FIX: Updating pulse count only worked for first channel.
- FIX: Default values for settings were in micro- or milliseconds, not ticks.

### 0.4.4
- MOD: Change time unit of 'Maximum pulses per time' setting to ticks
- MOD: Change time unit of 'Experiment startup trigger duration' setting to ticks
- MOD: Send time as ticks in event requests
- MOD: Clean up naming of front panel indicators

### 0.4.3
- MOD: Allow charging one channel while discharging another and vice versa.
- FIX: Charging and discharging checks did not work properly with 6-channel (Aalto) device.

### 0.4.2
- MOD: Change default setting for 'experiment start trigger duration' setting from 1 ms to 100 us.
- FIX: Validation of charge events when channel equals channel count.

### 0.4.1
- MOD: Only allow events on channels with coil memory attached.
- FIX: Event validation did not work for charge, discharge, and signal out event types.
- FIX: 'Board startup' error occasionally happening at startup (tentative fix).
- FIX: IGBT feedback check in startup - startup did not pass on Tubingen device.
- FIX: Discharge controller readings during and after shutdown - they showed nonsensical values occasionally.

### 0.4.0

- MOD: Change pulse final state and IGBT startup state to 'Hold' instead of 'Alternative hold'.
- ADD: Support Aalto target in the redesigned version of FPGA interface.
- MOD: Only update settings when experiment is started
- ADD: Send trigger out when experiment is started
- MOD: Drop voltages to zero when stopping an experiment to avoid 'carry-over' effect for the voltage.
- ADD: 'Experiment state' variable to front panel.
- FIX: Bug in which starting and stopping the device quickly in succession may not always work properly.
- FIX: Bug in which experiment could be quickly started and stopped in succession without an effect.
- MOD: Improve default settings.
