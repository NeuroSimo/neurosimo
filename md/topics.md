# NeuroSimo ROS Interface Inventory

This file lists the currently used ROS topics and services across NeuroSimo.

## Topics

### EEG Data and Device

- `/eeg/raw` - Raw EEG samples stream.
- `/eeg/enriched` - EEG stream with enrichment metadata.
- `/eeg/preprocessed` - Preprocessed EEG stream for pipeline modules.
- `/eeg/statistics` - EEG statistics for monitoring/UI.
- `/eeg_device/info` - EEG device capability/status info.
- `/eeg_bridge/state` - Bridge runtime state updates.
- `/eeg_bridge/heartbeat` - Bridge liveness heartbeat.
- `/eeg_bridge/health` - Bridge health/status details.
- `/eeg_simulator/state` - Simulator runtime state.
- `/eeg_simulator/heartbeat` - Simulator liveness heartbeat.
- `/eeg_simulator/health` - Simulator health/status details.
- `/eeg_replay/state` - Replay module runtime state.

### Session and System State

- `/session/state` - Overall session lifecycle state.
- `/session_recorder/state` - Recorder module state.
- `/session_exporter/state` - Exporter module state.
- `/system/disk_status` - Disk usage/status telemetry.
- `/resource_monitor/heartbeat` - Resource monitor liveness.
- `/resource_monitor/health` - Resource monitor health.

### Configuration and Catalog

- `/global_configurator/config` - Active global configuration.
- `/session_configurator/config` - Active session configuration.
- `/pipeline/decider/list` - Available decider modules.
- `/pipeline/preprocessor/list` - Available preprocessor modules.
- `/pipeline/presenter/list` - Available presenter modules.
- `/experiment/protocol/list` - Available experiment protocols.
- `/eeg_simulator/dataset/list` - Available simulator datasets.
- `/recording/recordings/list` - Available recordings list.

### Pipeline, Control, and Logs

- `/preprocessor/heartbeat` - Preprocessor liveness heartbeat.
- `/preprocessor/health` - Preprocessor health/status.
- `/presenter/heartbeat` - Presenter liveness heartbeat.
- `/presenter/health` - Presenter health/status.
- `/decider/heartbeat` - Decider liveness heartbeat.
- `/decider/health` - Decider health/status.
- `/decider/pulse_processed` - Pulse processing completion events.
- `/experiment_coordinator/heartbeat` - Coordinator liveness heartbeat.
- `/experiment_coordinator/health` - Coordinator health/status.
- `/trigger_timer/heartbeat` - Trigger timer liveness heartbeat.
- `/trigger_timer/health` - Trigger timer health/status.
- `/pipeline/decision_trace` - Detailed decision trace stream.
- `/pipeline/decision_trace/final` - Finalized decision trace output.
- `/pipeline/experiment_state` - Experiment runtime state.
- `/pipeline/sensory_stimulus` - Sensory stimulus events.
- `/pipeline/targeted_pulses` - Decider targeted pulses output.
- `/pipeline/latency/loopback` - Loopback latency telemetry.
- `/pipeline/latency/pulse_processing` - Pulse processing latency telemetry.
- `/pipeline/latency/event_processing` - Event processing latency telemetry.
- `/pipeline/decider/log` - Decider log stream.
- `/pipeline/preprocessor/log` - Preprocessor log stream.
- `/pipeline/presenter/log` - Presenter log stream.

### Neuronavigation

- `/neuronavigation/coil_at_target` - Coil-at-target boolean/status.
- `/neuronavigation/coil_target` - Target location/pose information.

## Services

### Session Lifecycle and Export

- `/session/start` - Start a session.
- `/session/abort` - Abort an active session.
- `/session/finish` - Finish a session cleanly.
- `/session/export` - Start session export.
- `/session/cancel_export` - Cancel an in-progress export.
- `/session_recorder/start` - Start recorder.
- `/session_recorder/stop` - Stop recorder.

### Projects and Configuration

- `/projects/create` - Create a new project.
- `/projects/list` - List available projects.
- `/session_configurator/set_parameters` - Set session configurator parameters.
- `/session_configurator/get_parameters` - Get session configurator parameters.
- `/global_configurator/set_parameters` - Set global configurator parameters.
- `/global_configurator/get_parameters` - Get global configurator parameters.

### Pipeline Initialization and Runtime Control

- `/pipeline/protocol/initialize` - Initialize protocol module.
- `/pipeline/protocol/finalize` - Finalize protocol module.
- `/pipeline/preprocessor/initialize` - Initialize preprocessor module.
- `/pipeline/preprocessor/finalize` - Finalize preprocessor module.
- `/pipeline/decider/initialize` - Initialize decider module.
- `/pipeline/decider/finalize` - Finalize decider module.
- `/pipeline/presenter/initialize` - Initialize presenter module.
- `/pipeline/presenter/finalize` - Finalize presenter module.
- `/pipeline/stimulation_tracer/initialize` - Initialize stimulation tracer.
- `/pipeline/stimulation_tracer/finalize` - Finalize stimulation tracer.
- `/pipeline/trigger_timer/initialize` - Initialize trigger timer.
- `/pipeline/trigger_timer/finalize` - Finalize trigger timer.
- `/pipeline/timed_trigger` - Issue/schedule timed trigger.
- `/experiment/pause` - Pause an experiment.
- `/experiment/resume` - Resume a paused experiment.
- `/experiment_coordinator/protocol/get_info` - Get protocol metadata/info.

### EEG Stream Backends and Recording Access

- `/eeg_device/initialize` - Initialize EEG device stream backend.
- `/eeg_device/streaming/start` - Start device streaming.
- `/eeg_device/streaming/stop` - Stop device streaming.
- `/eeg_simulator/streaming/start` - Start simulator streaming.
- `/eeg_simulator/streaming/stop` - Stop simulator streaming.
- `/eeg_replay/initialize` - Initialize replay backend.
- `/eeg_replay/streaming/start` - Start replay streaming.
- `/eeg_replay/streaming/stop` - Stop replay streaming.
- `/eeg_simulator/dataset/get_info` - Get simulator dataset details.
- `/recording_manager/recording/get_info` - Get recording metadata/details.
- `/recording_manager/recording/delete` - Delete a recording.
