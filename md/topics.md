# NeuroSimo ROS Interface Inventory

This file lists the currently used ROS topics and services across NeuroSimo.

## Topics

### EEG Data and Device

- `/neurosimo/eeg/raw` - Raw EEG samples stream.
- `/neurosimo/eeg/enriched` - EEG stream with enrichment metadata.
- `/neurosimo/eeg/preprocessed` - Preprocessed EEG stream for pipeline modules.
- `/neurosimo/eeg/statistics` - EEG statistics for monitoring/UI.
- `/neurosimo/eeg_device/info` - EEG device capability/status info.
- `/neurosimo/eeg_bridge/state` - Bridge runtime state updates.
- `/neurosimo/eeg_bridge/heartbeat` - Bridge liveness heartbeat.
- `/neurosimo/eeg_bridge/health` - Bridge health/status details.
- `/neurosimo/eeg_simulator/state` - Simulator runtime state.
- `/neurosimo/eeg_simulator/heartbeat` - Simulator liveness heartbeat.
- `/neurosimo/eeg_simulator/health` - Simulator health/status details.
- `/neurosimo/eeg_replay/state` - Replay module runtime state.

### Session and System State

- `/neurosimo/session/state` - Overall session lifecycle state.
- `/neurosimo/session_recorder/state` - Recorder module state.
- `/neurosimo/session_exporter/state` - Exporter module state.
- `/neurosimo/system/disk_status` - Disk usage/status telemetry.
- `/neurosimo/resource_monitor/heartbeat` - Resource monitor liveness.
- `/neurosimo/resource_monitor/health` - Resource monitor health.

### Configuration and Catalog

- `/neurosimo/global_configurator/config` - Active global configuration.
- `/neurosimo/session_configurator/config` - Active session configuration.
- `/neurosimo/pipeline/decider/list` - Available decider modules.
- `/neurosimo/pipeline/preprocessor/list` - Available preprocessor modules.
- `/neurosimo/pipeline/presenter/list` - Available presenter modules.
- `/neurosimo/experiment/protocol/list` - Available experiment protocols.
- `/neurosimo/eeg_simulator/dataset/list` - Available simulator datasets.
- `/neurosimo/recording/recordings/list` - Available recordings list.

### Pipeline, Control, and Logs

- `/neurosimo/preprocessor/heartbeat` - Preprocessor liveness heartbeat.
- `/neurosimo/preprocessor/health` - Preprocessor health/status.
- `/neurosimo/presenter/heartbeat` - Presenter liveness heartbeat.
- `/neurosimo/presenter/health` - Presenter health/status.
- `/neurosimo/decider/heartbeat` - Decider liveness heartbeat.
- `/neurosimo/decider/health` - Decider health/status.
- `/neurosimo/decider/pulse_processed` - Pulse processing completion events.
- `/neurosimo/experiment_coordinator/heartbeat` - Coordinator liveness heartbeat.
- `/neurosimo/experiment_coordinator/health` - Coordinator health/status.
- `/neurosimo/trigger_timer/heartbeat` - Trigger timer liveness heartbeat.
- `/neurosimo/trigger_timer/health` - Trigger timer health/status.
- `/neurosimo/pipeline/decision_trace` - Detailed decision trace stream.
- `/neurosimo/pipeline/decision_trace/final` - Finalized decision trace output.
- `/neurosimo/pipeline/experiment_state` - Experiment runtime state.
- `/neurosimo/pipeline/sensory_stimulus` - Sensory stimulus events.
- `/neurosimo/pipeline/latency/loopback` - Loopback latency telemetry.
- `/neurosimo/pipeline/latency/pulse_processing` - Pulse processing latency telemetry.
- `/neurosimo/pipeline/latency/event_processing` - Event processing latency telemetry.
- `/neurosimo/pipeline/decider/log` - Decider log stream.
- `/neurosimo/pipeline/preprocessor/log` - Preprocessor log stream.
- `/neurosimo/pipeline/presenter/log` - Presenter log stream.

### Externally Wired Topics

- `/mtms/targeted_pulses` - Targeted pulses output for mTMS system.
- `/neuronavigation/coil_at_target` - Coil-at-target boolean/status.
- `/neuronavigation/coil_target` - Target location/pose information.

## Services

### Session Lifecycle and Export

- `/neurosimo/session/start` - Start a session.
- `/neurosimo/session/abort` - Abort an active session.
- `/neurosimo/session/finish` - Finish a session cleanly.
- `/neurosimo/session/export` - Start session export.
- `/neurosimo/session/cancel_export` - Cancel an in-progress export.
- `/neurosimo/session_recorder/start` - Start recorder.
- `/neurosimo/session_recorder/stop` - Stop recorder.

### Projects and Configuration

- `/neurosimo/projects/create` - Create a new project.
- `/neurosimo/projects/list` - List available projects.
- `/neurosimo/session_configurator/set_parameters` - Set session configurator parameters.
- `/neurosimo/session_configurator/get_parameters` - Get session configurator parameters.
- `/neurosimo/global_configurator/set_parameters` - Set global configurator parameters.
- `/neurosimo/global_configurator/get_parameters` - Get global configurator parameters.

### Pipeline Initialization and Runtime Control

- `/neurosimo/pipeline/protocol/initialize` - Initialize protocol module.
- `/neurosimo/pipeline/protocol/finalize` - Finalize protocol module.
- `/neurosimo/pipeline/preprocessor/initialize` - Initialize preprocessor module.
- `/neurosimo/pipeline/preprocessor/finalize` - Finalize preprocessor module.
- `/neurosimo/pipeline/decider/initialize` - Initialize decider module.
- `/neurosimo/pipeline/decider/finalize` - Finalize decider module.
- `/neurosimo/pipeline/presenter/initialize` - Initialize presenter module.
- `/neurosimo/pipeline/presenter/finalize` - Finalize presenter module.
- `/neurosimo/pipeline/stimulation_tracer/initialize` - Initialize stimulation tracer.
- `/neurosimo/pipeline/stimulation_tracer/finalize` - Finalize stimulation tracer.
- `/neurosimo/pipeline/trigger_timer/initialize` - Initialize trigger timer.
- `/neurosimo/pipeline/trigger_timer/finalize` - Finalize trigger timer.
- `/neurosimo/pipeline/timed_trigger` - Issue/schedule timed trigger.
- `/neurosimo/experiment/pause` - Pause an experiment.
- `/neurosimo/experiment/resume` - Resume a paused experiment.
- `/neurosimo/experiment_coordinator/protocol/get_info` - Get protocol metadata/info.

### EEG Stream Backends and Recording Access

- `/neurosimo/eeg_device/initialize` - Initialize EEG device stream backend.
- `/neurosimo/eeg_device/streaming/start` - Start device streaming.
- `/neurosimo/eeg_device/streaming/stop` - Stop device streaming.
- `/neurosimo/eeg_simulator/streaming/start` - Start simulator streaming.
- `/neurosimo/eeg_simulator/streaming/stop` - Stop simulator streaming.
- `/neurosimo/eeg_replay/initialize` - Initialize replay backend.
- `/neurosimo/eeg_replay/streaming/start` - Start replay streaming.
- `/neurosimo/eeg_replay/streaming/stop` - Stop replay streaming.
- `/neurosimo/eeg_simulator/dataset/get_info` - Get simulator dataset details.
- `/neurosimo/recording_manager/recording/get_info` - Get recording metadata/details.
- `/neurosimo/recording_manager/recording/delete` - Delete a recording.
