# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased
### Added
- Added session lifecycle management with explicit initialize/start/pause/stop/finalize flow and clearer session stage reporting in the UI.
- Added explicit pause/resume support for sessions.
- Added support for complex multi-stage protocols with rest periods.
- Added recording and replay workflow: listing recordings, viewing recording metadata, replaying data, and deleting recordings from the UI.
- Added export workflow for recordings in the UI (select data types for export and progress display).
- Added project creation and project-level configuration from the UI, including subject ID/notes fields and switching the active project.
- Added global configuration from the UI (for example timing/latency limits, dropped-sample tolerance, locale, and trigger-to-pulse delay).
- Added richer decision diagnostics in `DecisionTrace` and UI (status, decision-path latency, stimulation horizon, strict horizon, timing/loopback limits).
- Added component health and heartbeat reporting plus disk-space status display in the UI.
- Added session and module fingerprints to recording metadata and UI.
- Added experimental support for targeted pulses from decider.

### Changed
- Changed platform to ROS 2 Jazzy (from Iron) and Ubuntu 24.04 (from 22.04).
- Changed EEG simulator dataset format:
  - Dataset JSON now defines looping behavior and optional pulse times.
  - Event files are no longer supported by the EEG simulator.
  - Dataset CSV no longer includes a timestamp column.
- Changed decider and preprocessor configuration to express sample windows in seconds.
- Changed UI architecture and layout substantially: modularized major panels/providers, improved panel spacing/sizing, and added detached experiment-state view.
- Changed front-end packaging from a browser-hosted React UI to an Electron desktop app.
- Changed configuration model to rely more on ROS parameters and shared global configuration instead of environment variables.
- Changed terminology in several user-facing areas for consistency (for example playback -> recording, timing error -> timing offset).
- Decider API updates:
  - Pass stage name to `process_periodic`, `process_pulse`, and `process_event` methods.
  - Pass a boolean to `process_periodic` indicating whether the call is a warm-up call.

### Fixed
- Fixed crashes caused by Python interpreter state persisting across sessions when using complex Python libraries, by restarting Python-based pipeline nodes for each new session.
- Fixed multiple session lifecycle race conditions and edge cases (start/stop/finalization ordering, initialization failure handling, and aborted-session handling).
- Fixed reliability issues in Python pipeline logging, including dropped logs during bursts and improved log delivery at session end.
- Fixed robustness issues around pipeline node shutdown/restart between sessions, including decider/preprocessor/presenter restarts and hang recovery.
- Fixed several EEG replay and simulator streaming issues, including start/stop behavior and end-of-data handling.
- Fixed numerous UI regressions across configuration, health/status displays, overlays, modal interactions, and latency displays.

### Removed
- Removed legacy session manager/recorder/batcher/message types and outdated mTMS-era code paths that no longer match the current architecture.
- Removed outdated setup/runtime artifacts, including legacy docker-compose usage and obsolete installation/configuration traces.

## [0.2.0] – 2025-12-04

### Deprecated
- mTMS support is deprecated as of v0.2.0. Future versions will not maintain mTMS compatibility. Users requiring mTMS integration should fork from v0.2.0.

### Added
- Added pulse lockout duration to decider configuration
- Added `event_processors` and `stimulus_processors` mappings for custom event/stimulus handling.
- Added support for custom sample windows in event processors
- Added explicit `periodic_processing_enabled` flag and support for setting the first periodic processing time.
- Added example decider for sending sensory stimuli to the presenter.

### Fixed
- Fixed loss of log messages with bursts of print statements in Python pipeline modules
- Corrected handling of look-ahead windows in EEG buffers for preprocessor and decider.
- Processing events in decider no longer reset the periodic processing interval.
- Fixed crashing in preprocessor, decider, and presenter when the project directory is deleted.

### Changed
- Decider API updates:
  - Standardized and renamed several configuration parameters (`events` → `predefined_events`, `sensory_stimuli` → `predefined_sensory_stimuli`, etc.).
  - Dropped `process_on_trigger` parameter from configuration
  - Introduced mandatory `process_eeg_trigger` method for handling triggers from the EEG device.
  - Renamed `process` to `process_periodic`.
  - Parameter naming and ordering updated in decider methods.

- Preprocessor API updates:
  - Parameter naming and ordering updated in preprocessor methods.

- Periodic processing:
  - Interval is now specified in seconds (`periodic_processing_interval`), not samples.
  - Scheduling is now independent of EEG buffer size.
  - Processing precedence documented: events and EEG triggers take priority over periodic processing.

- Behavior changes:
  - Sessions now automatically stop when the EEG simulator reaches the dataset end (unless loop mode is enabled).
  - Updated parameters in examples (e.g., 0.1s interval in phastimate example, event timings adjusted).

## [0.1.0] - 2025-10-27

### Added
- Initial release of NeuroSimo

[Unreleased]: https://github.com/your-org/neurosimo/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/your-org/neurosimo/releases/tag/v0.2.0
[0.1.0]: https://github.com/your-org/neurosimo/releases/tag/v0.1.0
