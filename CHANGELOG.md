# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] – 2025-12-04

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

[Unreleased]: https://github.com/your-org/neurosimo/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/your-org/neurosimo/releases/tag/v0.1.0
