# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Added pulse lockout duration to decider configuration
- Added `event_handlers` parameter to decider configuration: a dictionary from event types to handler functions
- Added support for custom sample windows for event handlers
- Allow setting the first periodic processing time in decider configuration, with convenient default behavior
- Added `periodic_processing_enabled` parameter to decider configuration, making it explicit whether periodic processing is enabled
- Added separate example decider for sending sensory stimuli to the presenter

### Fixed
- Fixed log message loss when publishing bursts of messages from Python pipeline modules by batching related messages together
- Fixed preprocessor and decider to correctly handle look-ahead windows in EEG buffers
- Events that trigger processing in decider now do not reset the processing interval
- Fixed crash in preprocessor, decider, and presenter when project directory is deleted while they are running

### Changed
- Automatically stop session when EEG simulator reaches end of dataset without loop mode
- Decreased processing interval to 0.1 seconds in phastimate example, with 2 second pulse lockout duration
- Periodic processing interval is now specified in seconds in decider configuration instead of samples, and the parameter name changed from `processing_interval_in_samples` to `periodic_processing_interval`
- Decider module now requires a `process_eeg_trigger` method, which is called when the EEG device receives a trigger
- Renamed `events` parameter to `predefined_events` in decider configuration and made it optional
- Renamed `sensory_stimuli` parameter to `predefined_sensory_stimuli` in decider configuration and made it optional
- Dropped `process_on_trigger` parameter from decider configuration: decider now always triggers processing if the sample includes an EEG trigger
- Made periodic processing scheduling independent of the buffer size in decider

## [0.1.0] - 2025-10-27

### Added
- Initial release of NeuroSimo

[Unreleased]: https://github.com/your-org/neurosimo/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/your-org/neurosimo/releases/tag/v0.1.0
