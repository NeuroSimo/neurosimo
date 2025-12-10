# Experiment Coordinator

The Experiment Coordinator enriches EEG samples with experiment state information based on protocol definitions.

## Overview

This node:
- Loads YAML protocol files defining experiment stages and rest periods
- Tracks experiment progression through stages and trials
- Enriches incoming EEG samples with experiment state metadata
- Handles pause/resume functionality
- Manages experiment timing (experiment time = sample time - pause duration)

## Topics

### Subscribed
- `/eeg/raw` (eeg_interfaces/Sample) - Raw EEG samples from EEG Bridge
- `/pipeline/pulse_events` (std_msgs/Empty) - Pulse events from Trigger Timer
- `/projects/active` (std_msgs/String) - Active project name

### Published
- `/eeg/enriched` (eeg_interfaces/Sample) - EEG samples enriched with experiment state
- `/experiment/coordinator/healthcheck` (system_interfaces/Healthcheck) - Health status
- `/experiment/protocol/list` (project_interfaces/ProtocolList) - Available protocols
- `/experiment/protocol` (std_msgs/String) - Currently loaded protocol

## Services

- `/experiment/protocol/set` (project_interfaces/SetProtocol) - Load a protocol
- `/experiment/pause` (std_srvs/Trigger) - Pause the experiment
- `/experiment/resume` (std_srvs/Trigger) - Resume the experiment

## Protocol Format

Protocols are defined in YAML files in the `protocols/` directory of each project. See `project_template/protocols/README.md` for details on the protocol format.

Example protocol structure:
```yaml
name: "My Protocol"
description: "Description"

stages:
  - stage:
      name: "baseline"
      trials: 20
  
  - rest:
      duration: 60.0
  
  - stage:
      name: "intervention"
      trials: 100
```

## Experiment State

The coordinator adds the following fields to EEG samples:

- `in_rest` - Whether currently in a rest period
- `paused` - Whether the experiment is paused
- `experiment_time` - Pause-adjusted time (sample.time - total_pause_duration)
- `trial` - Current trial within stage
- `stage_name` - Name of current stage
- `pulse_count` - Total number of pulses

## Behavior

### Stage Progression
- Each pulse event increments the trial counter
- When a stage's trial count is reached, it advances to the next element
- Stages track their start times for `wait_until` rest calculations

### Rest Periods
Two types of rest:
1. **Duration-based**: Rest for a fixed duration
2. **Wait-until**: Rest until a time relative to a previous stage start

### Pause/Resume
- Pausing stops experiment time progression
- Pause duration is calculated based on sample timestamps and subtracted from experiment time
- Experiment time = sample.time - total_pause_duration - current_pause (if paused)
- Pulse events during pause are ignored

## File Watching

The coordinator watches the protocols directory and:
- Reloads the current protocol if its file is modified
- Updates the protocol list when files are added/removed

## Launch

```bash
ros2 launch experiment_coordinator experiment_coordinator.launch.py
```

