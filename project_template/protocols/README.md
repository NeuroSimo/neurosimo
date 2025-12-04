# Experiment Protocols

This directory contains YAML protocol files that define the structure of your experiments.

## Protocol Format

Each protocol file should be a YAML file with the following structure:

```yaml
name: "Protocol name"
description: "Description of the protocol"

stages:
  - stage:
      name: "stage_name"
      trials: 100
      notes: "Optional notes about this stage"
  
  - rest:
      duration: 30.0  # Duration in seconds
      notes: "Optional notes"
  
  - rest:
      wait_until:
        anchor: "stage_name"  # Reference to a previous stage
        offset: 900.0  # Seconds after that stage started
      notes: "Optional notes"
```

## Elements

### Stage
A stage represents a period where stimuli are delivered. Each stage has:
- `name`: Unique identifier for the stage
- `trials`: Number of trials (pulse events) in this stage
- `notes`: Optional description

### Rest
A rest period where no stimuli are delivered. Can be defined in two ways:

1. **Duration-based**: Rest for a fixed duration
   ```yaml
   rest:
     duration: 60.0  # seconds
   ```

2. **Wait-until-based**: Rest until a specific time relative to a previous stage
   ```yaml
   rest:
     wait_until:
       anchor: "baseline"  # Name of a previous stage
       offset: 300.0  # Wait until 300s after that stage started
   ```

## Validation

The protocol loader will validate:
- All stage names are unique
- All `wait_until` anchors reference valid stage names
- All stages have at least 1 trial
- All durations and offsets are positive
- Each rest has either `duration` or `wait_until`, but not both

## Example

See `example.yaml` for a complete example protocol.

