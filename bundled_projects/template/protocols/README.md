# Experiment Protocols

This directory contains YAML protocol files that define the structure of your experiments.

## Protocol Format

Each protocol file should be a YAML file with the following structure:

```yaml
name: "Protocol name"
description: "Description of the protocol"  # Optional

safety:
  minimum_trial_interval: 2.0   # minimum seconds between consecutive trials (required)

stages:
  - stage:
      name: "stage_name"
      trials: 100               # shorthand: 100 periodic trials
      max_failures: 10          # optional: cap invalid trials before stage ends
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

## Safety

The `safety` section is required and defines safety constraints for the protocol:

- `minimum_trial_interval`: Minimum time in seconds between consecutive trials/pulses. Must be positive. This value is enforced by both the decider (which gates periodic processing after a pulse) and the trigger timer (which rejects triggers that arrive too soon after the previous one).

## Elements

### Stage
A stage represents a period where stimuli are delivered. Each stage has:
- `name`: Unique identifier for the stage
- `trials`: Specifies the trials in the stage. Two forms are supported:
  - **Shorthand** — a plain integer, equivalent to a single periodic trial group:
    ```yaml
    trials: 100
    ```
  - **List form** — one or more trial groups, each with:
    - `count` (required): Number of trials in the group
    - `timing` (optional): `periodic` (default) or `predetermined`
    - `type` (optional): An arbitrary string identifying the trial type, passed to the decider's `process_predetermined` method (only meaningful for `predetermined` trials)
    ```yaml
    trials:
      - { count: 150 }                                           # periodic (default)
      - { count: 50, timing: predetermined, type: low_iti }      # predetermined
    ```
- `order` (optional): When using the list form, set to `"random"` to shuffle the individual trials across all groups randomly. The shuffle is seeded by the subject ID for reproducibility.
- `max_failures` (optional): Maximum number of invalid trials allowed in this stage before the stage ends early. Must be greater than `0`. When omitted, invalid trials can be retried without a failure cap (the stage still ends only after the required number of **valid** trials). Invalid trials are reported by the decider's `process_pulse()` return value (`trial_invalid: true`); they do not advance the stage trial counter.
- `notes`: Optional description

#### Invalid trials and stage completion

A trial counts as **valid** when the decider does not mark it invalid after pulse processing. A trial counts as **invalid** when `process_pulse()` returns `{'trial_invalid': True}`.

A stage completes when either:

1. The required number of **valid** trials has been reached (`trials`), or
2. `max_failures` is set and the number of invalid trials in the stage reaches that limit (the stage ends even if fewer than `trials` valid trials were collected).

Without `max_failures`, only condition (1) applies: invalid trials are retried until enough valid trials are recorded.

#### Trial timing modes

| Mode | How it works |
|------|-------------|
| `periodic` | The decider's `process_periodic` is called on every decider cycle. On each call it decides whether to trigger a pulse or not. |
| `predetermined` | The decider's `process_predetermined` is called once per trial with `(reference_time, stage_name, trial, trial_type)`. It returns the `trigger_offset` for that trial upfront, and the trigger is scheduled accordingly. |

For an example decider implementing `process_predetermined`, see `decider/example_predetermined.py`.

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
- The `safety` section is present with a positive `minimum_trial_interval`
- The protocol has at least one element
- All stage names are unique
- All `wait_until` anchors reference valid stage names
- All stages have at least 1 trial
- `max_failures`, when present, is greater than 0
- All durations are positive (> 0)
- All offsets are non-negative (>= 0)
- Each rest has either `duration` or `wait_until`, but not both

## Examples

- `example.yaml`: Simple protocol with a single stage
- `example_stages.yaml`: Complete protocol demonstrating stages, duration-based rest, and wait-until-based rest
- `example_predetermined.yaml`: Protocol mixing periodic and predetermined trial timings, with randomized order

