# ============================================================================
# AttemptTrace.msg
# A trace event that tracks a single stimulation attempt through the pipeline.
# Progressively enriched by multiple nodes.
#
# Time units:
# - sample_time / requested_* / actual_* are seconds since recording start.
# - system_time_* are uint64 nanoseconds from a monotonic clock.
# ============================================================================

# ----------------------------------------------------------------------------
# Metadata (filled by Decider or scheduler)
# ----------------------------------------------------------------------------
uint8[16] session_id
uint64 attempt_in_session                         # Increments per attempt, resets per session

# ----------------------------------------------------------------------------
# Status (filled by each stage)
# ----------------------------------------------------------------------------
uint8 STATUS_SCHEDULED=1                  # trigger timer accepted and scheduled
uint8 STATUS_FIRED=2                      # trigger timer fired hardware trigger
uint8 STATUS_PULSE_PROCESSED=3            # pulse observed in EEG stream and corresponding decider processing completed
uint8 STATUS_LOOPBACK_LATENCY_EXCEEDED=4  # loopback latency exceeds threshold
uint8 STATUS_TOO_LATE=5                   # trigger time exceeded maximum timing offset
uint8 STATUS_ERROR=6                      # error (e.g. LabJack not connected, trigger already scheduled)
uint8 status

# ----------------------------------------------------------------------------
# Attempt information (filled by Decider)
# ----------------------------------------------------------------------------
uint8 ATTEMPT_TIMING_PERIODIC=1
uint8 ATTEMPT_TIMING_PREDETERMINED=2
uint8 attempt_timing              # How the attempt is triggered
string attempt_type               # Attempt type (e.g. "low_iti"), empty for periodic

uint64 decision_id                # Only set for periodic attempts

# ----------------------------------------------------------------------------
# Requested stimulation (filled by Decider)
# ----------------------------------------------------------------------------
float64 requested_stimulation_time        # seconds since recording start

# ----------------------------------------------------------------------------
# Timing reference (filled by Decider)
# ----------------------------------------------------------------------------
float64 reference_time                    # The reference time used for this attempt's pulse (seconds since recording start)

# ----------------------------------------------------------------------------
# TriggerTimer configuration (filled by TriggerTimer)
# ----------------------------------------------------------------------------
float64 maximum_timing_offset             # in seconds
float64 maximum_loopback_latency          # in seconds
float64 trigger_to_pulse_delay            # in seconds

# ----------------------------------------------------------------------------
# TriggerTimer timing (filled by TriggerTimer)
# ----------------------------------------------------------------------------
uint64 system_time_trigger_timer_received
uint64 system_time_trigger_timer_finished

# When TriggerTimer actually fired the hardware output
uint64 system_time_hardware_fired         # steady clock ns

float64 loopback_latency_at_scheduling
float64 latency_corrected_time_at_firing

# ----------------------------------------------------------------------------
# Observed pulse (filled by StimulationTracer)
# ----------------------------------------------------------------------------
float64 actual_stimulation_time           # seconds since recording start
uint64 actual_stimulation_sample_index

# Timing offset (actual - scheduled) in seconds
float64 timing_offset

# ----------------------------------------------------------------------------
# Trial validity (filled by Decider from process_pulse return value)
# ----------------------------------------------------------------------------
bool invalid_trial                       # Whether the trial produced invalid data
