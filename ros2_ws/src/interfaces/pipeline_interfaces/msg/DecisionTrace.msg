# ============================================================================
# DecisionTrace.msg
# A low-rate trace event that is progressively enriched by multiple nodes.
# Time units:
# - sample_time / requested_* / scheduled_* / observed_* are seconds since
#   recording start.
# - system_time_* are uint64 nanoseconds from a monotonic clock.
# ============================================================================

# ----------------------------------------------------------------------------
# Metadata (filled by Decider)
# ----------------------------------------------------------------------------
uint8[16] session_id
uint64 decision_id                  # Resets per session

# ----------------------------------------------------------------------------
# Status (filled by each stage)
# ----------------------------------------------------------------------------
uint8 STATUS_DECIDED_NO=0           # decider decided not to stimulate
uint8 STATUS_DECIDED_YES=1          # decider decided to stimulate (request created)
uint8 STATUS_SCHEDULED=2            # trigger_timer accepted and scheduled
uint8 STATUS_REJECTED=3             # trigger_timer rejected request (e.g. safety/late)
uint8 STATUS_FIRED=4                # trigger_timer fired hardware trigger
uint8 STATUS_PULSE_OBSERVED=5             # pulse observed in EEG stream
uint8 STATUS_MISSED=6               # expected pulse not observed within window
uint8 STATUS_ERROR=7                # generic error
uint8 status

# ----------------------------------------------------------------------------
# Decision info (filled by Decider)
# ----------------------------------------------------------------------------
float64 reference_sample_time        # seconds since recording start
uint64 reference_sample_index

bool stimulate

# Requested stimulation time
float64 requested_stimulation_time   # seconds since recording start

# ----------------------------------------------------------------------------
# Decider / preprocessing timing (filled by Decider)
# ----------------------------------------------------------------------------
float64 decider_duration             # seconds
float64 preprocessor_duration        # seconds
float64 decision_path_latency        # seconds from sample publish to decider decision

uint64 system_time_decider_received  # steady clock ns
uint64 system_time_decider_finished  # steady clock ns

# ----------------------------------------------------------------------------
# TriggerTimer timing (filled by TriggerTimer)
# ----------------------------------------------------------------------------
uint64 system_time_trigger_timer_received
uint64 system_time_trigger_timer_finished

# When TriggerTimer actually fired the hardware output
uint64 system_time_hardware_fired    # steady clock ns

float64 sample_time_at_firing              # seconds since recording start
float64 loopback_latency_at_firing
float64 latency_corrected_time_at_firing

# ----------------------------------------------------------------------------
# Observed pulse (filled by StimulationTracer)
# ----------------------------------------------------------------------------
float64 actual_stimulation_time      # seconds since recording start
uint64 actual_stimulation_sample_index

# Timing error (actual - scheduled) in seconds
float64 timing_error

# ----------------------------------------------------------------------------
# Pulse confirmation (filled by StimulationTracer)
# ----------------------------------------------------------------------------
uint8 METHOD_NONE=0                 # No confirmation
uint8 METHOD_EEG_TRIGGER=1          # Confirmation for pulse received via incoming trigger (when using the EEG device)
uint8 METHOD_TRIGGER_TIMER_FIRE=2   # Confirmed via generating an outgoing trigger (when using the simulator)
uint8 pulse_confirmation_method

bool pulse_confirmed
