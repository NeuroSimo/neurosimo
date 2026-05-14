# ============================================================================
# DecisionTrace.msg
# A trace event for a single decision made by the Decider.
# Published for every decision cycle (both positive and negative).
# Time units:
# - sample_time / reference_* are seconds since recording start.
# - system_time_* are uint64 nanoseconds from a monotonic clock.
# ============================================================================

# ----------------------------------------------------------------------------
# Status
# ----------------------------------------------------------------------------
uint8 STATUS_DECIDED_NO=1                 # decider decided not to stimulate
uint8 STATUS_DECIDED_YES=2                # decider decided to stimulate (trial created)
uint8 status

# ----------------------------------------------------------------------------
# Decision info (filled by Decider)
# ----------------------------------------------------------------------------
uint64 decision_id                        # Increments every decision, resets per session
float64 reference_sample_time             # seconds since recording start
uint64 reference_sample_index

bool stimulate

# ----------------------------------------------------------------------------
# Decider / preprocessing timing (filled by Decider)
# ----------------------------------------------------------------------------
float64 decider_duration                  # seconds
float64 preprocessor_duration             # seconds
float64 decision_path_latency             # seconds from sample publish to decider decision

uint64 system_time_decider_received       # steady clock ns
uint64 system_time_decider_finished       # steady clock ns
