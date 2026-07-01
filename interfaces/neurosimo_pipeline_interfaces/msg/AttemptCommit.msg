# ============================================================================
# AttemptCommit.msg
# Self-timed message from ExperimentCoordinator to Decider, declaring the
# identity and timing anchor of the current attempt.
#
# Unlike stream-timed fields on Sample.msg, this message's meaning is not
# tied to a specific sample position - it simply asserts "as of when you
# receive this, the current attempt has this identity and starts at this time."
# ============================================================================

uint8[16] session_id
uint64 attempt_in_session       # Which attempt is being committed
float64 attempt_start_time      # Timing anchor for the attempt (seconds since recording start).
                                # Session/stage start, the observed pulse, rest end, or resume.

string stage_name               # Current stage name from protocol
uint32 trial_in_stage           # Current trial slot within stage (0-based), advances on success
uint64 trial_in_session         # Total successful trials in session
