# ============================================================================
# AttemptCommit.msg
# Self-timed message from ExperimentCoordinator to Decider, declaring the
# identity (timing mode and type) of the current attempt.
#
# Unlike stream-timed fields on Sample.msg, this message's meaning is not
# tied to a specific sample position - it simply asserts "as of when you
# receive this, the current attempt has this identity."
# ============================================================================

uint64 attempt_in_session       # Which attempt is being committed (join key)

uint8 TRIAL_TIMING_PERIODIC=1
uint8 TRIAL_TIMING_PREDETERMINED=2
uint8 trial_timing              # How the trial timing is determined

string trial_type               # Trial type (e.g. "low_iti"), empty for periodic
