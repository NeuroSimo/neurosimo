# ============================================================================
# AttemptCommit.msg
# Self-timed message from ExperimentCoordinator to Decider, declaring the
# identity of the current attempt.
#
# Unlike stream-timed fields on Sample.msg, this message's meaning is not
# tied to a specific sample position - it simply asserts "as of when you
# receive this, the current attempt has this identity."
# ============================================================================

uint8[16] session_id
uint64 attempt_in_session       # Which attempt is being committed (join key)
