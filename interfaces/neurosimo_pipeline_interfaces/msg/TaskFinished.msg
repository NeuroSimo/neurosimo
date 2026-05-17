# TaskFinished.msg
# Sent from Decider to ExperimentCoordinator to signal that a task
# (started via TaskStart) has completed.

uint8[16] session_id
uint64 task_id              # Matches the TaskStart's task_id
