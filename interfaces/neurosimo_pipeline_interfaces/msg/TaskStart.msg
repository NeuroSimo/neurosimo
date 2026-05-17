# TaskStart.msg
# Sent from ExperimentCoordinator to Decider to start a named task
# (e.g., train_classifier). The decider runs process_task synchronously
# and replies with TaskFinished when done.

uint8[16] session_id
uint64 task_id              # Unique within session, assigned by coordinator
string task_name            # References decider's registered task handlers
