uint8 STOPPED = 0
uint8 INITIALIZING = 1
uint8 RUNNING = 2
uint8 FINALIZING = 3

uint8 state

# Optional reason for abort (shown when state is STOPPED after abort)
string abort_reason
