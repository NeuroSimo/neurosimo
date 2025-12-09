# The timing error (= the error between expected and actual stimulation time relative to the
# EEG device's clock). It is updated each time TMS device generates a pulse and a simultaneous
# trigger, and the trigger is timestamped by the EEG device and reaches back to the decider.
float64 error
