# EEG and EMG data in microvolts

float64[] eeg_data
float64[] emg_data
float64 time
# Should 'index' field be here also, as it is in Sample.msg?
bool valid

# The second trigger-in port of the EEG device (port B) is used for triggers
# that indicate simultaneous pulses (at least in case of conventional
# TMS devices for measuring the timing error - with mTMS device, it can be
# used more flexibly). Use this field to indicate that case.
#
# TODO: Naming could be clearer. Maybe 'is_stimulation_trigger'? The problem is
#   that it does not necessarily indicate simultaneous stimulation in case of
#   mTMS device.
bool pulse_delivered

bool is_event
string event_type

PreprocessedSampleMetadata metadata
