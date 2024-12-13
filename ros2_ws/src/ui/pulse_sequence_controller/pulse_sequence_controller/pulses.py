from event_interfaces.msg import WaveformPiece

fullEvent = {
    'pulse': {
        'waveform': [
            {'waveform_phase': 4, 'duration_in_ticks': 10000},
            {'waveform_phase': 3, 'duration_in_ticks': 30000},
            {'waveform_phase': 2, 'duration_in_ticks': 50000}
        ],
        'channel': 2,
        'event_info': {
            'id': 20,
            # TODO: Bitrotten: 'wait_for_trigger' has been replaced with 'execution_condition'.
            'wait_for_trigger': True,
            'execution_time': 10.0,
        }
    }
}

_pulse = [
    {'waveform_phase': 4, 'duration_in_ticks': 10000},
    {'waveform_phase': 3, 'duration_in_ticks': 30000},
    {'waveform_phase': 2, 'duration_in_ticks': 50000}
]

pulses = {}
TICK_DURATION_IN_US = 25


def waveform_piece_from_dict(pulse_dict):
    piece = WaveformPiece()
    piece.waveform_phase.value = pulse_dict['waveform_phase']
    piece.duration_in_ticks = pulse_dict['duration_in_ticks']
    return piece


def generate_pulse(channel, voltage):
    return pulses[channel]


def pulse_duration_in_us(pulse):
    duration = 0
    for piece in pulse:
        duration += piece.duration_in_ticks * TICK_DURATION_IN_US
    return duration / 1000


for index in range(0, 6):
    waveform = list(map(lambda piece: waveform_piece_from_dict(piece), _pulse))
    pulses[index + 1] = waveform
