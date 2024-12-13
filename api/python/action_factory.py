class ActionFactory:
    def create_pulse(self, x, y, angle, intensity):
        action = {
            'type': 'pulse',
            'params': {
                'x': x,
                'y': y,
                'angle': angle,
                'intensity': intensity,
            },
        }
        return action

    def create_trigger(self, port, delta_time=0.0):
        action = {
            'type': 'trigger',
            'params': {
                'port': port,
                'delta_time': delta_time,
            },
        }
        return action
