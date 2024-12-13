import numpy as np

class GridFactory:
    def __init__(self, actions):
        
        self.actions = actions

    def to_coords(self, grid):
        grid = np.asarray(grid)

        coords = np.argwhere(grid) - (np.asarray(grid.shape) - 1) / 2
        return coords

    def to_params(self, grid):
        coords = self.to_coords(grid)
        params = [{
            'x': x,
            'y': y,
        } for x, y in coords]

        return params

    def to_pulses(self, grid, angle, intensity):
        pulses = [self.actions.create_pulse(
            params['x'],
            params['y'],
            angle,
            intensity,
        ) for params in self.to_params(grid)]

        return pulses
