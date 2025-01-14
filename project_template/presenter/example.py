from psychopy import visual, core
import numpy as np


class Presenter:
    def __init__(self):
        self.win = visual.Window(size=[2560, 1440], pos=[2560, 0], allowGUI=False)

    def __del__(self):
        self.win.close()

    def process(self, state, parameter, duration):
        return True
