from psychopy import visual, core
import numpy as np


class Presenter:
    def __init__(self):
        self.win = visual.Window(size=[2560, 1440], pos=[2560, 0], allowGUI=False)

    def __del__(self):
        self.win.close()

    def process(self, stimulus_type, parameters):
        """Process a sensory stimulus sent from the decider."""
        print(f"Received stimulus: {stimulus_type}")
        print(f"Parameters: {parameters}")
        
        if stimulus_type == 'visual_cue':
            # Extract parameters with defaults
            color = parameters.get('color', 'white')
            size = parameters.get('size', 50)
            duration = parameters.get('duration', 1.0)
            pos_x = parameters.get('position_x', 0)
            pos_y = parameters.get('position_y', 0)
            
            # Create and display visual stimulus
            stim = visual.Circle(
                win=self.win,
                radius=size,
                fillColor=color,
                pos=[pos_x, pos_y]
            )
            
            stim.draw()
            self.win.flip()
            core.wait(duration)
            self.win.flip()  # Clear screen
            
            return True
            
        elif stimulus_type == 'text_message':
            message = parameters.get('text', 'Default message')
            duration = parameters.get('duration', 2.0)
            
            text_stim = visual.TextStim(
                win=self.win,
                text=message,
                height=50
            )
            
            text_stim.draw()
            self.win.flip()
            core.wait(duration)
            self.win.flip()  # Clear screen
            
            return True
            
        else:
            print(f"Unknown stimulus type: {stimulus_type}")
            return False
