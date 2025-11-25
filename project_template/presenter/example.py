from typing import Any, Dict
from psychopy import visual, core
import numpy as np


class Presenter:
    def __init__(self) -> None:
        self.win = visual.Window(size=[400, 400], pos=[0, 0], allowGUI=False, units='norm')
        print("Presenter initialized")

    def __del__(self) -> None:
        self.win.close()

    def get_configuration(self) -> Dict[str, Any]:
        """Return configuration dictionary for the presenter."""
        return {
            'stimulus_processors': {
                'visual_cue': self.process_visual_cue,
                'text_message': self.process_text_message,
            }
        }

    def process_visual_cue(self, parameters: Dict[str, Any]) -> bool:
        """Process a visual cue stimulus."""
        print(f"Processing visual_cue")
        print(f"Parameters: {parameters}")
        
        # Extract parameters with defaults
        color = parameters.get('color', 'white')
        size = parameters.get('size', 0.1)
        duration = parameters.get('duration', 1.0)
        pos_x = parameters.get('position_x', 0.0)
        pos_y = parameters.get('position_y', 0.0)
        
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

    def process_text_message(self, parameters: Dict[str, Any]) -> bool:
        """Process a text message stimulus."""
        print(f"Processing text_message")
        print(f"Parameters: {parameters}")
        
        message = parameters.get('text', 'Default message')
        duration = parameters.get('duration', 2.0)
        
        text_stim = visual.TextStim(
            win=self.win,
            text=message,
        )
        
        text_stim.draw()
        self.win.flip()
        core.wait(duration)
        self.win.flip()  # Clear screen
        
        return True
