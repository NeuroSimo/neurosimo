from psychopy import visual, core
import numpy as np


class Presenter:
    def __init__(self):
        self.win = visual.Window(size=[2560, 1440], pos=[2560, 0], allowGUI=False)

    def __del__(self):
        self.win.close()

    def process(self, stimulus_type, parameters):
        """Process a sensory stimulus sent from the decider.
        
        This method is called automatically by the pipeline when a sensory stimulus
        is received from the decider. The stimulus is parsed from the sensory_stimuli
        list defined in the decider's get_configuration() or returned from the 
        decider's process() method.
        
        Args:
            stimulus_type (str): The type of stimulus (from the 'type' field in the 
                                sensory stimulus dictionary).
            parameters (dict): A dictionary containing stimulus parameters (from the 
                              'parameters' field in the sensory stimulus dictionary).
                              Numeric values are automatically parsed back to int/float
                              when possible, otherwise they remain as strings.
        
        Returns:
            bool: True if the stimulus was successfully processed, False otherwise.
                  Returning False will log an error but won't stop the pipeline.
        
        Example stimulus from decider:
        {
            'time': 5.0,
            'type': 'visual_cue',
            'parameters': {
                'color': 'red',
                'size': 100,
                'duration': 0.5,
                'position_x': 0,
                'position_y': 0
            }
        }
        
        This would result in process() being called with:
        - stimulus_type = 'visual_cue'
        - parameters = {'color': 'red', 'size': 100, 'duration': 0.5, 'position_x': 0, 'position_y': 0}
        """
        
        print(f"Received stimulus: {stimulus_type}")
        print(f"Parameters: {parameters}")
        
        # Example: Handle different stimulus types
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
            
            # Display for specified duration
            core.wait(duration)
            
            # Clear screen
            self.win.flip()
            
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
            self.win.flip()
            
            return True
            
        else:
            print(f"Unknown stimulus type: {stimulus_type}")
            return False
