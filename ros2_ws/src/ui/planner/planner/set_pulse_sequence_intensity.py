import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, PulseSequence
from ui_interfaces.srv import SetPulseSequenceIntensity

from .state_node import StateNode


class SetPulseSequenceIntensityNode(StateNode):

    def __init__(self):
        super().__init__('set_pulse_sequence_intensity')
        self.create_service(SetPulseSequenceIntensity, '/planner/set_pulse_sequence_intensity',
                            self.set_pulse_sequence_intensity_callback)

    def set_pulse_sequence_intensity_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info(f"Setting pulse sequence {request.name} intensity to {request.new_intensity}")

        found = False
        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name:
                pulse_sequence.intensity = request.new_intensity
                for pulse in pulse_sequence.pulses:
                    pulse.intensity = request.new_intensity
                found = True

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    set_pulse_sequence_intensity_node = SetPulseSequenceIntensityNode()

    rclpy.spin(set_pulse_sequence_intensity_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
