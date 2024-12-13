import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, PulseSequence
from ui_interfaces.srv import RenamePulseSequence

from .state_node import StateNode


class RenamePulseSequenceNode(StateNode):

    def __init__(self):
        super().__init__('rename_pulse_sequence')
        self.create_service(RenamePulseSequence, '/planner/rename_pulse_sequence', self.rename_pulse_sequence_callback)

    def rename_pulse_sequence_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info('Renaming {} to {}'.format(request.name, request.new_name))

        i = 0
        for pulse_sequence in state.pulse_sequences:

            # Name already exists
            if pulse_sequence.name == request.new_name:
                response.success = False
                return response

            # Save index of pulse_sequence in case new_name is unique
            if pulse_sequence.name == request.name:
                i = state.pulse_sequences.index(pulse_sequence)

        state.pulse_sequences[i].name = request.new_name

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    rename_pulse_sequence_node = RenamePulseSequenceNode()

    rclpy.spin(rename_pulse_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
