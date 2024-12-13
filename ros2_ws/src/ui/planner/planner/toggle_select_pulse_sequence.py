import rclpy

from ui_interfaces.srv import ToggleSelectPulseSequence

from .state_node import StateNode


class ToggleSelectPulseSequenceNode(StateNode):

    def __init__(self):
        super().__init__('toggle_select_pulse_sequence')
        self.create_service(ToggleSelectPulseSequence, '/planner/toggle_select_pulse_sequence',
                            self.toggle_select_pulse_sequence_callback)

    def toggle_select_pulse_sequence_callback(self, request, response):
        self.get_logger().info('Incoming request')

        state = self._state
        if state is None:
            response.success = False
            return response

        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name:
                pulse_sequence.selected = not pulse_sequence.selected
                self.get_logger().info('Toggled select for {}'.format(pulse_sequence.name))
        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_select_pulse_sequence_node = ToggleSelectPulseSequenceNode()

    rclpy.spin(toggle_select_pulse_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
