import rclpy

from ui_interfaces.srv import ToggleVisiblePulse

from .state_node import StateNode


class ToggleVisiblePulseNode(StateNode):

    def __init__(self):
        super().__init__('toggle_visible_pulse')
        self.create_service(ToggleVisiblePulse, '/planner/toggle_visible_pulse',
                            self.toggle_visible_pulse_callback)

    def toggle_visible_pulse_callback(self, request, response):
        self.get_logger().info(
            f"Toggling pulse sequence {request.name} pulse {request.pulse_index} visible")

        state = self._state
        if state is None:
            response.success = False
            return response

        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name and request.pulse_index < len(pulse_sequence.pulses):
                pulse_sequence.pulses[request.pulse_index].visible = not pulse_sequence.pulses[request.pulse_index].visible
        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_visible_pulse_node = ToggleVisiblePulseNode()

    rclpy.spin(toggle_visible_pulse_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
