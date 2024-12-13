import rclpy

from ui_interfaces.srv import ToggleSelectPulse

from .state_node import StateNode


class ToggleSelectPulseNode(StateNode):

    def __init__(self):
        super().__init__('toggle_select_pulse')
        self.create_service(ToggleSelectPulse, '/planner/toggle_select_pulse',
                            self.toggle_select_pulse_callback)

    def toggle_select_pulse_callback(self, request, response):
        self.get_logger().info(
            f"Toggling pulse sequence {request.name} pulse {request.pulse_index} selected")

        state = self._state
        if state is None:
            response.success = False
            return response

        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name and request.pulse_index < len(pulse_sequence.pulses):
                pulse_sequence.pulses[request.pulse_index].selected = not pulse_sequence.pulses[request.pulse_index].selected
        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_select_pulse_node = ToggleSelectPulseNode()

    rclpy.spin(toggle_select_pulse_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
