import rclpy
from ui_interfaces.srv import ChangePulseIndex

from .state_node import StateNode


class ChangePulseIndexNode(StateNode):

    def __init__(self):
        super().__init__('change_pulse_index')
        self.create_service(ChangePulseIndex, '/planner/change_pulse_index',
                            self.change_pulse_index_callback)

    def change_pulse_index_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        pulse_sequence = next((seq for seq in state.pulse_sequences if seq.name == request.name), None)

        if pulse_sequence is None or request.pulse_index >= len(pulse_sequence.pulses):
            response.success = False
            return response

        self.get_logger().info(f"Moving pulse sequence {request.name} pulse {request.pulse_index} to {request.new_index}")

        pulse = pulse_sequence.pulses[request.pulse_index]
        pulse_sequence.pulses.remove(pulse)
        pulse_sequence.pulses.insert(request.new_index, pulse)

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    change_pulse_index_node = ChangePulseIndexNode()

    rclpy.spin(change_pulse_index_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
