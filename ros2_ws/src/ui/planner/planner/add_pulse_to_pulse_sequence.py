import rclpy

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import AddPulseToPulseSequence

from .state_node import StateNode


class AddPulseToPulseSequenceNode(StateNode):

    def __init__(self):
        super().__init__('add_target')
        self.create_service(AddPulseToPulseSequence, '/planner/add_pulse_to_pulse_sequence',
                            self.add_pulse_to_pulse_sequence_callback)

    def add_pulse_to_pulse_sequence_callback(self, request, response):
        self.get_logger().info(f"Adding pulse to {request.name}")

        state = self._state

        if state is None:
            response.success = False
            return response

        pulse_sequence = next((seq for seq in state.pulse_sequences if seq.name == request.name), None)
        if pulse_sequence is None:
            response.success = False
            return response

        pulse_sequence.pulses.append(request.pulse)

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    add_pulse_to_pulse_sequence_node = AddPulseToPulseSequenceNode()

    rclpy.spin(add_pulse_to_pulse_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
