import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, PulseSequence
from ui_interfaces.srv import SetPulseSequenceIsi

from .state_node import StateNode


class SetPulseSequenceIsiNode(StateNode):

    def __init__(self):
        super().__init__('set_pulse_sequence_isi')
        self.create_service(SetPulseSequenceIsi, '/planner/set_pulse_sequence_isi',
                            self.set_pulse_sequence_isi_callback)

    def set_pulse_sequence_isi_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info(f"Setting pulse sequence {request.name} isi to {request.new_isi}")

        found = False
        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name:
                pulse_sequence.isi = request.new_isi
                for pulse in pulse_sequence.pulses:
                    pulse.isi = request.new_isi
                found = True

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    set_pulse_sequence_isi_node = SetPulseSequenceIsiNode()

    rclpy.spin(set_pulse_sequence_isi_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
