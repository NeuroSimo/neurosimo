import rclpy

from ui_interfaces.msg import PlannerState, PulseSequence
from ui_interfaces.srv import AddPulseSequence

from .state_node import StateNode


class AddPulseSequenceNode(StateNode):

    def __init__(self):
        super().__init__('add_pulse_sequence')
        self.create_service(AddPulseSequence, '/planner/add_pulse_sequence', self.add_pulse_sequence_callback)

    def first_available_pulse_sequence_name(self):
        if self._state is None:
            return "Sequence-0"

        pulse_sequence_names = [pulse_sequence.name for pulse_sequence in self._state.pulse_sequences]
        idx = 0
        while True:
            pulse_sequence_name = "Sequence-{}".format(idx)
            if pulse_sequence_name not in pulse_sequence_names:
                break
            idx += 1
        return pulse_sequence_name

    def create_new_pulse_sequence(self, pulses):
        pulse_sequence = PulseSequence()

        pulse_sequence.name = self.first_available_pulse_sequence_name()
        pulse_sequence.comment = ""
        pulse_sequence.visible = True
        pulse_sequence.selected = False
        pulse_sequence.isi = 100
        pulse_sequence.nof_pulses = len(pulses)
        pulse_sequence.pulses = pulses
        pulse_sequence.intensity = 100.0
        pulse_sequence.channel_info = []

        return pulse_sequence

    def add_pulse_sequence_callback(self, request, response):
        self.get_logger().info('Incoming request')

        pulse_sequence = self.create_new_pulse_sequence(
            pulses=request.pulses
        )

        if self._state is None:
            msg = PlannerState()
            msg.pulse_sequences = [
                pulse_sequence
            ]
        else:
            msg = self._state
            if not msg.pulse_sequences:
                msg.pulse_sequences = []

            msg.pulse_sequences.append(pulse_sequence)

        self._state_publisher.publish(msg)

        response.success = True
        return response


def main():
    rclpy.init()

    add_pulse_sequence_node = AddPulseSequenceNode()

    rclpy.spin(add_pulse_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
