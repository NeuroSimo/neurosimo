import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Pulse
from ui_interfaces.srv import SetPulseIsi

from .state_node import StateNode


class SetPulseIsiNode(StateNode):

    def __init__(self):
        super().__init__('set_pulse_isi')
        self.create_service(SetPulseIsi, '/planner/set_pulse_isi',
                            self.set_pulse_isi_callback)

    def set_pulse_isi_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info(
            f"Setting pulse sequence {request.name} pulse {request.pulse_index} isi to {request.new_isi}")

        found = False
        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name and request.pulse_index < len(pulse_sequence.pulses):
                pulse_sequence.pulses[request.pulse_index].isi = request.new_isi
                found = True

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    set_pulse_isi_node = SetPulseIsiNode()

    rclpy.spin(set_pulse_isi_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
