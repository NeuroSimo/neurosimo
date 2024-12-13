import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Pulse
from ui_interfaces.srv import SetPulseIntensity

from .state_node import StateNode


class SetPulseIntensityNode(StateNode):

    def __init__(self):
        super().__init__('set_pulse_intensity')
        self.create_service(SetPulseIntensity, '/planner/set_pulse_intensity',
                            self.set_pulse_intensity_callback)

    def set_pulse_intensity_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info(
            f"Setting pulse sequence {request.name} pulse {request.pulse_index} intensity to {request.new_intensity}")

        found = False
        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name and request.pulse_index < len(pulse_sequence.pulses):
                pulse_sequence.pulses[request.pulse_index].intensity = request.new_intensity
                found = True

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    set_pulse_intensity_node = SetPulseIntensityNode()

    rclpy.spin(set_pulse_intensity_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
