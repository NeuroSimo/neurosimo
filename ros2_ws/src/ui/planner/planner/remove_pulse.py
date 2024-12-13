import rclpy
from ui_interfaces.srv import RemovePulse

from .state_node import StateNode


# TODO: Move this from here to some utils-like module
#
def drop_unique(l, cond):
    """Given a list and a condition, drop the element for which the condition is True.
    Modify the list in place. Return the index of the dropped element.

    If the condition is True for none of the elements or more than one element, raise an error.

    Parameters
    ----------
    l
        The list of interest.
    cond
        A function that takes an element of the list and return a boolean.
    """
    index = None
    for i, element in enumerate(l):
        if cond(element):
            assert index is None, "Condition is True for more than one element"
            index = i
    assert index is not None, "Condition is True for none of the elements"
    del l[index]

    return index


class RemovePulseNode(StateNode):

    def __init__(self):
        super().__init__('remove_pulse')
        self.create_service(RemovePulse, '/planner/remove_pulse', self.remove_pulse_callback)

    def remove_pulse_callback(self, request, response):
        self.get_logger().info(f'Removing pulse {request.pulse_index} from {request.name}')

        state = self._state
        if state is None:
            response.success = False
            return response

        found = False
        for pulse_sequence in state.pulse_sequences:
            if pulse_sequence.name == request.name:
                if request.pulse_index < len(pulse_sequence.pulses):
                    if len(pulse_sequence.pulses) > 2:
                        found = True
                        del pulse_sequence.pulses[request.pulse_index]

                    break

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    remove_pulse_node = RemovePulseNode()

    rclpy.spin(remove_pulse_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
