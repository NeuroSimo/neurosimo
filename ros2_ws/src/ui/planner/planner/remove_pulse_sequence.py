import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, PulseSequence
from ui_interfaces.srv import RemovePulseSequence

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


class RemovePulseSequenceNode(StateNode):

    def __init__(self):
        super().__init__('remove_pulse_sequence')
        self.create_service(RemovePulseSequence, '/planner/remove_pulse_sequence', self.remove_pulse_sequence_callback)

    def remove_pulse_sequence_callback(self, request, response):
        self.get_logger().info(f'Removing pulse sequence {request.name}')

        state = self._state
        if state is None:
            response.success = False
            return response

        idx = drop_unique(state.pulse_sequences, lambda pulse_sequence: pulse_sequence.name == request.name)

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    remove_pulse_sequence_node = RemovePulseSequenceNode()

    rclpy.spin(remove_pulse_sequence_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
