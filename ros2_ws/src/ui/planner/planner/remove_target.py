import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import RemoveTarget

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


class RemoveTargetNode(StateNode):

    def __init__(self):
        super().__init__('remove_target')
        self.create_service(RemoveTarget, '/planner/remove_target', self.remove_target_callback)

    def remove_target_callback(self, request, response):
        self.get_logger().info(f'Removing target {request.name}')

        state = self._state
        if state is None:
            response.success = False
            return response

        index_of_removed_target = drop_unique(state.targets, lambda target: target.name == request.name)

        for pulse_sequence in state.pulse_sequences:
            pulses_to_keep = list(
                filter(lambda pulse: pulse.target_index != index_of_removed_target, pulse_sequence.pulses))

            for pulse in pulses_to_keep:
                new_index = pulse.target_index if pulse.target_index < index_of_removed_target else pulse.target_index - 1
                pulse.target_index = new_index

            pulse_sequence.pulses = pulses_to_keep

        state.pulse_sequences = list(filter(lambda seq: len(seq.pulses) > 0, state.pulse_sequences))

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    remove_target_node = RemoveTargetNode()

    rclpy.spin(remove_target_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
