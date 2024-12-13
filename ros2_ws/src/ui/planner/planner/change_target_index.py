import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import ChangeTargetIndex

from .state_node import StateNode


class ChangeTargetIndexNode(StateNode):

    def __init__(self):
        super().__init__('change_target_index')
        self.create_service(ChangeTargetIndex, '/planner/change_target_index',
                            self.change_target_index_callback)

    def change_target_index_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        original_index = -1

        for i in range(len(state.targets)):
            if state.targets[i].name == request.name:
                original_index = i

        # If target with name was not found
        if original_index == -1:
            response.success = False
            return response

        self.get_logger().info(
            'Moving target {} from index {} to {}'.format(request.name, original_index, request.new_index))

        target = state.targets[original_index]

        state.targets.remove(target)
        state.targets.insert(request.new_index, target)

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    change_target_index_node = ChangeTargetIndexNode()

    rclpy.spin(change_target_index_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
