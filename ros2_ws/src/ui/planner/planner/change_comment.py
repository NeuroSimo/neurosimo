import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import ChangeComment

from .state_node import StateNode


class ChangeCommentNode(StateNode):

    def __init__(self):
        super().__init__('change_comment')
        self.create_service(ChangeComment, '/planner/change_comment', self.change_comment_callback)

    def change_comment_callback(self, request, response):
        self.get_logger().info('Incoming request')

        state = self._state
        if state is None:
            response.success = False
            return response

        count = 0
        for target in state.targets:

            # Name exists
            if target.name == request.name:
                if count < 1:
                    target.comment = request.new_comment
                    count += 1

                else:
                    response.success = False
                    return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    change_comment_node = ChangeCommentNode()

    rclpy.spin(change_comment_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
