import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import ClearState

from .state_node import StateNode


class ClearStateNode(StateNode):

    def __init__(self):
        super().__init__('clear_state')
        self.create_service(ClearState, '/planner/clear_state', self.clear_state_callback)

    def clear_state_callback(self, request, response):
        self.get_logger().info('Clearing state')

        state = self._state
        if state is None:
            response.success = True
            return response

        msg = PlannerState()

        self._state_publisher.publish(msg)

        response.success = True
        return response


def main():
    rclpy.init()

    clear_state_node = ClearStateNode()
    rclpy.spin(clear_state_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
