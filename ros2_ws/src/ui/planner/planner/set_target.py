import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from ui_interfaces.msg import PlannerState, Target
from std_msgs.msg import Bool, Float64, String

from ui_interfaces.srv import SetTarget

from .state_node import StateNode


class SetTargetNode(StateNode):

    def __init__(self):
        super().__init__('set_target')
        self.create_service(SetTarget, '/planner/set_target', self.set_target_callback)

    def set_target_callback(self, request, response):
        self.get_logger().info('Incoming request: Set target to '.format(request.name))

        state = self._state
        if state is None:
            response.success = False
            return response

        for target in state.targets:
            if target.name == request.name:
                target.target = not target.target
            else:
                # Only allow one target at a time for now.
                target.target = False

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()
    set_target_node = SetTargetNode()
    rclpy.spin(set_target_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
