import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from ui_interfaces.msg import PlannerState, Target
from std_msgs.msg import Bool, Float64, String

from ui_interfaces.srv import ToggleSelectTarget

from .state_node import StateNode


class ToggleSelectTargetNode(StateNode):

    def __init__(self):
        super().__init__('toggle_select_target')
        self.create_service(ToggleSelectTarget, '/planner/toggle_select_target', self.toggle_select_target_callback)

    def toggle_select_target_callback(self, request, response):
        self.get_logger().info('Incoming request')

        state = self._state
        if state is None:
            response.success = False
            return response

        for target in state.targets:
            if target.name == request.name:
                target.selected = not target.selected

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_select_target_node = ToggleSelectTargetNode()

    rclpy.spin(toggle_select_target_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
