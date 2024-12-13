import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import ToggleVisible

from .state_node import StateNode


class ToggleVisibleNode(StateNode):

    def __init__(self):
        super().__init__('toggle_visible')
        self.create_service(ToggleVisible, '/planner/toggle_visible', self.toggle_visible_callback)

    def toggle_visible_callback(self, request, response):
        self.get_logger().info('Incoming request')

        state = self._state
        if state is None:
            response.success = False
            return response
        
        found = False
        for target in state.targets:
            if target.name == request.name:
                found = True
                target.visible = not target.visible

        if not found:
            response.success = False
            return response

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_visible_node = ToggleVisibleNode()

    rclpy.spin(toggle_visible_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
