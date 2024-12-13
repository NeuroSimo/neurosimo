import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from std_msgs.msg import Bool, Float64, String
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from ui_interfaces.srv import ToggleNavigation

from .state_node import StateNode


class ToggleNavigationNode(StateNode):

    def __init__(self):
        super().__init__('toggle_navigation')
        self.create_service(ToggleNavigation, '/planner/toggle_navigation', self.toggle_navigation_callback)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._navigating_publisher = self.create_publisher(
            Bool,
            "/navigation/navigate",
            qos
        )
        self._navigating_subscriber = self.create_subscription(
            Bool,
            "/navigation/navigate",
            self.update_navigating,
            qos
        )
        self._navigating = None

    def update_navigating(self, navigating):
        self._navigating = navigating.data

    def toggle_navigation_callback(self, request, response):
        self.get_logger().info('Incoming request')

        state = self._state
        if state is None:
            response.success = False
            return response

        count = 0
        for target in state.targets:

            # Target-attribute of target is True
            if target.target:
                if count < 1:
                    count += 1
                else:
                    response.success = False
                    return response

        if count == 1:
            self.get_logger().info('Toggling navigating from {} to {}'.format(self._navigating, not self._navigating))

            msg = Bool()
            msg.data = not self._navigating

            self._navigating_publisher.publish(msg)

        response.success = True
        return response


def main():
    rclpy.init()

    toggle_navigation_node = ToggleNavigationNode()

    rclpy.spin(toggle_navigation_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
