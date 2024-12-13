import rclpy
from ui_interfaces.srv import SetTargetOrientation

from .state_node import StateNode


class SetTargetOrientationNode(StateNode):

    def __init__(self):
        super().__init__('set_target_orientation')
        self.create_service(SetTargetOrientation, '/planner/set_target_orientation',
                            self.set_target_orientation_callback)

    def set_target_orientation_callback(self, request, response):
        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info(f'Setting target {request.target_id} orientation to {request.orientation}')

        # not found
        if request.target_id >= len(state.targets):
            response.success = False
            return response

        target = state.targets[request.target_id]
        target.pose.orientation = request.orientation

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    set_target_orientation_node = SetTargetOrientationNode()

    rclpy.spin(set_target_orientation_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
