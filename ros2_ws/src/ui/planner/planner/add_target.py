import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from std_msgs.msg import Bool, Float64, String
from ui_interfaces.srv import AddTarget

from .state_node import StateNode

class AddTargetNode(StateNode):

    def __init__(self):
        super().__init__('add_target')
        self.create_service(AddTarget, '/planner/add_target', self.add_target_callback)

    def first_available_target_name(self):
        if self._state is None:
            return "Target-0"

        target_names = [target.name for target in self._state.targets]
        idx = 0
        while True:
            target_name = "Target-{}".format(idx)
            if target_name not in target_names:
                break
            idx += 1
        return target_name

    def create_new_target(self, pose):
        target = Target()

        target.name = self.first_available_target_name()
        target.type = "Target"
        target.comment = ""
        target.selected = False
        target.target = False  # XXX: Misnomer
        target.pose = pose

        target.intensity = 100.0
        target.iti = 100.0

        return target

    def add_target_callback(self, request, response):
        self.get_logger().info('Incoming request')

        target = self.create_new_target(
            pose=request.target  # XXX: Misnomer
        )

        if self._state is None:
            msg = PlannerState()
            msg.targets = [
                target
            ]
        else:
            msg = self._state
            msg.targets.append(target)

        self._state_publisher.publish(msg)

        response.success = True
        return response


def main():
    rclpy.init()

    add_target_node = AddTargetNode()

    rclpy.spin(add_target_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
