import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Target
from ui_interfaces.srv import RenameTarget

from .state_node import StateNode


class RenameTargetNode(StateNode):

    def __init__(self):
        super().__init__('rename_target')
        self.create_service(RenameTarget, '/planner/rename_target', self.rename_target_callback)

    def rename_target_callback(self, request, response):

        state = self._state
        if state is None:
            response.success = False
            return response

        self.get_logger().info('Renaming {} to {}'.format(request.name, request.new_name))
        
        i = 0
        for target in state.targets:

            # Name already exists
            if target.name == request.new_name: 
                response.success = False
                return response
            
            # Save index of target in case new_name is unique
            if target.name == request.name:
                i = state.targets.index(target) 
        
        state.targets[i].name = request.new_name

        self._state_publisher.publish(state)

        response.success = True
        return response


def main():
    rclpy.init()

    rename_target_node = RenameTargetNode()

    rclpy.spin(rename_target_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
