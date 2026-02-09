import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.srv import (
    ListProjects,
    SetActiveProject,
)

from std_msgs.msg import String

from .global_storage_manager import GlobalStorageManager


class GlobalConfiguratorNode(Node):
    def __init__(self):
        super().__init__('global_configurator')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Initialize global storage manager
        self.storage_manager = GlobalStorageManager(self.logger)

        # Declare ROS2 parameters
        # Project parameter
        active_project = self.storage_manager.get_active_project()
        self.declare_parameter('active_project', active_project)

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetActiveProject, '/projects/active/set', self.set_active_project_callback, callback_group=self.callback_group)

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos, callback_group=self.callback_group)

        # Set active project
        self.set_active_project(active_project)

    def set_active_project(self, project_name):
        if not project_name in self.storage_manager.list_projects():
            self.logger.error(f"Project does not exist: {project_name}")
            return False

        # Store the active project in the project state if it has changed
        if project_name != self.storage_manager.get_active_project():
            self.storage_manager.save_active_project(project_name)

        # Update ROS parameter
        self.set_parameters([
            rclpy.parameter.Parameter('active_project', rclpy.parameter.Parameter.Type.STRING, project_name)
        ])

        # Publish active project
        msg = String(data=project_name)
        self.active_project_publisher.publish(msg)

        return True

    # Service callbacks

    def list_projects_callback(self, request, response):
        try:
            response.projects = self.storage_manager.list_projects()
            response.success = True
            self.logger.info("Projects successfully listed.")
        except Exception as e:
            self.logger.error(f"Error listing projects: {e}")
            response.success = False
        return response

    def set_active_project_callback(self, request, response):
        project = request.project

        success = self.set_active_project(project)
        if not success:
            response.success = False
            return response

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GlobalConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()