import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.srv import ListProjects
from system_interfaces.msg import GlobalConfig

from rcl_interfaces.msg import SetParametersResult

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

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.global_config_publisher = self.create_publisher(GlobalConfig, "/global_configurator/config", qos, callback_group=self.callback_group)

        # Add parameter change callback to save changes to global state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Publish initial global config
        global_config = {'active_project': active_project}
        self.publish_global_config(global_config)

    def set_active_project(self, project_name):
        """Set the active project and save to storage."""
        if not project_name in self.storage_manager.list_projects():
            self.logger.error(f"Project does not exist: {project_name}")
            return False

        # Store the active project in the project state if it has changed
        if project_name != self.storage_manager.get_active_project():
            self.storage_manager.save_active_project(project_name)
            self.logger.info(f"Updated global config with active_project={project_name}")

        return True
    
    def publish_global_config(self, global_config: dict):
        """Build and publish global config from dict."""
        config = GlobalConfig()
        config.active_project = global_config.get('active_project', '')
        self.global_config_publisher.publish(config)

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
    
    def parameter_change_callback(self, params):
        """Callback to handle global parameter changes and save them to global state."""
        try:
            # Load current global state
            global_config = {'active_project': self.storage_manager.get_active_project()}
            
            for param in params:
                if param.name == 'active_project':
                    project_name = param.value
                    
                    # Validate and save using set_active_project
                    success = self.set_active_project(project_name)
                    if not success:
                        return SetParametersResult(successful=False, reason=f"Project does not exist: {project_name}")
                    
                    # Update the global config dict
                    global_config[param.name] = param.value
            
            # Publish the updated global config immediately (from the dict, not from ROS params)
            self.publish_global_config(global_config)

        except Exception as e:
            self.logger.error(f"Error handling parameter changes: {e}")
            return SetParametersResult(successful=False, reason=str(e))

        # Return success to allow parameter changes
        return SetParametersResult(successful=True)

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