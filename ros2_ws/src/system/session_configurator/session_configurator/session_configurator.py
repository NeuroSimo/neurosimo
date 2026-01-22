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

from .project_manager import ProjectManager


class SessionConfiguratorNode(Node):
    def __init__(self):
        super().__init__('session_configurator')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Initialize project manager
        self.project_manager = ProjectManager(self.logger)

        # Declare ROS2 parameters
        self.declare_parameter('project', 'example')

        # Decider parameters
        self.declare_parameter('decider.module', 'example')
        self.declare_parameter('decider.enabled', False)

        # Preprocessor parameters
        self.declare_parameter('preprocessor.module', 'example')
        self.declare_parameter('preprocessor.enabled', False)

        # Presenter parameters
        self.declare_parameter('presenter.module', 'example')
        self.declare_parameter('presenter.enabled', False)

        # Experiment parameters
        self.declare_parameter('experiment.protocol', 'example')

        # Simulator parameters
        self.declare_parameter('simulator.dataset_filename', 'random_data_1_khz.json')
        self.declare_parameter('simulator.start_time', 0.0)

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetActiveProject, '/projects/active/set', self.set_active_project_callback, callback_group=self.callback_group)

        # Publisher
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos, callback_group=self.callback_group)

        # Set active project
        active_project = self.project_manager.get_active_project()
        self.set_active_project(active_project)

    def set_active_project(self, project_name):
        if not project_name in self.project_manager.list_projects():
            self.logger.error(f"Project does not exist: {project_name}")
            return False

        # Publish active project
        msg = String(data=project_name)
        self.active_project_publisher.publish(msg)

        # Load session state for the project
        session_state = self.project_manager.get_session_state_for_project(project_name)

        # Set ROS2 parameters from the loaded state
        self.set_parameters([
            rclpy.parameter.Parameter('project', rclpy.parameter.Parameter.Type.STRING, project_name),
            rclpy.parameter.Parameter('decider.module', rclpy.parameter.Parameter.Type.STRING, session_state["decider"]["module"]),
            rclpy.parameter.Parameter('decider.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["decider"]["enabled"]),
            rclpy.parameter.Parameter('preprocessor.module', rclpy.parameter.Parameter.Type.STRING, session_state["preprocessor"]["module"]),
            rclpy.parameter.Parameter('preprocessor.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["preprocessor"]["enabled"]),
            rclpy.parameter.Parameter('presenter.module', rclpy.parameter.Parameter.Type.STRING, session_state["presenter"]["module"]),
            rclpy.parameter.Parameter('presenter.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["presenter"]["enabled"]),
            rclpy.parameter.Parameter('experiment.protocol', rclpy.parameter.Parameter.Type.STRING, session_state["experiment"]["protocol"]),
            rclpy.parameter.Parameter('simulator.dataset_filename', rclpy.parameter.Parameter.Type.STRING, session_state["simulator"]["dataset_filename"]),
            rclpy.parameter.Parameter('simulator.start_time', rclpy.parameter.Parameter.Type.DOUBLE, session_state["simulator"]["start_time"]),
        ])
        return True

    # Service callbacks

    def list_projects_callback(self, request, response):
        try:
            response.projects = self.project_manager.list_projects()
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

        # Store the active project in the project state
        self.project_manager.save_active_project(project)

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SessionConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()