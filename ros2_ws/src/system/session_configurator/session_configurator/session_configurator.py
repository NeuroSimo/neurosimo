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
from rcl_interfaces.msg import SetParametersResult

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

        # Add parameter change callback to save changes to session state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

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
            rclpy.parameter.Parameter('decider.module', rclpy.parameter.Parameter.Type.STRING, session_state["decider.module"]),
            rclpy.parameter.Parameter('decider.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["decider.enabled"]),
            rclpy.parameter.Parameter('preprocessor.module', rclpy.parameter.Parameter.Type.STRING, session_state["preprocessor.module"]),
            rclpy.parameter.Parameter('preprocessor.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["preprocessor.enabled"]),
            rclpy.parameter.Parameter('presenter.module', rclpy.parameter.Parameter.Type.STRING, session_state["presenter.module"]),
            rclpy.parameter.Parameter('presenter.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["presenter.enabled"]),
            rclpy.parameter.Parameter('experiment.protocol', rclpy.parameter.Parameter.Type.STRING, session_state["experiment.protocol"]),
            rclpy.parameter.Parameter('simulator.dataset_filename', rclpy.parameter.Parameter.Type.STRING, session_state["simulator.dataset_filename"]),
            rclpy.parameter.Parameter('simulator.start_time', rclpy.parameter.Parameter.Type.DOUBLE, session_state["simulator.start_time"]),
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

    def parameter_change_callback(self, params):
        """Callback to handle parameter changes and save them to session state."""
        try:
            # Find the project parameter in params (by param.name)
            project_param = None
            for param in params:
                if param.name == 'project':
                    project_param = param
                    break

            if project_param is not None:
                active_project = self.project_manager.get_active_project()

                new_project = project_param.value
                if new_project != active_project:
                    self.logger.info(f"Project parameter changed from '{active_project}' to '{new_project}', switching projects")
                    success = self.set_active_project(new_project)
                    if not success:
                        self.logger.error(f"Failed to switch to project '{new_project}'")
                        return rclpy.parameter.SetParametersResult(successful=False, reason=f"Invalid project: {new_project}")

                    self.project_manager.save_active_project(new_project)

                params = [p for p in params if p.name != 'project']

            # Handle other parameter changes
            if params:
                active_project = self.project_manager.get_active_project()

                session_state = self.project_manager.load_session_state(active_project)

                for param in params:
                    session_state[param.name] = param.value

                # Save the updated session state
                self.project_manager.save_session_state(active_project, session_state)
                self.logger.info(f"Updated session state for project '{active_project}' with parameter changes")

        except Exception as e:
            self.logger.error(f"Error handling parameter changes: {e}")
            return SetParametersResult(successful=False, reason=str(e))

        # Return success to allow parameter changes
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = SessionConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()