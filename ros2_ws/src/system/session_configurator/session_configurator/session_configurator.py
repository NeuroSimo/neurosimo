import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.srv import (
    ListProjects,
    SetActiveProject,
)
from system_interfaces.srv import GetSessionConfig
from system_interfaces.msg import SessionConfig
from project_interfaces.msg import FilenameList

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

        # Session parameters
        self.declare_parameter('notes', '')
        self.declare_parameter('subject_id', '')

        # Decider parameters
        self.declare_parameter('decider.module', '')
        self.declare_parameter('decider.enabled', False)

        # Preprocessor parameters
        self.declare_parameter('preprocessor.module', '')
        self.declare_parameter('preprocessor.enabled', False)

        # Presenter parameters
        self.declare_parameter('presenter.module', '')
        self.declare_parameter('presenter.enabled', False)

        # Experiment parameters
        self.declare_parameter('experiment.protocol', '')

        # Simulator parameters
        self.declare_parameter('simulator.dataset_filename', '')
        self.declare_parameter('simulator.start_time', 0.0)

        # Playback parameters
        self.declare_parameter('playback.bag_filename', '')
        self.declare_parameter('playback.is_preprocessed', False)

        # Data source parameter
        self.declare_parameter('data_source', 'simulator')

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetActiveProject, '/projects/active/set', self.set_active_project_callback, callback_group=self.callback_group)
        self.create_service(GetSessionConfig, '/session_configurator/get_config', self.get_session_config_callback, callback_group=self.callback_group)

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos, callback_group=self.callback_group)

        self.decider_list_publisher = self.create_publisher(FilenameList, "/pipeline/decider/list", qos, callback_group=self.callback_group)
        self.preprocessor_list_publisher = self.create_publisher(FilenameList, "/pipeline/preprocessor/list", qos, callback_group=self.callback_group)
        self.presenter_list_publisher = self.create_publisher(FilenameList, "/pipeline/presenter/list", qos, callback_group=self.callback_group)
        self.protocol_list_publisher = self.create_publisher(FilenameList, "/experiment/protocol/list", qos, callback_group=self.callback_group)
        self.dataset_list_publisher = self.create_publisher(FilenameList, "/eeg_simulator/dataset/list", qos, callback_group=self.callback_group)
        self.recordings_list_publisher = self.create_publisher(FilenameList, "/playback/recordings/list", qos, callback_group=self.callback_group)

        # Set active project
        active_project = self.project_manager.get_active_project()
        self.set_active_project(active_project)

        # Add parameter change callback to save changes to session state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def set_active_project(self, project_name):
        if not project_name in self.project_manager.list_projects():
            self.logger.error(f"Project does not exist: {project_name}")
            return False

        # Store the active project in the project state if it has changed
        if project_name != self.project_manager.get_active_project():
            self.project_manager.save_active_project(project_name)

        # Publish active project
        msg = String(data=project_name)
        self.active_project_publisher.publish(msg)

        # Load session state for the project
        session_state = self.project_manager.get_session_state_for_project(project_name)

        # Set ROS2 parameters from the loaded state
        self.set_parameters([
            rclpy.parameter.Parameter('notes', rclpy.parameter.Parameter.Type.STRING, session_state["notes"]),
            rclpy.parameter.Parameter('subject_id', rclpy.parameter.Parameter.Type.STRING, session_state["subject_id"]),
            rclpy.parameter.Parameter('decider.module', rclpy.parameter.Parameter.Type.STRING, session_state["decider.module"]),
            rclpy.parameter.Parameter('decider.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["decider.enabled"]),
            rclpy.parameter.Parameter('preprocessor.module', rclpy.parameter.Parameter.Type.STRING, session_state["preprocessor.module"]),
            rclpy.parameter.Parameter('preprocessor.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["preprocessor.enabled"]),
            rclpy.parameter.Parameter('presenter.module', rclpy.parameter.Parameter.Type.STRING, session_state["presenter.module"]),
            rclpy.parameter.Parameter('presenter.enabled', rclpy.parameter.Parameter.Type.BOOL, session_state["presenter.enabled"]),
            rclpy.parameter.Parameter('experiment.protocol', rclpy.parameter.Parameter.Type.STRING, session_state["experiment.protocol"]),
            rclpy.parameter.Parameter('simulator.dataset_filename', rclpy.parameter.Parameter.Type.STRING, session_state["simulator.dataset_filename"]),
            rclpy.parameter.Parameter('simulator.start_time', rclpy.parameter.Parameter.Type.DOUBLE, session_state["simulator.start_time"]),
            rclpy.parameter.Parameter('data_source', rclpy.parameter.Parameter.Type.STRING, session_state["data_source"]),
            rclpy.parameter.Parameter('playback.bag_filename', rclpy.parameter.Parameter.Type.STRING, session_state["playback.bag_filename"]),
            rclpy.parameter.Parameter('playback.is_preprocessed', rclpy.parameter.Parameter.Type.BOOL, session_state["playback.is_preprocessed"]),
        ])

        # Publish the lists of modules for the new project
        self.publish_filename_list(project_name, "decider", [".py"], self.decider_list_publisher, "decider")
        self.publish_filename_list(project_name, "preprocessor", [".py"], self.preprocessor_list_publisher, "preprocessor")
        self.publish_filename_list(project_name, "presenter", [".py"], self.presenter_list_publisher, "presenter")
        self.publish_filename_list(project_name, "protocols", [".yaml", ".yml"], self.protocol_list_publisher, "protocol")
        self.publish_filename_list(project_name, "eeg_simulator", [".json"], self.dataset_list_publisher, "dataset")
        self.publish_filename_list(project_name, "recordings", [".json"], self.recordings_list_publisher, "recordings")

        return True

    def list_modules(self, project_name, subdirectory, file_extensions):
        """List all files with specified extensions in the subdirectory of the specified project."""
        module_dir = os.path.join(self.project_manager.PROJECTS_ROOT, project_name, subdirectory)

        if not os.path.exists(module_dir):
            self.logger.warning(f"Directory does not exist: {module_dir}")
            return []

        try:
            # List all files with the specified extensions
            matching_files = []
            for ext in file_extensions:
                matching_files.extend([f for f in os.listdir(module_dir)
                                     if os.path.isfile(os.path.join(module_dir, f)) and f.endswith(ext)])

            self.logger.info(f"Found {len(matching_files)} modules in project '{project_name}'/{subdirectory}: {matching_files}")
            return matching_files

        except Exception as e:
            self.logger.error(f"Error listing modules for project '{project_name}'/{subdirectory}: {e}")
            return []

    def publish_filename_list(self, project_name, subdirectory, file_extensions, publisher, component_name):
        """Publish the list of modules for the specified project and component."""
        modules = self.list_modules(project_name, subdirectory, file_extensions)

        # Create and publish FilenameList message
        msg = FilenameList()
        msg.filenames = modules
        publisher.publish(msg)

        self.logger.info(f"Published {component_name} module list for project '{project_name}': {modules}")

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

        response.success = True
        return response

    def get_session_config_callback(self, request, response):
        """Return the current session configuration."""
        # Get current parameter values
        config = SessionConfig()

        config.project_name = self.project_manager.get_active_project()
        config.subject_id = self.get_parameter('subject_id').get_parameter_value().string_value
        config.notes = self.get_parameter('notes').get_parameter_value().string_value

        config.decider_module = self.get_parameter('decider.module').get_parameter_value().string_value
        config.decider_enabled = self.get_parameter('decider.enabled').get_parameter_value().bool_value

        config.preprocessor_module = self.get_parameter('preprocessor.module').get_parameter_value().string_value
        config.preprocessor_enabled = self.get_parameter('preprocessor.enabled').get_parameter_value().bool_value

        config.presenter_module = self.get_parameter('presenter.module').get_parameter_value().string_value
        config.presenter_enabled = self.get_parameter('presenter.enabled').get_parameter_value().bool_value

        config.protocol_filename = self.get_parameter('experiment.protocol').get_parameter_value().string_value
        config.data_source = self.get_parameter('data_source').get_parameter_value().string_value
        config.simulator_dataset_filename = self.get_parameter('simulator.dataset_filename').get_parameter_value().string_value
        config.simulator_start_time = self.get_parameter('simulator.start_time').get_parameter_value().double_value

        response.config = config
        response.success = True
 
        self.logger.info(f"Returned session config: subject={config.subject_id}, data_source={config.data_source}")

        return response

    def parameter_change_callback(self, params):
        """Callback to handle session parameter changes and save them to session state."""
        try:
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