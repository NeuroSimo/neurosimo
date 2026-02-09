import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from system_interfaces.srv import GetSessionConfig
from system_interfaces.msg import SessionConfig
from project_interfaces.msg import FilenameList

from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult

from .session_storage_manager import SessionStorageManager
from .directory_watcher import DirectoryWatcher


class SessionConfiguratorNode(Node):
    def __init__(self):
        super().__init__('session_configurator')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Initialize session storage manager
        self.storage_manager = SessionStorageManager(self.logger)
        
        # Initialize directory watcher
        self.directory_watcher = DirectoryWatcher(self.logger)
        
        # Track active project
        self.active_project = None

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
        self.create_service(GetSessionConfig, '/session_configurator/get_config', self.get_session_config_callback, callback_group=self.callback_group)

        # Subscribers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_subscription = self.create_subscription(
            String,
            '/projects/active',
            self.active_project_callback,
            qos,
            callback_group=self.callback_group
        )

        # Publishers
        self.decider_list_publisher = self.create_publisher(FilenameList, "/pipeline/decider/list", qos, callback_group=self.callback_group)
        self.preprocessor_list_publisher = self.create_publisher(FilenameList, "/pipeline/preprocessor/list", qos, callback_group=self.callback_group)
        self.presenter_list_publisher = self.create_publisher(FilenameList, "/pipeline/presenter/list", qos, callback_group=self.callback_group)
        self.protocol_list_publisher = self.create_publisher(FilenameList, "/experiment/protocol/list", qos, callback_group=self.callback_group)
        self.dataset_list_publisher = self.create_publisher(FilenameList, "/eeg_simulator/dataset/list", qos, callback_group=self.callback_group)
        self.recordings_list_publisher = self.create_publisher(FilenameList, "/playback/recordings/list", qos, callback_group=self.callback_group)

        # Define directory watch configurations
        self.watch_configs = [
            ("decider", [".py"], self.decider_list_publisher, "decider"),
            ("preprocessor", [".py"], self.preprocessor_list_publisher, "preprocessor"),
            ("presenter", [".py"], self.presenter_list_publisher, "presenter"),
            ("protocols", [".yaml", ".yml"], self.protocol_list_publisher, "protocol"),
            ("eeg_simulator", [".json"], self.dataset_list_publisher, "dataset"),
            ("recordings", [".json"], self.recordings_list_publisher, "recordings"),
        ]

        # Add parameter change callback to save changes to session state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def active_project_callback(self, msg):
        """Handle active project changes from global configurator."""
        project_name = msg.data
        self.logger.info(f"Active project changed to: {project_name}")
        
        self.active_project = project_name
        
        # Load session config for the new project
        session_config = self.storage_manager.get_session_config_for_project(project_name)

        # Set ROS2 parameters from the loaded config
        self.set_parameters([
            rclpy.parameter.Parameter('notes', rclpy.parameter.Parameter.Type.STRING, session_config["notes"]),
            rclpy.parameter.Parameter('subject_id', rclpy.parameter.Parameter.Type.STRING, session_config["subject_id"]),
            rclpy.parameter.Parameter('decider.module', rclpy.parameter.Parameter.Type.STRING, session_config["decider.module"]),
            rclpy.parameter.Parameter('decider.enabled', rclpy.parameter.Parameter.Type.BOOL, session_config["decider.enabled"]),
            rclpy.parameter.Parameter('preprocessor.module', rclpy.parameter.Parameter.Type.STRING, session_config["preprocessor.module"]),
            rclpy.parameter.Parameter('preprocessor.enabled', rclpy.parameter.Parameter.Type.BOOL, session_config["preprocessor.enabled"]),
            rclpy.parameter.Parameter('presenter.module', rclpy.parameter.Parameter.Type.STRING, session_config["presenter.module"]),
            rclpy.parameter.Parameter('presenter.enabled', rclpy.parameter.Parameter.Type.BOOL, session_config["presenter.enabled"]),
            rclpy.parameter.Parameter('experiment.protocol', rclpy.parameter.Parameter.Type.STRING, session_config["experiment.protocol"]),
            rclpy.parameter.Parameter('simulator.dataset_filename', rclpy.parameter.Parameter.Type.STRING, session_config["simulator.dataset_filename"]),
            rclpy.parameter.Parameter('simulator.start_time', rclpy.parameter.Parameter.Type.DOUBLE, session_config["simulator.start_time"]),
            rclpy.parameter.Parameter('data_source', rclpy.parameter.Parameter.Type.STRING, session_config["data_source"]),
            rclpy.parameter.Parameter('playback.bag_filename', rclpy.parameter.Parameter.Type.STRING, session_config["playback.bag_filename"]),
            rclpy.parameter.Parameter('playback.is_preprocessed', rclpy.parameter.Parameter.Type.BOOL, session_config["playback.is_preprocessed"]),
        ])

        # Publish the lists of modules for the new project and setup watches
        self.publish_and_watch_directories(project_name)

    def list_files(self, project_name, subdirectory, file_extensions):
        """List all files with specified extensions in the subdirectory of the specified project."""
        module_dir = os.path.join(self.storage_manager.PROJECTS_ROOT, project_name, subdirectory)

        if not os.path.exists(module_dir):
            self.logger.warning(f"Directory does not exist: {module_dir}")
            return []

        try:
            # List all files with the specified extensions
            matching_files = []
            for ext in file_extensions:
                matching_files.extend([f for f in os.listdir(module_dir)
                                     if os.path.isfile(os.path.join(module_dir, f)) and f.endswith(ext)])

            # Sort files alphabetically (chronological order for timestamp-based names)
            matching_files.sort(reverse=True)

            self.logger.info(f"Found {len(matching_files)} modules in project '{project_name}'/{subdirectory}: {matching_files}")
            return matching_files

        except Exception as e:
            self.logger.error(f"Error listing modules for project '{project_name}'/{subdirectory}: {e}")
            return []

    def publish_filename_list(self, project_name, subdirectory, file_extensions, publisher, component_name):
        """Publish the list of modules for the specified project and component."""
        modules = self.list_files(project_name, subdirectory, file_extensions)

        # Create and publish FilenameList message
        msg = FilenameList()
        msg.filenames = modules
        publisher.publish(msg)

        self.logger.info(f"Published {component_name} module list for project '{project_name}': {modules}")
    
    def publish_and_watch_directories(self, project_name):
        """Publish filename lists and setup file system watches for all directories."""
        # Unwatch all previous directories
        self.directory_watcher.unwatch_all()
        
        # Publish initial lists and setup watches for each directory
        for subdirectory, file_extensions, publisher, component_name in self.watch_configs:
            # Publish initial filename list
            self.publish_filename_list(project_name, subdirectory, file_extensions, publisher, component_name)
            
            # Setup watch for the directory
            directory_path = os.path.join(self.storage_manager.PROJECTS_ROOT, project_name, subdirectory)
            
            # Create callback for this specific directory
            def create_callback(proj, subdir, exts, pub, comp):
                return lambda: self.publish_filename_list(proj, subdir, exts, pub, comp)
            
            callback = create_callback(project_name, subdirectory, file_extensions, publisher, component_name)
            self.directory_watcher.watch_directory(directory_path, file_extensions, callback)

    # Service callbacks

    def get_session_config_callback(self, request, response):
        """Return the current session configuration."""
        # Get current parameter values
        config = SessionConfig()

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
        """Callback to handle session parameter changes and save them to session config."""
        try:
            if not self.active_project:
                self.logger.warning("No active project set, cannot save session config")
                return SetParametersResult(successful=True)

            session_config = self.storage_manager.load_session_config(self.active_project)

            for param in params:
                session_config[param.name] = param.value

            # Save the updated session config
            self.storage_manager.save_session_config(self.active_project, session_config)
            self.logger.info(f"Updated session config for project '{self.active_project}' with parameter changes")

        except Exception as e:
            self.logger.error(f"Error handling parameter changes: {e}")
            return SetParametersResult(successful=False, reason=str(e))

        # Return success to allow parameter changes
        return SetParametersResult(successful=True)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.directory_watcher.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SessionConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()