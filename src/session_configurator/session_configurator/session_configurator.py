import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.executors import SingleThreadedExecutor

from neurosimo_system_interfaces.msg import SessionConfig, GlobalConfig
from neurosimo_project_interfaces.msg import FilenameList
from rcl_interfaces.msg import SetParametersResult

from .session_storage_manager import SessionStorageManager
from .directory_watcher import DirectoryWatcher


class SessionConfiguratorNode(Node):
    def __init__(self):
        super().__init__('session_configurator')
        self.logger = self.get_logger()

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

        self.declare_parameter('replay.bag_id', '')
        self.declare_parameter('replay.play_preprocessed', False)

        # Data source parameter
        self.declare_parameter('data_source', 'simulator')


        # Subscribers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.global_config_subscription = self.create_subscription(
            GlobalConfig,
            '/neurosimo/global_configurator/config',
            self.global_config_callback,
            qos
        )

        # Publishers
        self.decider_list_publisher = self.create_publisher(FilenameList, "/neurosimo/pipeline/decider/list", qos)
        self.preprocessor_list_publisher = self.create_publisher(FilenameList, "/neurosimo/pipeline/preprocessor/list", qos)
        self.presenter_list_publisher = self.create_publisher(FilenameList, "/neurosimo/pipeline/presenter/list", qos)
        self.protocol_list_publisher = self.create_publisher(FilenameList, "/neurosimo/experiment/protocol/list", qos)
        self.dataset_list_publisher = self.create_publisher(FilenameList, "/neurosimo/eeg_simulator/dataset/list", qos)
        self.recordings_list_publisher = self.create_publisher(FilenameList, "/neurosimo/recording/recordings/list", qos)
        
        # Session config publisher
        self.session_config_publisher = self.create_publisher(SessionConfig, "/neurosimo/session_configurator/config", qos)

        # Define directory watch configurations.
        # Each entry: (subdirectory, extensions, publisher, component_name, selection_param_name)
        # selection_param_name is the session-config / ROS parameter whose value must be
        # one of the entries in the published list; if not, it is reconciled.
        self.watch_configs = [
            ("decider", [".py"], self.decider_list_publisher, "decider", "decider.module"),
            ("preprocessor", [".py"], self.preprocessor_list_publisher, "preprocessor", "preprocessor.module"),
            ("presenter", [".py"], self.presenter_list_publisher, "presenter", "presenter.module"),
            ("protocols", [".yaml", ".yml"], self.protocol_list_publisher, "protocol", "experiment.protocol"),
            ("eeg_simulator", [".json"], self.dataset_list_publisher, "dataset", "simulator.dataset_filename"),
            ("recordings", [".json"], self.recordings_list_publisher, "recordings", "replay.bag_id"),
        ]

        # Add parameter change callback to save changes to session state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def global_config_callback(self, msg):
        """Handle global config changes from global configurator."""
        project_name = msg.active_project
        
        # Only process if active project has actually changed
        if project_name == self.active_project:
            return

        self.logger.info(f"Active project changed to: {project_name}")
        self.active_project = project_name
        
        # Load session config for the new project
        session_config = self.storage_manager.get_session_config_for_project(project_name)

        # Compute available filename lists for this project before setting parameters,
        # so selections that no longer match available files can be reconciled first.
        filename_lists = {
            component_name: self.compute_filename_list(project_name, subdirectory, file_extensions, component_name)
            for subdirectory, file_extensions, _, component_name, _ in self.watch_configs
        }

        # Reconcile stored selections against the available files.
        for _, _, _, component_name, param_name in self.watch_configs:
            self.reconcile_config_value(session_config, param_name, component_name, filename_lists[component_name])

        # Persist any reconciliation so that parameter_change_callback, when it reloads
        # from disk below, sees a consistent baseline.
        self.storage_manager.save_session_config(project_name, session_config)

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
            rclpy.parameter.Parameter('replay.bag_id', rclpy.parameter.Parameter.Type.STRING, session_config["replay.bag_id"]),
            rclpy.parameter.Parameter('replay.play_preprocessed', rclpy.parameter.Parameter.Type.BOOL, session_config["replay.play_preprocessed"]),
        ])

        # Publish session config from the loaded dict (not from ROS params)
        self.publish_session_config(session_config)

        # Publish the lists of modules for the new project and setup watches
        self.publish_and_watch_directories(project_name, filename_lists)

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

    def compute_filename_list(self, project_name, subdirectory, file_extensions, component_name):
        """Compute the list of filenames for the specified project and component."""
        modules = self.list_files(project_name, subdirectory, file_extensions)

        # For recordings, expose bag_ids (base name without .json) instead of JSON filenames
        if component_name == "recordings":
            modules = [m.rsplit(".json", 1)[0] if m.endswith(".json") else m for m in modules]

        return modules

    def publish_filename_list(self, project_name, publisher, component_name, modules):
        """Publish a precomputed filename list for the specified component."""
        msg = FilenameList()
        msg.filenames = modules
        publisher.publish(msg)

        self.logger.info(f"Published {component_name} module list for project '{project_name}': {modules}")

    def reconcile_config_value(self, session_config, param_name, component_name, filenames):
        """In-place reconcile a stored selection against available filenames.

        If the stored value is not in the list, fall back to the first available entry,
        or to an empty string if the list is empty. Mutates session_config.
        """
        current = session_config.get(param_name, '')
        if current in filenames:
            return
        new_value = filenames[0] if filenames else ''
        if new_value == current:
            return
        self.logger.warning(
            f"Stored {component_name} selection '{current}' is not in available list; "
            f"reconciling '{param_name}' to '{new_value}'"
        )
        session_config[param_name] = new_value

    def reconcile_parameter(self, param_name, component_name, filenames):
        """Reconcile a live ROS parameter against an available filename list.

        If the parameter's current value is not in the list, update it (which triggers
        parameter_change_callback to persist the change and republish session config).
        """
        current = self.get_parameter(param_name).get_parameter_value().string_value
        if current in filenames:
            return
        new_value = filenames[0] if filenames else ''
        if new_value == current:
            return
        self.logger.warning(
            f"Active {component_name} selection '{current}' is not in available list; "
            f"updating '{param_name}' to '{new_value}'"
        )
        self.set_parameters([
            rclpy.parameter.Parameter(param_name, rclpy.parameter.Parameter.Type.STRING, new_value)
        ])

    def publish_and_watch_directories(self, project_name, filename_lists):
        """Publish filename lists and setup file system watches for all directories."""
        self.directory_watcher.unwatch_all()

        for subdirectory, file_extensions, publisher, component_name, param_name in self.watch_configs:
            self.publish_filename_list(project_name, publisher, component_name, filename_lists[component_name])

            directory_path = os.path.join(self.storage_manager.PROJECTS_ROOT, project_name, subdirectory)

            def create_callback(proj, subdir, exts, pub, comp, param):
                return lambda: self.handle_directory_change(proj, subdir, exts, pub, comp, param)

            callback = create_callback(project_name, subdirectory, file_extensions, publisher, component_name, param_name)
            self.directory_watcher.watch_directory(directory_path, file_extensions, callback)

    def handle_directory_change(self, project_name, subdirectory, file_extensions, publisher, component_name, param_name):
        """Callback invoked when a watched directory changes.

        Recomputes and republishes the filename list, then reconciles the associated
        parameter so the UI never displays a stale selection that is no longer available.
        """
        modules = self.compute_filename_list(project_name, subdirectory, file_extensions, component_name)
        self.publish_filename_list(project_name, publisher, component_name, modules)
        self.reconcile_parameter(param_name, component_name, modules)

    # Service callbacks

    def publish_session_config(self, session_config: dict):
        """Build and publish session config from dict."""
        config = SessionConfig()
        
        config.subject_id = session_config.get('subject_id', '')
        config.notes = session_config.get('notes', '')
        
        config.decider_module = session_config.get('decider.module', '')
        config.decider_enabled = session_config.get('decider.enabled', False)
        
        config.preprocessor_module = session_config.get('preprocessor.module', '')
        config.preprocessor_enabled = session_config.get('preprocessor.enabled', False)
        
        config.presenter_module = session_config.get('presenter.module', '')
        config.presenter_enabled = session_config.get('presenter.enabled', False)
        
        config.protocol_filename = session_config.get('experiment.protocol', '')
        config.data_source = session_config.get('data_source', 'simulator')
        config.simulator_dataset_filename = session_config.get('simulator.dataset_filename', '')
        config.simulator_start_time = session_config.get('simulator.start_time', 0.0)
        
        config.replay_bag_id = session_config.get('replay.bag_id', '')
        config.replay_play_preprocessed = session_config.get('replay.play_preprocessed', False)
        
        self.session_config_publisher.publish(config)

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

            # Publish the updated session config immediately (from the dict, not from ROS params)
            self.publish_session_config(session_config)

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
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()