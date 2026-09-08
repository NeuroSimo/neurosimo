import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from neurosimo_project_interfaces.srv import ListProjects
from neurosimo_project_interfaces.msg import FilenameList
from neurosimo_system_interfaces.msg import SystemConfig
from neurosimo_system_interfaces.srv import GetSystemConfig, SetSystemConfig

from directory_utils import DirectoryWatcher

from .system_storage_manager import SystemStorageManager


# Config fields, in the order they appear in SystemConfig.msg. The persisted JSON uses
# the same names, so the message and the stored dict convert without a mapping table.
CONFIG_FIELDS = [
    'active_project',
    # EEG Configuration
    'eeg_port',
    'eeg_device',
    'turbolink_sampling_frequency',
    'turbolink_eeg_channel_count',
    'maximum_dropped_samples',
    # LabJack Configuration
    'enable_labjack',
    # Timing Configuration
    'maximum_loopback_latency',
    'maximum_timing_error',
    'trigger_to_pulse_delay',
    # Disk Space Monitoring Configuration
    'disk_warning_threshold',
    'disk_error_threshold',
    # System Configuration
    'locale',
]


class SystemConfiguratorNode(Node):
    """Owns the system configuration: persists it, publishes it, and validates updates.

    The configuration is set through the /neurosimo/system_configurator/config/set
    service, which takes a complete SystemConfig. Each call is an atomic full replace:
    either the whole config validates and is persisted, or nothing changes.
    """

    def __init__(self):
        super().__init__('system_configurator')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Initialize system storage manager
        self.storage_manager = SystemStorageManager(self.logger)

        # Load system config, migrating the legacy global_config.json if present
        system_config = self.storage_manager.get_system_config()

        # Services
        self.create_service(ListProjects, '/neurosimo/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetSystemConfig, '/neurosimo/system_configurator/config/set', self.set_system_config_callback, callback_group=self.callback_group)
        self.create_service(GetSystemConfig, '/neurosimo/system_configurator/config/get', self.get_system_config_callback, callback_group=self.callback_group)

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.system_config_publisher = self.create_publisher(SystemConfig, "/neurosimo/system_configurator/config", qos, callback_group=self.callback_group)
        self.project_list_publisher = self.create_publisher(FilenameList, "/neurosimo/system_configurator/projects", qos, callback_group=self.callback_group)

        # Publish initial system config and project list
        self.publish_system_config(system_config)
        self.publish_project_list()

        # Watch the projects root so that projects added or removed on disk are
        # reflected live, and the active project is reconciled if it disappears.
        self.directory_watcher = DirectoryWatcher(self.logger)
        self.directory_watcher.watch_subdirectories(
            self.storage_manager.PROJECTS_ROOT, self.handle_projects_change)

    # Conversion between the SystemConfig message and the persisted dict

    @staticmethod
    def config_to_dict(config):
        return {field: getattr(config, field) for field in CONFIG_FIELDS}

    @staticmethod
    def dict_to_config(config_dict):
        config = SystemConfig()
        for field in CONFIG_FIELDS:
            setattr(config, field, config_dict[field])
        return config

    def publish_system_config(self, config_dict):
        """Build and publish the system config from dict."""
        self.system_config_publisher.publish(self.dict_to_config(config_dict))

    def validate_config(self, config_dict):
        """Validate a proposed config. Returns an error message, or None if valid.

        An empty active_project is allowed only when no projects exist on disk, which
        is the state the node reconciles to when the last project is removed.
        """
        active_project = config_dict['active_project']
        projects = self.storage_manager.list_projects()

        if active_project == '':
            if projects:
                return f"Active project cannot be empty; available projects: {projects}"
        elif active_project not in projects:
            return f"Project does not exist: {active_project}"

        return None

    def publish_project_list(self):
        """Publish the current list of projects available on disk."""
        msg = FilenameList()
        msg.filenames = self.storage_manager.list_projects()
        self.project_list_publisher.publish(msg)
        self.logger.info(f"Published project list: {msg.filenames}")

    def handle_projects_change(self):
        """Watcher callback: the set of project directories on disk changed.

        Republishes the project list and, if the active project no longer exists,
        reconciles it to an available project and republishes the system config so
        downstream nodes never keep a project name without a directory.
        """
        config = self.storage_manager.load_system_config()
        previous_active = config.get('active_project')

        # reconcile_active_project persists a corrected active_project (falling back
        # to the first available project, or '' if none remain) when needed.
        self.storage_manager.reconcile_active_project(config)
        new_active = config.get('active_project')

        # Always republish so subscribers stay current with add/remove/rename.
        self.publish_project_list()

        if new_active == previous_active:
            return

        self.logger.info(
            f"Active project reconciled from '{previous_active}' to '{new_active}'")
        self.publish_system_config(config)

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
    
    def get_system_config_callback(self, request, response):
        try:
            response.config = self.dict_to_config(self.storage_manager.load_system_config())
            response.success = True
        except Exception as e:
            self.logger.error(f"Error getting system config: {e}")
            response.success = False
        return response

    def set_system_config_callback(self, request, response):
        """Validate, persist, and publish a complete system config."""
        try:
            config_dict = self.config_to_dict(request.config)

            error = self.validate_config(config_dict)
            if error is not None:
                self.logger.error(f"Rejected system config: {error}")
                response.success = False
                response.message = error
                return response

            self.storage_manager.save_system_config(config_dict)
            self.publish_system_config(config_dict)

            self.logger.info("System config updated.")
            response.success = True
            response.message = ''
        except Exception as e:
            self.logger.error(f"Error setting system config: {e}")
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.directory_watcher.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SystemConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()