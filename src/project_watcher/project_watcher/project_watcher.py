import os
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.executors import SingleThreadedExecutor

from neurosimo_system_interfaces.msg import GlobalConfig
from neurosimo_project_interfaces.msg import FilenameList

from directory_utils import DirectoryWatcher


class ProjectWatcherNode(Node):
    """Publishes the files available in the active project.

    Session configuration itself lives in the UI and is sent to the session manager with the
    start request, so this node only reports what the active project contains.
    """

    PROJECTS_ROOT = '/app/projects'

    def __init__(self):
        super().__init__('project_watcher')
        self.logger = self.get_logger()

        # Initialize directory watcher
        self.directory_watcher = DirectoryWatcher(self.logger)

        # Track active project
        self.active_project = None

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
        self.external_recordings_list_publisher = self.create_publisher(FilenameList, "/neurosimo/eeg_simulator/external_recordings/list", qos)

        # Define directory watch configurations.
        # Each entry: (subdirectory, extensions, publisher, component_name, reverse)
        # reverse controls sort direction: False for naturally-ordered module/protocol
        # filenames (e.g. "0_hotspot", "1_baseline", ...), True for timestamp-based
        # filenames where the newest entry should be listed first.
        self.watch_configs = [
            ("decider", [".py"], self.decider_list_publisher, "decider", False),
            ("preprocessor", [".py"], self.preprocessor_list_publisher, "preprocessor", False),
            ("presenter", [".py"], self.presenter_list_publisher, "presenter", False),
            ("protocols", [".yaml", ".yml"], self.protocol_list_publisher, "protocol", False),
            ("eeg_simulator", [".json"], self.dataset_list_publisher, "dataset", True),
            ("recordings", [".json"], self.recordings_list_publisher, "recordings", True),
            ("external_recordings", [".vhdr"], self.external_recordings_list_publisher, "external_recordings", True),
        ]

    def global_config_callback(self, msg):
        """Handle global config changes from global configurator."""
        project_name = msg.active_project

        # Only process if active project has actually changed
        if project_name == self.active_project:
            return

        self.logger.info(f"Active project changed to: {project_name}")
        self.active_project = project_name

        filename_lists = {
            component_name: self.compute_filename_list(project_name, subdirectory, file_extensions, component_name, reverse)
            for subdirectory, file_extensions, _, component_name, reverse in self.watch_configs
        }

        # Publish the lists of modules for the new project and setup watches
        self.publish_and_watch_directories(project_name, filename_lists)

    @staticmethod
    def natural_sort_key(filename):
        """Split filename into text/number chunks so numeric prefixes sort by value (e.g. "2_" before "10_")."""
        return [int(chunk) if chunk.isdigit() else chunk.lower() for chunk in re.split(r'(\d+)', filename)]

    def list_files(self, project_name, subdirectory, file_extensions, reverse=False):
        """List all files with specified extensions in the subdirectory of the specified project."""
        module_dir = os.path.join(self.PROJECTS_ROOT, project_name, subdirectory)

        if not os.path.exists(module_dir):
            self.logger.warning(f"Directory does not exist: {module_dir}")
            return []

        try:
            # List all files with the specified extensions
            matching_files = []
            for ext in file_extensions:
                matching_files.extend([f for f in os.listdir(module_dir)
                                     if os.path.isfile(os.path.join(module_dir, f)) and f.endswith(ext)])

            # Natural sort so numeric prefixes order by value; reverse for timestamp-based
            # names where the newest entry should be listed first.
            matching_files.sort(key=self.natural_sort_key, reverse=reverse)

            self.logger.info(f"Found {len(matching_files)} modules in project '{project_name}'/{subdirectory}: {matching_files}")
            return matching_files

        except Exception as e:
            self.logger.error(f"Error listing modules for project '{project_name}'/{subdirectory}: {e}")
            return []

    def compute_filename_list(self, project_name, subdirectory, file_extensions, component_name, reverse=False):
        """Compute the list of filenames for the specified project and component."""
        modules = self.list_files(project_name, subdirectory, file_extensions, reverse)

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

    def publish_and_watch_directories(self, project_name, filename_lists):
        """Publish filename lists and setup file system watches for all directories."""
        self.directory_watcher.unwatch_all()

        for subdirectory, file_extensions, publisher, component_name, reverse in self.watch_configs:
            self.publish_filename_list(project_name, publisher, component_name, filename_lists[component_name])

            directory_path = os.path.join(self.PROJECTS_ROOT, project_name, subdirectory)

            def create_callback(proj, subdir, exts, pub, comp, rev):
                return lambda: self.handle_directory_change(proj, subdir, exts, pub, comp, rev)

            callback = create_callback(project_name, subdirectory, file_extensions, publisher, component_name, reverse)
            self.directory_watcher.watch_directory(directory_path, file_extensions, callback)

    def handle_directory_change(self, project_name, subdirectory, file_extensions, publisher, component_name, reverse=False):
        """Callback invoked when a watched directory changes.

        Recomputes and republishes the filename list. Selections are not reconciled here:
        they belong to the UI, which decides what to show, and a selection that no longer
        exists is rejected when a session is started.
        """
        modules = self.compute_filename_list(project_name, subdirectory, file_extensions, component_name, reverse)
        self.publish_filename_list(project_name, publisher, component_name, modules)

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.directory_watcher.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ProjectWatcherNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()