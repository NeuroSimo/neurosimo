import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import os
import shutil
import stat

from project_interfaces.srv import CreateProject
from .random_data_generator import RandomDataGenerator


class ProjectManagerNode(Node):
    def __init__(self):
        super().__init__('project_manager')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Create the create project service
        self.create_service(CreateProject, '/projects/create', self.create_project_callback, callback_group=self.callback_group)

        self.logger.info('Project manager node initialized')

    def create_project_callback(self, request, response):
        """Create a new project directly without calling the shell script."""
        project_name = request.project_name

        if not project_name:
            self.logger.error('Project name cannot be empty')
            response.success = False
            return response

        # Validate project name (no spaces or special characters)
        if ' ' in project_name or '/' in project_name or '\\' in project_name:
            self.logger.error(f'Project name contains invalid characters: {project_name}')
            response.success = False
            return response

        # Use fixed paths
        projects_root = '/app/projects'
        project_dir = os.path.join(projects_root, project_name)

        # Check if project directory already exists
        if os.path.exists(project_dir):
            self.logger.error(f'Project already exists: {project_name}')
            response.success = False
            return response

        # Create the project directory
        os.makedirs(project_dir, exist_ok=True)
        self.logger.info(f'Created project directory: {project_dir}')

        # Copy project template to the new project directory
        template_dir = '/app/bundled_projects/template'
        for item in os.listdir(template_dir):
            source = os.path.join(template_dir, item)
            destination = os.path.join(project_dir, item)
            if os.path.isdir(source):
                shutil.copytree(source, destination)
            else:
                shutil.copy2(source, destination)
        self.logger.info(f'Copied project template to: {project_dir}')

        # Generate random data files using the RandomDataGenerator class
        eeg_simulator_dir = os.path.join(project_dir, 'eeg_simulator')
        os.makedirs(eeg_simulator_dir, exist_ok=True)

        # Create recordings directory
        recordings_dir = os.path.join(project_dir, 'recordings')
        os.makedirs(recordings_dir, exist_ok=True)

        # Create a single generator instance
        generator = RandomDataGenerator()

        # Define the data generation configurations
        data_configs = [
            ("Random, 100 Hz", 100, "random_data_100_hz"),
            ("Random, 1 kHz", 1000, "random_data_1_khz"),
            ("Random, 3 kHz", 3000, "random_data_3_khz"),
            ("Random, 5 kHz", 5000, "random_data_5_khz"),
        ]

        for dataset_name, sampling_freq, filename in data_configs:
            generator.generate_and_save(
                dataset_name=dataset_name,
                sampling_frequency=sampling_freq,
                output_filename=filename,
                output_directory=eeg_simulator_dir,
                loop=True
            )
            self.logger.info(f'Generated {filename} with {sampling_freq} Hz sampling frequency')

        # Set proper permissions for user access
        self._set_project_permissions(project_dir)

        self.logger.info(f'Successfully created project: {project_name}')
        response.success = True

        return response

    def _set_project_permissions(self, project_dir):
        """Set proper permissions and ownership on project directory and contents."""
        # Get the ownership of the parent projects directory to determine target user/group
        projects_root = os.path.dirname(project_dir)
        parent_stat = os.stat(projects_root)
        target_uid = parent_stat.st_uid
        target_gid = parent_stat.st_gid

        # Set directory permissions to 775 (rwxrwxr-x) for group access
        # Set file permissions to 664 (rw-rw-r--) for group access
        for root, dirs, files in os.walk(project_dir):
            # Set directory permissions and ownership
            for dir_name in dirs:
                dir_path = os.path.join(root, dir_name)
                os.chmod(dir_path, stat.S_IRWXU | stat.S_IRWXG | stat.S_IROTH | stat.S_IXOTH)
                os.chown(dir_path, target_uid, target_gid)

            # Set file permissions and ownership
            for file_name in files:
                file_path = os.path.join(root, file_name)
                os.chmod(file_path, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP | stat.S_IROTH)
                os.chown(file_path, target_uid, target_gid)

        self.logger.info(f'Set permissions and ownership on project directory: {project_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = ProjectManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()