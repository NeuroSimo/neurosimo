import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from project_interfaces.srv import GetRecordingInfo
from project_interfaces.msg import RecordingInfo
from std_msgs.msg import String

import os
import json
from pathlib import Path


class SessionPlayerNode(Node):
    def __init__(self):
        super().__init__('session_player')
        self.logger = self.get_logger()
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Active project state
        self._active_project = None
        self._recordings_directory = None

        # Subscribe to active project with latched QoS
        qos_persist_latest = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._active_project_subscriber = self.create_subscription(
            String,
            '/projects/active',
            self._handle_set_active_project,
            qos_persist_latest,
            callback_group=self.callback_group
        )

        # Create service for getting recording info
        self._get_recording_info_service = self.create_service(
            GetRecordingInfo,
            '/session_player/recording/get_info',
            self._handle_get_recording_info,
            callback_group=self.callback_group
        )

        self.logger.info('Session Player initialized')

    def _handle_set_active_project(self, msg):
        """Handle active project changes."""
        self._active_project = msg.data
        self._recordings_directory = f'/app/projects/{self._active_project}/recordings'
        self.logger.info(f'Active project set to: {self._active_project}')
        self.logger.info(f'Recordings directory: {self._recordings_directory}')

    def _handle_get_recording_info(self, request, response):
        """Handle get recording info service calls."""
        self.logger.info(f'Received get recording info request for: {request.filename}')

        # Check if recordings directory is set
        if not self._recordings_directory:
            self.logger.error('Recordings directory not set. Make sure a project is active.')
            response.success = False
            return response

        # Build path to the metadata JSON file
        json_file_path = os.path.join(self._recordings_directory, request.filename)

        # Check if file exists
        if not os.path.exists(json_file_path):
            self.logger.error(f'Recording metadata file not found: {json_file_path}')
            response.success = False
            return response

        # Read and parse the JSON file
        try:
            with open(json_file_path, 'r') as f:
                metadata = json.load(f)

            # Create RecordingInfo message
            recording_info = RecordingInfo()

            # Set basic info
            recording_info.json_filename = request.filename

            # Extract stream info
            stream_info = metadata.get('stream_info', {})
            recording_info.num_eeg_channels = stream_info.get('num_eeg_channels', 0)
            recording_info.num_emg_channels = stream_info.get('num_emg_channels', 0)
            recording_info.sampling_frequency = stream_info.get('sampling_frequency', 0)

            # Extract session config
            session_config = metadata.get('session_config', {})
            recording_info.project_name = session_config.get('project_name', '')
            recording_info.subject_id = session_config.get('subject_id', '')
            recording_info.protocol_name = session_config.get('protocol_name', '')
            recording_info.notes = session_config.get('notes', '')

            # Extract pipeline component config from session_config
            recording_info.preprocessor_module = session_config.get('preprocessor_module', '')
            recording_info.decider_module = session_config.get('decider_module', '')
            recording_info.presenter_module = session_config.get('presenter_module', '')
            recording_info.preprocessor_enabled = session_config.get('preprocessor_enabled', False)
            recording_info.decider_enabled = session_config.get('decider_enabled', False)
            recording_info.presenter_enabled = session_config.get('presenter_enabled', False)

            # Extract timestamps
            recording_info.start_time = metadata.get('start_time', '')
            recording_info.end_time = metadata.get('end_time', '')

            # Calculate duration if both timestamps are available
            if recording_info.start_time and recording_info.end_time:
                from datetime import datetime
                start = datetime.fromisoformat(recording_info.start_time)
                end = datetime.fromisoformat(recording_info.end_time)
                recording_info.duration = (end - start).total_seconds()
            else:
                recording_info.duration = 0.0

            # Extract software provenance
            provenance = metadata.get('software_provenance', {})
            recording_info.git_commit = provenance.get('git_commit', '')
            recording_info.git_state = provenance.get('git_state', '')
            recording_info.version = provenance.get('version', '')

            # Check if session has been exported
            # Export folder is named [recording_name]_export next to the recording directory
            recording_name = os.path.splitext(request.filename)[0]  # Remove .json extension
            export_folder_path = os.path.join(self._recordings_directory, f'{recording_name}_export')
            recording_info.exported = os.path.exists(export_folder_path) and os.path.isdir(export_folder_path)
            # Store relative path from project root for electronAPI compatibility
            recording_info.export_directory = f'recordings/{recording_name}_export' if recording_info.exported else ''

            response.recording_info = recording_info
            response.success = True

            self.logger.info(f'Successfully retrieved recording info for: {request.filename}')

        except json.JSONDecodeError as e:
            self.logger.error(f'Failed to parse JSON file {json_file_path}: {e}')
            response.success = False
        except Exception as e:
            self.logger.error(f'Error reading recording info: {e}')
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SessionPlayerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
