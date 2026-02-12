import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from project_interfaces.srv import GetRecordingInfo
from project_interfaces.msg import RecordingInfo
from system_interfaces.msg import GlobalConfig

import os
import json
from pathlib import Path


# Expected structure and types of the recording metadata JSON
METADATA_SCHEMA = {
    'session_id': str,
    'global_config': dict,
    'session_config': dict,
    'stream_info': {
        'num_eeg_channels': int,
        'num_emg_channels': int,
        'sampling_frequency': int,
    },
    'timing': {
        'start_time': str,
        'end_time': str,
        'duration': float,
    },
    'provenance': {
        'software': {
            'git_commit': str,
            'git_state': str,
            'version': str,
        },
        # Fingerprints object may be present with numeric values;
        # individual keys under it are optional.
        'fingerprints': {
            'data_source': int,
            'preprocessor': int,
            'decision': int,
        },
    },
}


class RecordingManagerNode(Node):
    def __init__(self):
        super().__init__('recording_manager')
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

        self._global_config_subscriber = self.create_subscription(
            GlobalConfig,
            '/global_configurator/config',
            self._handle_global_config,
            qos_persist_latest,
            callback_group=self.callback_group
        )

        # Create service for getting recording info
        self._get_recording_info_service = self.create_service(
            GetRecordingInfo,
            '/recording_manager/recording/get_info',
            self._handle_get_recording_info,
            callback_group=self.callback_group
        )

        self.logger.info('Recording manager initialized')

    def _validate_node(self, obj, schema, require_all_keys: bool) -> bool:
        """Recursively validate a dict against a simple schema dict."""
        if not isinstance(obj, dict):
            return False

        for key, expected in schema.items():
            if key not in obj:
                if require_all_keys:
                    return False
                continue

            value = obj[key]

            if isinstance(expected, dict):
                # Nested dict schema; inner keys are optional by default.
                if not isinstance(value, dict):
                    return False
                if not self._validate_node(value, expected, require_all_keys=False):
                    return False
            else:
                # Leaf type
                if not isinstance(value, expected):
                    return False

        return True

    def _validate_metadata(self, metadata):
        """Validate that the recording metadata matches the expected schema."""
        return self._validate_node(metadata, METADATA_SCHEMA, require_all_keys=True)

    def _handle_global_config(self, msg):
        """Handle global config changes."""
        project_name = msg.active_project
        
        # Only process if active project has actually changed
        if project_name == self._active_project:
            return
        
        self._active_project = project_name
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

            # Validate metadata structure
            if not self._validate_metadata(metadata):
                self.logger.error(f'Invalid metadata structure in file: {json_file_path}')
                response.success = False
                return response

            # Create RecordingInfo message
            recording_info = RecordingInfo()

            # Set basic info
            recording_info.json_filename = request.filename

            # Extract session config fields
            session_config = metadata.get('session_config', {})
            recording_info.protocol_filename = session_config.get('protocol_filename', '')
            recording_info.data_source = session_config.get('data_source', '')
            recording_info.simulator_dataset_filename = session_config.get('simulator_dataset_filename', '')
            recording_info.simulator_start_time = session_config.get('simulator_start_time', 0.0)
            recording_info.replay_bag_id = session_config.get('replay_bag_id', '')
            recording_info.replay_play_preprocessed = session_config.get('replay_play_preprocessed', False)

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

            # Extract timestamps (schema guarantees correct types)
            timing = metadata.get('timing', {})
            recording_info.start_time = timing.get('start_time', '')
            recording_info.end_time = timing.get('end_time', '')
            recording_info.duration = timing.get('duration', 0.0)

            # Extract software provenance and fingerprints
            provenance = metadata.get('provenance', {})
            software = provenance.get('software', {})
            fingerprints = provenance.get('fingerprints', {})

            recording_info.git_commit = software.get('git_commit', '')
            recording_info.git_state = software.get('git_state', '')
            recording_info.version = software.get('version', '')

            # Fingerprints (schema guarantees ints)
            recording_info.data_source_fingerprint = fingerprints.get('data_source', 0)
            recording_info.preprocessor_fingerprint = fingerprints.get('preprocessor', 0)
            recording_info.decision_fingerprint = fingerprints.get('decision', 0)

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
    node = RecordingManagerNode()

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
