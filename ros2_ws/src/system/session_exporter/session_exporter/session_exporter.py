import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from system_interfaces.srv import ExportSession, CancelExport
from system_interfaces.msg import ExportDataType, ExporterState

import os
import csv
import json
import threading
from datetime import datetime
from pathlib import Path

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
except ImportError as e:
    print(f"Warning: Could not import rosbag2_py: {e}")


# Mapping from export data types to ROS topics
DATA_TYPE_TO_TOPIC = {
    ExportDataType.RAW_EEG: '/eeg/raw',
    ExportDataType.ENRICHED_EEG: '/eeg/enriched',
    ExportDataType.PREPROCESSED_EEG: '/eeg/preprocessed',
    ExportDataType.STIMULATION_DECISIONS: '/pipeline/decision_trace/final',
    ExportDataType.DECIDER_LOGS: '/pipeline/decider/log',
    ExportDataType.PREPROCESSOR_LOGS: '/pipeline/preprocessor/log',
    ExportDataType.PRESENTER_LOGS: '/pipeline/presenter/log',
    ExportDataType.SENSORY_STIMULI: '/pipeline/sensory_stimulus',
}

# Mapping from export data types to human-readable names
DATA_TYPE_TO_NAME = {
    ExportDataType.RAW_EEG: 'raw_eeg',
    ExportDataType.ENRICHED_EEG: 'enriched_eeg',
    ExportDataType.PREPROCESSED_EEG: 'preprocessed_eeg',
    ExportDataType.STIMULATION_DECISIONS: 'stimulation_decisions',
    ExportDataType.DECIDER_LOGS: 'decider_logs',
    ExportDataType.PREPROCESSOR_LOGS: 'preprocessor_logs',
    ExportDataType.PRESENTER_LOGS: 'presenter_logs',
    ExportDataType.SENSORY_STIMULI: 'sensory_stimuli',
}

# EEG export field definitions
EEG_FIELDS_RAW = [
    'sample_index',
    'time',
    'system_time_data_source_published',
    'is_session_start',
    'is_session_end',
    'pulse_trigger',
]

EEG_FIELDS_ENRICHED = EEG_FIELDS_RAW + [
    'in_rest',
    'paused',
    'experiment_time',
    'stage_name',
    'stage_index',
    'trial',
    'pulse_count',
]

EEG_FIELDS_PREPROCESSED = EEG_FIELDS_ENRICHED + [
    'valid',
    'preprocessor_duration',
]

# Mapping from topic to field list
TOPIC_TO_FIELDS = {
    '/eeg/raw': EEG_FIELDS_RAW,
    '/eeg/enriched': EEG_FIELDS_ENRICHED,
    '/eeg/preprocessed': EEG_FIELDS_PREPROCESSED,
}

# Decision trace export fields
DECISION_TRACE_FIELDS = [
    'decision_id',
    'status',
    'reference_sample_time',
    'reference_sample_index',
    'stimulate',
    'requested_stimulation_time',
    'decider_duration',
    'preprocessor_duration',
    'decision_path_latency',
    'system_time_decider_received',
    'system_time_decider_finished',
    'system_time_trigger_timer_received',
    'system_time_trigger_timer_finished',
    'system_time_hardware_fired',
    'sample_time_at_firing',
    'loopback_latency_at_firing',
    'latency_corrected_time_at_firing',
    'actual_stimulation_time',
    'actual_stimulation_sample_index',
    'timing_error',
    'pulse_confirmed',
    'pulse_confirmation_method',
]

# Status mapping for human-readable export
DECISION_TRACE_STATUS_MAP = {
    0: 'decided_no',
    1: 'decided_yes',
    2: 'scheduled',
    3: 'rejected',
    4: 'fired',
    5: 'pulse_observed',
    6: 'missed',
    7: 'error',
}

# Pulse confirmation method mapping for human-readable export
DECISION_TRACE_PULSE_CONFIRMATION_MAP = {
    0: 'none',
    1: 'eeg_trigger',
    2: 'trigger_timer_fire',
}

# Log export fields
LOG_FIELDS = [
    'sample_time',
    'is_initialization',
    'level',
    'message',
]

# Sensory stimulus export fields (base fields, parameters added dynamically)
SENSORY_STIMULUS_FIELDS = [
    'time',
    'type',
]


class SessionExporterNode(Node):
    def __init__(self):
        super().__init__('session_exporter')
        self.logger = self.get_logger()
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Export state
        self._export_thread = None
        self._export_lock = threading.Lock()
        self._cancel_flag = threading.Event()

        # Create state publisher (latched)
        state_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._state_publisher = self.create_publisher(
            ExporterState,
            '/session_exporter/state',
            state_qos
        )

        # Publish initial IDLE state
        self._publish_state(ExporterState.IDLE, '', 0.0)

        # Create service servers
        self.create_service(
            ExportSession,
            '/session/export',
            self.export_session_callback,
            callback_group=self.callback_group
        )
        
        self.create_service(
            CancelExport,
            '/session/cancel_export',
            self.cancel_export_callback,
            callback_group=self.callback_group
        )

        self.logger.info('Session Exporter initialized')

    def export_session_callback(self, request, response):
        """Handle export session service calls (short, spawns background thread)."""
        self.logger.info(f'Received export request for: {request.project_name}/{request.recording_name}')

        with self._export_lock:
            # Check if already exporting
            if self._export_thread is not None and self._export_thread.is_alive():
                self.logger.warn('Export already in progress, ignoring request')
                response.success = False
                return response

            # Validate request
            bag_path = self._find_bag_path(request.project_name, request.recording_name)
            if not bag_path:
                self.logger.error(f'Could not find recording: {request.recording_name} in project {request.project_name}')
                response.success = False
                return response

            # Build topic mapping for requested data types
            topics_to_export = {}
            for data_type in request.data_types:
                if data_type.value not in DATA_TYPE_TO_TOPIC:
                    self.logger.warn(f'Unknown data type: {data_type.value}, skipping')
                    continue
                topic = DATA_TYPE_TO_TOPIC[data_type.value]
                name = DATA_TYPE_TO_NAME[data_type.value]
                topics_to_export[topic] = name

            if not topics_to_export:
                response.success = False
                return response

            # Clear cancel flag and start export in background thread
            self._cancel_flag.clear()
            self._export_thread = threading.Thread(
                target=self._run_export,
                args=(bag_path, request.recording_name, topics_to_export),
                daemon=True
            )
            self._export_thread.start()

            response.success = True
            return response

    def cancel_export_callback(self, request, response):
        """Handle cancel export service calls."""
        self.logger.info('Received cancel export request')
        
        with self._export_lock:
            if self._export_thread is None or not self._export_thread.is_alive():
                response.success = False
                return response
            
            # Set cancel flag
            self._cancel_flag.set()
            response.success = True
            return response

    def _publish_state(self, state, recording_name, progress):
        """Publish exporter state."""
        msg = ExporterState()
        msg.state = state
        msg.recording_name = recording_name
        msg.progress = progress
        self._state_publisher.publish(msg)

    def _run_export(self, bag_path, recording_name, topics_to_export):
        """Run export in background thread."""
        try:
            # Create export directory
            export_dir = self._create_export_directory(bag_path)
            
            self.logger.info(f'Starting export of {len(topics_to_export)} topics')
            self._publish_state(ExporterState.EXPORTING, recording_name, 0.0)

            # Export all topics in a single pass
            exported_files = self._export_topics_single_pass(
                bag_path, recording_name, topics_to_export, export_dir
            )

            # Check if cancelled
            if self._cancel_flag.is_set():
                self.logger.info('Export cancelled')
                self._publish_state(ExporterState.IDLE, '', 0.0)
                return

            if not exported_files:
                self._publish_state(ExporterState.ERROR, recording_name, 0.0)
                self.logger.error('No data exported')
                return

            self._publish_state(ExporterState.IDLE, '', 0.0)
            self.logger.info(f'Export complete: {export_dir}, exported {len(exported_files)} file(s)')

        except Exception as e:
            self.logger.error(f'Export failed: {str(e)}')
            self._publish_state(ExporterState.ERROR, recording_name, 0.0)

    def _find_bag_path(self, project_name, recording_name):
        """Find the bag directory path from project and recording name."""
        # Construct the expected path
        bag_path = Path('/app/projects') / project_name / 'recordings' / recording_name
        
        if not bag_path.exists():
            self.logger.error(f'Recording directory does not exist: {bag_path}')
            return None
        
        if not bag_path.is_dir():
            self.logger.error(f'Recording path is not a directory: {bag_path}')
            return None
        
        # Verify there's at least one .mcap file
        mcap_files = list(bag_path.glob('*.mcap'))
        if not mcap_files:
            self.logger.error(f'No .mcap files found in: {bag_path}')
            return None
        
        return str(bag_path)

    def _create_export_directory(self, bag_path):
        """Create export directory next to the bag file."""
        bag_path = Path(bag_path)
        export_dir = bag_path.parent / f'{bag_path.name}_export'
        export_dir.mkdir(parents=True, exist_ok=True)
        return export_dir

    def _get_bag_duration(self, bag_path):
        """Get bag duration from session metadata JSON file."""
        json_path = Path(bag_path).parent / f'{Path(bag_path).name}.json'
        
        if not json_path.exists():
            self.logger.warn(f'Metadata file not found: {json_path}')
            return None
        
        try:
            with open(json_path, 'r') as f:
                metadata = json.load(f)
            
            if 'start_time' not in metadata or 'end_time' not in metadata:
                return None
            
            start = datetime.fromisoformat(metadata['start_time'])
            end = datetime.fromisoformat(metadata['end_time'])
            duration = (end - start).total_seconds()
            
            return duration
        except Exception as e:
            self.logger.warn(f'Failed to parse metadata: {e}')
            return None

    def _export_topics_single_pass(self, bag_path, recording_name, topics_to_export, export_dir):
        """Export multiple topics in a single pass through the bag."""
        # Get bag duration for progress calculation
        bag_duration = self._get_bag_duration(bag_path)
        
        # Set up bag reader
        storage_options = StorageOptions(uri=str(bag_path), storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic metadata
        topic_types = reader.get_all_topics_and_types()
        topic_type_map = {t.name: t.type for t in topic_types}

        # Set up writers and message types for each requested topic
        writers = {}
        msg_types = {}
        message_counts = {}
        
        for topic, name in topics_to_export.items():
            if topic not in topic_type_map:
                self.logger.warn(f'Topic {topic} not found in bag, skipping')
                continue
            
            msg_types[topic] = get_message(topic_type_map[topic])
            message_counts[topic] = 0
            
            # Initialize writer based on topic type
            output_file = export_dir / f'{name}.csv'
            
            if topic in ['/eeg/raw', '/eeg/enriched', '/eeg/preprocessed']:
                writers[topic] = self._create_eeg_writer(output_file, topic)
            elif topic == '/pipeline/decision_trace/final':
                writers[topic] = self._create_decision_trace_writer(output_file)
            elif topic in ['/pipeline/decider/log', '/pipeline/preprocessor/log', '/pipeline/presenter/log']:
                writers[topic] = self._create_log_writer(output_file)
            elif topic == '/pipeline/sensory_stimulus':
                writers[topic] = self._create_sensory_stimulus_writer(output_file)

        if not writers:
            self.logger.warn('No valid topics to export')
            return []

        # Single pass through the bag
        first_timestamp = None
        last_progress_update = 0.0
        progress_update_interval = 0.01  # Update every 1%
        
        while reader.has_next():
            # Check for cancellation
            if self._cancel_flag.is_set():
                self.logger.info('Export cancelled by user')
                # Close all open files
                for writer_info in writers.values():
                    writer_info['file'].close()
                return []
            
            topic_name, data, timestamp = reader.read_next()
            
            # Track first timestamp for progress calculation
            if first_timestamp is None:
                first_timestamp = timestamp
            
            # Update progress if we have bag duration
            if bag_duration is not None and bag_duration > 0:
                elapsed_ns = timestamp - first_timestamp
                elapsed_s = elapsed_ns / 1e9
                progress = min(elapsed_s / bag_duration, 1.0)
                
                # Publish progress updates at regular intervals
                if progress - last_progress_update >= progress_update_interval:
                    self._publish_state(
                        ExporterState.EXPORTING,
                        recording_name,
                        progress
                    )
                    last_progress_update = progress
            
            if topic_name not in writers:
                continue
            
            # Deserialize and write immediately
            msg = deserialize_message(data, msg_types[topic_name])
            writer_info = writers[topic_name]
            
            if topic_name in ['/eeg/raw', '/eeg/enriched', '/eeg/preprocessed']:
                self._write_eeg_message(writer_info, timestamp, msg)
            elif topic_name == '/pipeline/decision_trace/final':
                self._write_decision_trace_message(writer_info, timestamp, msg)
            elif topic_name in ['/pipeline/decider/log', '/pipeline/preprocessor/log', '/pipeline/presenter/log']:
                self._write_log_message(writer_info, timestamp, msg)
            elif topic_name == '/pipeline/sensory_stimulus':
                self._write_sensory_stimulus_message(writer_info, timestamp, msg)
            
            message_counts[topic_name] += 1

        # Close all writers
        exported_files = []
        for topic, writer_info in writers.items():
            writer_info['file'].close()
            count = message_counts[topic]
            self.logger.info(f'Exported {count} messages from {topic} to {writer_info["path"]}')
            if count > 0:
                exported_files.append(writer_info['path'])

        return exported_files

    def _create_eeg_writer(self, output_file, topic):
        """Create a CSV writer for EEG messages with fixed columns."""
        f = open(output_file, 'w', newline='')
        return {
            'file': f,
            'writer': None,  # Will be created after first message determines column count
            'path': output_file,
            'fieldnames': None,
            'topic': topic,
        }

    def _write_eeg_message(self, writer_info, timestamp, msg):
        """Write a single EEG message to CSV."""
        topic = writer_info['topic']
        
        # On first message, determine fixed columns based on topic type
        if writer_info['writer'] is None:
            # Get base fields from topic configuration
            fieldnames = TOPIC_TO_FIELDS[topic].copy()
            
            # Add EEG channels
            for i in range(len(msg.eeg)):
                fieldnames.append(f'eeg_{i}')
            
            # Add EMG channels
            for i in range(len(msg.emg)):
                fieldnames.append(f'emg_{i}')
            
            writer_info['fieldnames'] = fieldnames
            writer_info['writer'] = csv.DictWriter(writer_info['file'], fieldnames=fieldnames)
            writer_info['writer'].writeheader()
        
        # Build row from message attributes
        row_data = {}
        
        # Add all configured fields
        for field in TOPIC_TO_FIELDS[topic]:
            row_data[field] = getattr(msg, field)
        
        # Add channel data
        for i, val in enumerate(msg.eeg):
            row_data[f'eeg_{i}'] = val
        
        for i, val in enumerate(msg.emg):
            row_data[f'emg_{i}'] = val
        
        writer_info['writer'].writerow(row_data)

    def _create_decision_trace_writer(self, output_file):
        """Create a CSV writer for decision trace messages."""
        f = open(output_file, 'w', newline='')
        writer = csv.DictWriter(f, fieldnames=DECISION_TRACE_FIELDS)
        writer.writeheader()
        return {
            'file': f,
            'writer': writer,
            'path': output_file,
        }

    def _write_decision_trace_message(self, writer_info, timestamp, msg):
        """Write a single decision trace message to CSV."""
        row_data = {}
        for field in DECISION_TRACE_FIELDS:
            value = getattr(msg, field)
            # Convert session_id array to string representation
            if field == 'session_id':
                row_data[field] = ''.join(f'{b:02x}' for b in value)
            # Convert status to human-readable text
            elif field == 'status':
                row_data[field] = DECISION_TRACE_STATUS_MAP.get(value, f'unknown_{value}')
            # Convert pulse_confirmation_method to human-readable text
            elif field == 'pulse_confirmation_method':
                row_data[field] = DECISION_TRACE_PULSE_CONFIRMATION_MAP.get(value, f'unknown_{value}')
            else:
                row_data[field] = value
        writer_info['writer'].writerow(row_data)

    def _create_log_writer(self, output_file):
        """Create a CSV writer for log messages."""
        f = open(output_file, 'w', newline='')
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS)
        writer.writeheader()
        return {
            'file': f,
            'writer': writer,
            'path': output_file,
        }

    def _write_log_message(self, writer_info, timestamp, msg):
        """Write log messages to CSV."""
        # LogMessages contains a list of LogMessage
        for log_msg in msg.messages:
            row_data = {field: getattr(log_msg, field) for field in LOG_FIELDS}
            writer_info['writer'].writerow(row_data)

    def _create_sensory_stimulus_writer(self, output_file):
        """Create a CSV writer for sensory stimulus messages."""
        f = open(output_file, 'w', newline='')
        return {
            'file': f,
            'writer': None,  # Will be created after first message determines parameters
            'path': output_file,
            'fieldnames': None,
        }

    def _write_sensory_stimulus_message(self, writer_info, timestamp, msg):
        """Write a single sensory stimulus message to CSV."""
        # On first message, determine fixed columns based on parameter keys
        if writer_info['writer'] is None:
            fieldnames = SENSORY_STIMULUS_FIELDS.copy()
            
            # Add parameter columns
            param_keys = sorted([param.key for param in msg.parameters])
            for key in param_keys:
                fieldnames.append(f'param_{key}')
            
            writer_info['fieldnames'] = fieldnames
            writer_info['writer'] = csv.DictWriter(writer_info['file'], fieldnames=fieldnames)
            writer_info['writer'].writeheader()
        
        # Build row with base fields
        row_data = {field: getattr(msg, field) for field in SENSORY_STIMULUS_FIELDS}
        
        # Add parameters
        for param in msg.parameters:
            row_data[f'param_{param.key}'] = param.value
        
        writer_info['writer'].writerow(row_data)


def main(args=None):
    rclpy.init(args=args)
    node = SessionExporterNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
