import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from eeg_interfaces.srv import InitializeEegReplayStream, StartStreaming, StopStreaming
from eeg_interfaces.msg import StreamInfo
from system_interfaces.msg import GlobalConfig, DataSourceState
from system_interfaces.srv import AbortSession

import os
import json
import subprocess
import signal
from pathlib import Path
from threading import Lock, Thread


class EegReplayNode(Node):
    def __init__(self):
        super().__init__('eeg_replay')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # State variables
        self.is_initialized = False
        self.bag_filepath = None
        self.play_preprocessed = False
        self.stream_info = StreamInfo()
        self.data_source_fingerprint = 0
        self.bag_process = None
        self.process_lock = Lock()
        self.stop_requested = False
        
        # QoS profile for latched topics
        qos_persist_latest = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publisher for data source state
        self.state_publisher = self.create_publisher(
            DataSourceState,
            '/eeg_replay/state',
            qos_persist_latest,
            callback_group=self.callback_group
        )

        # Client for aborting session
        self.abort_session_client = self.create_client(
            AbortSession,
            '/session/abort',
            callback_group=self.callback_group
        )

        # Subscribe to global config
        self.global_config = None
        self.global_config_subscription = self.create_subscription(
            GlobalConfig,
            '/global_configurator/config',
            self.global_config_callback,
            qos_persist_latest,
            callback_group=self.callback_group
        )

        # Service server for initialization
        self.initialize_service = self.create_service(
            InitializeEegReplayStream,
            '/eeg_replay/initialize',
            self.handle_initialize,
            callback_group=self.callback_group
        )

        # Service servers for streaming control
        self.start_streaming_service = self.create_service(
            StartStreaming,
            '/eeg_replay/streaming/start',
            self.handle_start_streaming,
            callback_group=self.callback_group
        )

        self.stop_streaming_service = self.create_service(
            StopStreaming,
            '/eeg_replay/streaming/stop',
            self.handle_stop_streaming,
            callback_group=self.callback_group
        )

        # Publish initial state
        self.publish_state(DataSourceState.READY)
        self.logger.info('EEG Replay Node initialized')

    def global_config_callback(self, msg):
        """Handle global config updates."""
        self.global_config = msg
        self.logger.info(f'Received global config: active_project={msg.active_project}')

    def publish_state(self, state):
        """Publish current data source state."""
        msg = DataSourceState()
        msg.state = state
        self.state_publisher.publish(msg)

    def handle_initialize(self, request, response):
        """Handle initialization service call."""
        self.logger.info(f'Initializing replay: project={request.project_name}, bag_id={request.bag_id}, play_preprocessed={request.play_preprocessed}')

        # Check if we have global config
        if self.global_config is None:
            self.logger.error('Global config not yet received')
            return response

        # Check if bag_id is provided
        if not request.bag_id:
            self.logger.error('bag_id is empty')
            return response

        # Construct paths
        # Recordings are stored as: projects/{project_name}/recordings/{bag_id}/
        # Metadata is stored as: projects/{project_name}/recordings/{bag_id}.json
        recordings_base = Path('projects') / request.project_name / 'recordings'
        bag_path = recordings_base / request.bag_id
        metadata_path = recordings_base / f'{request.bag_id}.json'
        
        if not bag_path.exists():
            self.logger.error(f'Recording path does not exist: {bag_path}')
            return response

        if not metadata_path.exists():
            self.logger.error(f'Metadata file not found: {metadata_path}')
            return response

        self.bag_filepath = str(bag_path)
        self.play_preprocessed = request.play_preprocessed
        
        # Read stream info from metadata.json

        try:
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
            
            # Extract stream info from metadata
            stream_info_dict = metadata.get('stream_info', {})
            self.stream_info.sampling_frequency = stream_info_dict.get('sampling_frequency', 0)
            self.stream_info.num_eeg_channels = stream_info_dict.get('num_eeg_channels', 0)
            self.stream_info.num_emg_channels = stream_info_dict.get('num_emg_channels', 0)

            # Extract data source fingerprint from metadata
            provenance = metadata.get('provenance', {})
            fingerprints = provenance.get('fingerprints', {})
            self.data_source_fingerprint = fingerprints.get('data_source', 0)

            self.logger.info(f'Loaded stream info: {self.stream_info.sampling_frequency}Hz, '
                           f'{self.stream_info.num_eeg_channels} EEG channels, '
                           f'{self.stream_info.num_emg_channels} EMG channels')
            self.logger.info(f'Data source fingerprint: 0x{self.data_source_fingerprint:016x}')

        except json.JSONDecodeError as e:
            self.logger.error(f'Failed to parse metadata JSON: {e}')
            return response
        except Exception as e:
            self.logger.error(f'Error reading metadata: {e}')
            return response

        self.is_initialized = True
        self.publish_state(DataSourceState.READY)

        response.stream_info = self.stream_info

        self.logger.info(f'Replay initialized successfully: {self.bag_filepath} (topic: {"/eeg/preprocessed" if self.play_preprocessed else "/eeg/enriched"})')
        return response

    def handle_start_streaming(self, request, response):
        """Handle start streaming service call."""
        self.logger.info('Received start streaming request')

        if not self.is_initialized:
            self.logger.error('Not initialized, cannot start streaming')
            response.success = False
            return response

        with self.process_lock:
            if self.bag_process is not None:
                self.logger.warning('Bag playback already running')
                response.success = False
                return response

            # Determine which EEG topic to play back
            eeg_topic = '/eeg/preprocessed' if self.play_preprocessed else '/eeg/enriched'

            # Always also replay experiment UI state for the frontend
            topics = [
                eeg_topic,
                '/pipeline/experiment_state',
            ]

            # Build ros2 bag play command
            cmd = [
                'ros2', 'bag', 'play',
                self.bag_filepath,
                '--topics',
                *topics,
                '--clock', '200'  # Publish clock at 200 Hz
            ]

            try:
                self.logger.info(f'Starting bag playback: {" ".join(cmd)}')
                self.stop_requested = False
                self.bag_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Create new process group for proper cleanup
                )
                # Start background monitor thread to detect natural end of bag
                Thread(target=self._monitor_bag_process, daemon=True).start()
                self.publish_state(DataSourceState.RUNNING)
                response.success = True
                self.logger.info('Bag playback started successfully')
            except Exception as e:
                self.logger.error(f'Failed to start bag playback: {e}')
                self.bag_process = None
                response.success = False

        return response

    def handle_stop_streaming(self, request, response):
        """Handle stop streaming service call."""
        self.logger.info('Received stop streaming request')

        with self.process_lock:
            if self.bag_process is None:
                self.logger.warning('No bag playback running')
                response.success = False
                response.data_source_fingerprint = 0
                return response

            try:
                # Mark that this stop was requested explicitly to avoid aborting session
                self.stop_requested = True
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                
                # Wait for process to terminate (with timeout)
                try:
                    self.bag_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.logger.warning('Bag process did not terminate, sending SIGKILL')
                    os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
                    self.bag_process.wait()

                self.bag_process = None
                self.publish_state(DataSourceState.READY)
                response.success = True
                response.data_source_fingerprint = self.data_source_fingerprint
                self.logger.info(f'Bag playback stopped successfully (fingerprint: 0x{self.data_source_fingerprint:016x})')
            except Exception as e:
                self.logger.error(f'Failed to stop bag playback: {e}')
                response.success = False
                response.data_source_fingerprint = 0

        return response

    def _monitor_bag_process(self):
        """Background thread that waits for bag playback to finish and aborts session if it ends naturally."""
        with self.process_lock:
            process = self.bag_process

        if process is None:
            return

        # Wait for the bag process to exit
        retcode = process.wait()

        with self.process_lock:
            # If the process handle has changed (e.g. restarted) or was cleared, do nothing
            if process is not self.bag_process:
                return

            self.bag_process = None

            # If stop was requested explicitly, just transition to READY without aborting the session
            if self.stop_requested:
                self.logger.info(f'Bag playback stopped manually with return code {retcode}')
                self.publish_state(DataSourceState.READY)
                return

        # Natural end of bag playback: mark READY and abort session
        self.logger.info(f'Bag playback finished with return code {retcode}, aborting session')
        self.publish_state(DataSourceState.READY)
        self.abort_session()

    def abort_session(self):
        """Request session abort via AbortSession service."""
        if not self.abort_session_client.wait_for_service(timeout_sec=1.0):
            self.logger.error('AbortSession service not available, cannot abort session')
            return

        request = AbortSession.Request()
        request.source = 'eeg_replay'

        self.abort_session_client.call_async(request)
        self.logger.info('Requested session abort: bag playback finished')

    def cleanup(self):
        """Cleanup on node shutdown."""
        with self.process_lock:
            if self.bag_process is not None:
                try:
                    os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                    self.bag_process.wait(timeout=5.0)
                except:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = EegReplayNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
