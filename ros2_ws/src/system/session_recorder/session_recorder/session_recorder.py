import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from system_interfaces.srv import StartRecording, StopRecording
from system_interfaces.msg import RecorderState, SessionConfig, DiskStatus
from rosidl_runtime_py.convert import message_to_ordereddict

import os
import json
import uuid
import signal
import subprocess
from datetime import datetime
import time


# Topics to record during a session
TOPICS_TO_RECORD = [
    # EEG data flow
    '/eeg/raw',
    '/eeg/enriched',
    '/eeg/preprocessed',

    # Pipeline outputs
    '/pipeline/loopback_latency',
    '/pipeline/experiment_state',
    '/pipeline/sensory_stimulus',

    # Pipeline logs
    '/pipeline/decider/log',
    '/pipeline/preprocessor/log',
    '/pipeline/presenter/log',

    # Session state
    '/session/state',

    # Decision traces
    '/pipeline/decision_trace',        # Mostly for debugging
    '/pipeline/decision_trace/final',

    # Heartbeats (for debugging)
    '/eeg_bridge/heartbeat',
    '/eeg_simulator/heartbeat',
    '/preprocessor/heartbeat',
    '/presenter/heartbeat',
    '/decider/heartbeat',
    '/experiment_coordinator/heartbeat',
    '/resource_monitor/heartbeat',
    '/trigger_timer/heartbeat',

    # Health (for debugging)
    '/eeg_bridge/health',
    '/eeg_simulator/health',
    '/preprocessor/health',
    '/presenter/health',
    '/decider/health',
    '/experiment_coordinator/health',
    '/resource_monitor/health',
    '/trigger_timer/health',
]


class SessionRecorderNode(Node):
    def __init__(self):
        super().__init__('session_recorder')
        self.logger = self.get_logger()
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Recording state
        self._is_recording = False
        self._current_session_id = None
        self._bag_path = None
        self._recording_config = None
        self._process = None
        self._monitor_timer = None

        # Disk status tracking
        self._disk_space_ok = False

        # Create service servers
        self.create_service(
            StartRecording,
            '/session_recorder/start',
            self.start_recording_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            StopRecording,
            '/session_recorder/stop',
            self.stop_recording_callback,
            callback_group=self.callback_group
        )

        # Create state publisher (latched)
        state_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._state_publisher = self.create_publisher(
            RecorderState,
            '/session_recorder/state',
            state_qos
        )

        # Publish initial STOPPED state
        self._publish_state(RecorderState.STOPPED)

        # Subscribe to disk status with latched QoS to match publisher
        disk_status_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._disk_status_subscriber = self.create_subscription(
            DiskStatus,
            '/system/disk_status',
            self._disk_status_callback,
            disk_status_qos
        )

        self.logger.info('Session Recorder initialized')

    def _disk_status_callback(self, msg):
        """Handle disk status updates."""
        self._disk_space_ok = msg.is_ok

    def _get_software_provenance(self):
        """Get software provenance information."""
        provenance = {}

        # Git commit
        provenance['git_commit'] = os.environ.get('GIT_COMMIT') or 'unknown'

        # Git state
        state = (os.environ.get('GIT_STATE') or 'unknown').strip().lower()
        if state not in ('clean', 'dirty', 'unknown'):
            state = 'unknown'
        provenance['git_state'] = state

        # Version = git tag (+ dirty suffix if applicable)
        tag = os.environ.get('GIT_TAG') or 'unknown'
        if tag != 'unknown' and state == 'dirty':
            provenance['version'] = f"{tag}-dirty"
        else:
            provenance['version'] = tag

        return provenance

    def _get_software_provenance(self):
        """Get software provenance information."""
        provenance = {}

        # Git commit
        provenance['git_commit'] = os.environ.get('GIT_COMMIT') or 'unknown'

        # Git state
        state = (os.environ.get('GIT_STATE') or 'unknown').strip().lower()
        if state not in ('clean', 'dirty', 'unknown'):
            state = 'unknown'
        provenance['git_state'] = state

        # Version = git tag (+ dirty suffix if applicable)
        tag = os.environ.get('GIT_TAG') or 'unknown'
        if tag != 'unknown' and state == 'dirty':
            provenance['version'] = f"{tag}-dirty"
        else:
            provenance['version'] = tag

        return provenance

    def start_recording_callback(self, request, response):
        """Handle start recording service calls."""
        self.logger.info('Received start recording request')

        if self._is_recording:
            self.logger.warn('Already recording, ignoring start request')
            response.success = False
            return response

        # Check disk status before allowing recording
        if not self._disk_space_ok:
            self.logger.warn('Disk space is insufficient, cannot start recording')
            response.success = False
            return response

        # Store session configuration
        self._current_session_id = bytes(request.session_id)
        session_uuid = uuid.UUID(bytes=self._current_session_id)

        start_time = datetime.now().isoformat()

        self._recording_config = {
            'session_id': str(session_uuid),
            'global_config': message_to_ordereddict(request.global_config),
            'session_config': message_to_ordereddict(request.session_config),
            'stream_info': message_to_ordereddict(request.stream_info),
            'provenance': {
                'software': self._get_software_provenance(),
                'fingerprints': {}
            },
            'timing': {
                'start_time': start_time,
                # end_time and duration will be filled in when recording stops
            },
        }

        # Create bag directory
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bag_name = f'{timestamp}_{request.session_config.subject_id}'
        project_bags_dir = f'/app/projects/{request.global_config.active_project}/recordings'

        os.makedirs(project_bags_dir, exist_ok=True)
        self._bag_path = os.path.join(project_bags_dir, bag_name)

        # Build ros2 bag record command
        cmd = [
            'ros2', 'bag', 'record',
            '--output', self._bag_path,
            '--storage', 'mcap',
        ]

        # Add topics
        for topic in TOPICS_TO_RECORD:
            cmd.append(topic)

        self.logger.info(f'Starting ros2 bag record: {" ".join(cmd)}')

        # Start the recording process
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,  # Create new process group for clean shutdown
        )

        # Wait briefly and check if process started successfully
        time.sleep(0.5)
        if self._process.poll() is not None:
            # Process exited immediately - read error
            _, stderr = self._process.communicate()
            error_msg = stderr.decode().strip() if stderr else 'Unknown error'
            self.logger.error(f'ros2 bag record failed to start: {error_msg}')
            response.success = False
            self._cleanup_recording()
            return response

        # Write initial session metadata
        self._write_session_metadata()

        # Start monitor timer (5 Hz)
        self._monitor_timer = self.create_timer(0.2, self._monitor_process_callback)

        # Publish RECORDING state now that process is confirmed alive
        self._publish_state(RecorderState.RECORDING)

        self._is_recording = True

        self.logger.info(f'Started recording to {self._bag_path}')
        response.success = True

        return response

    def stop_recording_callback(self, request, response):
        """Handle stop recording service calls."""
        self.logger.info('Received stop recording request')

        if not self._is_recording:
            self.logger.warn('Not currently recording')
            response.success = False
            response.bag_path = ''
            return response

        # Verify session ID matches
        session_id = bytes(request.session_id)
        if session_id != self._current_session_id:
            self.logger.warn('Session ID mismatch, ignoring stop request')
            response.success = False
            response.bag_path = ''
            return response

        # Record end time, duration, and fingerprints
        if self._recording_config:
            end_time = datetime.now().isoformat()

            self._recording_config['timing']['end_time'] = end_time

            # Compute duration
            start_dt = datetime.fromisoformat(start_time)
            end_dt = datetime.fromisoformat(end_time)
            self._recording_config['timing']['duration'] = (end_dt - start_dt).total_seconds()

            fingerprints = {}

            # Fingerprints from decider, preprocessor, and data source
            if request.decision_fingerprint != 0:
                fingerprints['decision'] = int(request.decision_fingerprint)
            if request.preprocessor_fingerprint != 0:
                fingerprints['preprocessor'] = int(request.preprocessor_fingerprint)
            if request.data_source_fingerprint != 0:
                fingerprints['data_source'] = int(request.data_source_fingerprint)

            self._recording_config['provenance']['fingerprints'] = fingerprints

        # Stop the recording process gracefully
        bag_path = self._bag_path
        self._stop_process()

        # Publish STOPPED state
        self._publish_state(RecorderState.STOPPED)

        # Write final session metadata
        self._write_session_metadata()

        self._cleanup_recording()

        self.logger.info(f'Stopped recording: {bag_path}')
        response.success = True
        response.bag_path = bag_path

        return response

    def _stop_process(self):
        """Stop the recording process gracefully."""
        process = self._process
        if process is None:
            return

        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            return

        def _signal_and_wait(sig, timeout):
            try:
                os.killpg(pgid, sig)
                process.wait(timeout=timeout)
                return True
            except subprocess.TimeoutExpired:
                return False
            except ProcessLookupError:
                return True

        # SIGINT: graceful shutdown
        if _signal_and_wait(signal.SIGINT, timeout=5.0):
            self.logger.info('Recording process stopped cleanly')
            return

        # SIGTERM: stronger request
        self.logger.warn('Recording process did not stop, sending SIGTERM')
        if _signal_and_wait(signal.SIGTERM, timeout=2.0):
            return

        # SIGKILL: last resort
        self.logger.warn('Recording process still running, sending SIGKILL')
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass

        process.wait()

    def _publish_state(self, state):
        """Publish recorder state."""
        msg = RecorderState()
        msg.state = state
        self._state_publisher.publish(msg)

    def _monitor_process_callback(self):
        """Monitor the recording process for unexpected exits."""
        if not self._is_recording or self._process is None:
            return

        # Check if process has exited
        if self._process.poll() is None:
            return

        # Process has exited unexpectedly
        exit_code = self._process.returncode
        self.logger.error(f'Recording process exited unexpectedly with code {exit_code}')

        # Publish ERROR state
        self._publish_state(RecorderState.ERROR)

        self._cleanup_recording()

    def _write_session_metadata(self):
        """Write session configuration to a sidecar JSON file."""
        if self._bag_path and self._recording_config:
            metadata_path = f'{self._bag_path}.json'
            with open(metadata_path, 'w') as f:
                json.dump(self._recording_config, f, indent=2)
            self.logger.info(f'Wrote session metadata to {metadata_path}')

    def _cleanup_recording(self):
        """Clean up recording state."""
        # Stop and cleanup monitor timer
        if self._monitor_timer:
            self._monitor_timer.cancel()
            self._monitor_timer = None

        self._process = None
        self._is_recording = False
        self._current_session_id = None
        self._bag_path = None
        self._recording_config = None


def main(args=None):
    rclpy.init(args=args)
    node = SessionRecorderNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
