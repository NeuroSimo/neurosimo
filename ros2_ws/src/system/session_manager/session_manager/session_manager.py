import rclpy
import re
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionClient

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from system_interfaces.msg import SessionState
from system_interfaces.srv import StartRecording, StopRecording, GetSessionConfig
from system_interfaces.msg import SessionConfig
from pipeline_interfaces.srv import (
    InitializeProtocol, FinalizeDecider, FinalizePreprocessor, FinalizePresenter,
    InitializeDecider, InitializePreprocessor, InitializePresenter,
    InitializeStimulationTracer, FinalizeStimulationTracer
)
from eeg_interfaces.action import InitializeSimulatorStream, InitializePlaybackStream
from eeg_interfaces.srv import InitializeEegDeviceStream, StartStreaming, StopStreaming
from eeg_interfaces.msg import StreamInfo, EegDeviceInfo

from std_msgs.msg import String
from std_srvs.srv import Trigger
from threading import Lock, Event, Thread, current_thread
import time
import uuid


class SessionManagerNode(Node):
    def __init__(self):
        super().__init__('session_manager')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Create service servers for session management
        self.create_service(
            Trigger,
            '/session/start',
            self.start_session_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            Trigger,
            '/session/abort',
            self.abort_session_callback,
            callback_group=self.callback_group
        )

        self.create_service(
            Trigger,
            '/session/finish',
            self.finish_session_callback,
            callback_group=self.callback_group
        )

        # Create latched publisher for session state
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.session_state_publisher = self.create_publisher(
            SessionState,
            '/session/state',
            state_qos,
            callback_group=self.callback_group
        )

        # Create clients for initialization operations
        self.protocol_init_client = self.create_client(
            InitializeProtocol, '/pipeline/protocol/initialize', callback_group=self.callback_group)
        self.preprocessor_init_client = self.create_client(
            InitializePreprocessor, '/pipeline/preprocessor/initialize', callback_group=self.callback_group)
        self.decider_init_client = self.create_client(
            InitializeDecider, '/pipeline/decider/initialize', callback_group=self.callback_group)
        self.presenter_init_client = self.create_client(
            InitializePresenter, '/pipeline/presenter/initialize', callback_group=self.callback_group)
        self.stimulation_tracer_init_client = self.create_client(
            InitializeStimulationTracer, '/pipeline/stimulation_tracer/initialize', callback_group=self.callback_group)
        self.simulator_stream_init_client = ActionClient(
            self, InitializeSimulatorStream, '/eeg_simulator/initialize', callback_group=self.callback_group)
        self.eeg_device_stream_init_client = self.create_client(
            InitializeEegDeviceStream, '/eeg_device/initialize', callback_group=self.callback_group)
        self.playback_stream_init_client = ActionClient(
            self, InitializePlaybackStream, '/playback/initialize', callback_group=self.callback_group)

        # Create clients for finalization operations
        self.preprocessor_finalize_client = self.create_client(
            FinalizePreprocessor, '/pipeline/preprocessor/finalize', callback_group=self.callback_group)
        self.decider_finalize_client = self.create_client(
            FinalizeDecider, '/pipeline/decider/finalize', callback_group=self.callback_group)
        self.presenter_finalize_client = self.create_client(
            FinalizePresenter, '/pipeline/presenter/finalize', callback_group=self.callback_group)
        self.stimulation_tracer_finalize_client = self.create_client(
            FinalizeStimulationTracer, '/pipeline/stimulation_tracer/finalize', callback_group=self.callback_group)

        # Create clients for session recording
        self.recording_start_client = self.create_client(
            StartRecording, '/session_recorder/start', callback_group=self.callback_group)
        self.recording_stop_client = self.create_client(
            StopRecording, '/session_recorder/stop', callback_group=self.callback_group)

        # Create clients for streaming start operations
        self.playback_streaming_start_client = self.create_client(
            # TODO: Replace with playback service once implemented
            StartStreaming, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_simulator_streaming_start_client = self.create_client(
            StartStreaming, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_device_streaming_start_client = self.create_client(
            StartStreaming, '/eeg_device/streaming/start', callback_group=self.callback_group)

        # Create clients for streaming stop operations
        self.playback_streaming_stop_client = self.create_client(
            # TODO: Replace with playback service once implemented
            StopStreaming, '/eeg_simulator/streaming/stop', callback_group=self.callback_group)
        self.eeg_simulator_streaming_stop_client = self.create_client(
            StopStreaming, '/eeg_simulator/streaming/stop', callback_group=self.callback_group)
        self.eeg_device_streaming_stop_client = self.create_client(
            StopStreaming, '/eeg_device/streaming/stop', callback_group=self.callback_group)

        # Create client for getting session config from session configurator
        self.get_session_config_client = self.create_client(
            GetSessionConfig, '/session_configurator/get_config', callback_group=self.callback_group)

        # Wait for all clients to be available
        action_clients = [
            (self.simulator_stream_init_client, '/eeg_simulator/initialize'),
# TODO: Check playback stream initialization client once implemented
#
#            (self.playback_stream_init_client, '/playback/initialize'),
        ]
        for client, topic in action_clients:
            while not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'Action {topic} not available, waiting...')

        service_clients = [
            (self.protocol_init_client, '/pipeline/protocol/initialize'),
            (self.preprocessor_init_client, '/pipeline/preprocessor/initialize'),
            (self.decider_init_client, '/pipeline/decider/initialize'),
            (self.presenter_init_client, '/pipeline/presenter/initialize'),
            (self.stimulation_tracer_init_client, '/pipeline/stimulation_tracer/initialize'),
            (self.eeg_device_stream_init_client, '/eeg_device/initialize'),
            (self.preprocessor_finalize_client, '/pipeline/preprocessor/finalize'),
            (self.decider_finalize_client, '/pipeline/decider/finalize'),
            (self.presenter_finalize_client, '/pipeline/presenter/finalize'),
            (self.stimulation_tracer_finalize_client, '/pipeline/stimulation_tracer/finalize'),
            (self.playback_streaming_start_client, '/eeg_simulator/streaming/start'),
            (self.eeg_simulator_streaming_start_client, '/eeg_simulator/streaming/start'),
            (self.eeg_device_streaming_start_client, '/eeg_device/streaming/start'),
            (self.playback_streaming_stop_client, '/eeg_simulator/streaming/stop'),
            (self.eeg_simulator_streaming_stop_client, '/eeg_simulator/streaming/stop'),
            (self.eeg_device_streaming_stop_client, '/eeg_device/streaming/stop'),
            (self.get_session_config_client, '/session_configurator/get_config'),
            (self.recording_start_client, '/session_recorder/start'),
            (self.recording_stop_client, '/session_recorder/stop'),
        ]
        for client, topic in service_clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {topic} not available, waiting...')

        # Session state
        self._session_thread = None
        self._thread_lock = Lock()
        self._stop_event = Event()

        # Publish initial idle state
        self.publish_session_state(False, SessionState.STOPPED)

        self.logger.info("Session Manager initialized")

    # Helper functions
    def publish_session_state(self, is_running, stage):
        """Publish current session state."""
        msg = SessionState()
        msg.is_running = is_running
        msg.stage = stage
        self.session_state_publisher.publish(msg)

    # Service callbacks
    def start_session_callback(self, request, response):
        """Handle start session service calls."""
        self.logger.info('Received start session request')

        with self._thread_lock:
            if self._session_thread is not None and self._session_thread.is_alive():
                self.logger.warn('Session already running, ignoring start request')
                response.success = False
                return response

            self._stop_event.clear()

            self._session_thread = Thread(target=self.run_session, daemon=True)
            self._session_thread.start()

            response.success = True
            self.logger.info('Session start request accepted')

        return response

    def abort_session_callback(self, request, response):
        """Handle abort session service calls."""
        self.logger.info('Received abort session request')

        with self._thread_lock:
            if self._session_thread is None or not self._session_thread.is_alive():
                self.logger.info('No session running to abort')
                response.success = False
                return response

            self._stop_event.set()

            response.success = True
            self.logger.info('Session abort request accepted')

        return response

    def finish_session_callback(self, request, response):
        """Handle finish session service calls."""
        self.logger.info('Received finish session request')

        with self._thread_lock:
            if self._session_thread is None or not self._session_thread.is_alive():
                self.logger.info('No session running to finish')
                response.success = False
                return response

            # TODO: Thus far, aborting (manually from the UI) and finishing (automatically when the
            #       protocol is complete) are the same operation. In the future, we may want to
            #       separate them.
            self._stop_event.set()

            response.success = True
            self.logger.info('Session finish request accepted')

        return response

    def run_session(self):
        """Run a complete session lifecycle."""
        session_id = list(uuid.uuid4().bytes)

        # Set session state to initializing
        self.publish_session_state(True, SessionState.INITIALIZING)

        # Fetch session parameters
        session_config = self.get_session_config()
        if session_config is None or not self.validate_session_config(session_config):
            self.logger.error('Failed to get or validate session parameters')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize data stream first to get stream info
        stream_info = self.initialize_stream(session_id, session_config)
        if stream_info is None:
            self.logger.error('Stream initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize protocol
        if not self.initialize_protocol(session_id, session_config):
            self.logger.error('Protocol initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize presenter.
        #
        # Note: Presenter must be initialized before decider to ensure that potential sensory stimuli sent by decider
        #       during initialization are properly queued.
        if not self.initialize_presenter(session_config, session_id, stream_info):
            self.logger.error('Presenter initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize decider
        if not self.initialize_decider(session_config, session_id, stream_info):
            self.logger.error('Decider initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize preprocessor
        if not self.initialize_preprocessor(session_config, session_id, stream_info):
            self.logger.error('Preprocessor initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Initialize stimulation tracer
        if not self.initialize_stimulation_tracer(session_id):
            self.logger.error('StimulationTracer initialization failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Start recording
        if not self.start_recording(session_id, session_config, stream_info):
            self.logger.error('Recording start failed')
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Start data streaming
        if not self.start_data_streaming(session_config, session_id):
            self.logger.error('Data streaming start failed')
            self.stop_recording(session_id)  # Clean up recording on failure
            self.publish_session_state(False, SessionState.STOPPED)
            return

        # Set session state to running
        self.publish_session_state(True, SessionState.RUNNING)

        # Run session loop
        while rclpy.ok() and not self._stop_event.wait(timeout=0.1):
            pass

        self.publish_session_state(False, SessionState.FINALIZING)

        # Stop data streaming first
        self.stop_data_streaming(session_config, session_id)

        # Stop recording
        self.stop_recording(session_id)

        # Finalize session components
        self.finalize_session_components(session_id)

        # Publish stopped state
        self.publish_session_state(False, SessionState.STOPPED)

        # Clear thread reference
        with self._thread_lock:
            if self._session_thread is current_thread():
                self._session_thread = None

    def get_session_config(self):
        """Get session configuration from session configurator."""
        request = GetSessionConfig.Request()
        response = self.call_service(self.get_session_config_client, request, '/session_configurator/get_config')

        if response is None:
            self.logger.error('Failed to get session config from session configurator')
            return None

        config = response.config
        self.logger.info(f'Fetched session config: subject={config.subject_id}, '
                        f'data_source={config.data_source}, '
                        f'protocol={config.protocol_filename}')

        return config

    def validate_session_config(self, config):
        """Validate session configuration."""
        # Subject ID validation: S[0-9][0-9][0-9]
        subject_id = config.subject_id
        if not re.match(r"^S[0-9]{3}$", subject_id):
            self.logger.error(f"Invalid subject ID: {subject_id}. Must be 'S' followed by 3 digits.")
            return False

        # Module validations: must end with .py if not empty
        if config.decider_module and not config.decider_module.endswith('.py'):
            self.logger.error(f"Invalid decider module: {config.decider_module}. Must end with '.py'.")
            return False
        if config.preprocessor_module and not config.preprocessor_module.endswith('.py'):
            self.logger.error(f"Invalid preprocessor module: {config.preprocessor_module}. Must end with '.py'.")
            return False
        if config.presenter_module and not config.presenter_module.endswith('.py'):
            self.logger.error(f"Invalid presenter module: {config.presenter_module}. Must end with '.py'.")
            return False

        # Experiment protocol validation: must end with .yml or .yaml
        protocol = config.protocol_filename
        if not (protocol.endswith('.yml') or protocol.endswith('.yaml')):
            self.logger.error(f"Invalid experiment protocol: {protocol}. Must end with '.yml' or '.yaml'.")
            return False

        # Data source validation
        data_source = config.data_source
        if data_source not in ["simulator", "playback", "eeg_device"]:
            self.logger.error(f"Invalid data source: {data_source}. Must be 'simulator', 'playback', or 'eeg_device'.")
            return False

        return True

    def initialize_stream(self, session_id, session_config):
        """Initialize the data stream source (simulator, device, or playback)."""
        data_source = session_config.data_source
        project_name = session_config.project_name
        self.logger.info(f'Initializing stream source: {data_source}')

        stream_info = StreamInfo()

        if data_source == 'simulator':
            dataset_filename = session_config.simulator_dataset_filename
            start_time = session_config.simulator_start_time

            goal = InitializeSimulatorStream.Goal()
            goal.session_id = session_id
            goal.project_name = project_name
            goal.dataset_filename = dataset_filename
            goal.start_time = start_time

            result = self.call_action(self.simulator_stream_init_client, goal, '/eeg_simulator/initialize')
            if result is None:
                return None
            stream_info = result.stream_info

        elif data_source == 'eeg_device':
            request = InitializeEegDeviceStream.Request()
            # Request is currently empty as per requirement
            
            response = self.call_service(self.eeg_device_stream_init_client, request, '/eeg_device/initialize')
            if response is None:
                return None
            stream_info = response.stream_info

        elif data_source == 'playback':
            # Playback initialization is dummy for now
            self.logger.info('Playback initialization (dummy)')
            stream_info = StreamInfo()
            stream_info.sampling_frequency = 0
            stream_info.num_eeg_channels = 0
            stream_info.num_emg_channels = 0

        else:
            self.logger.error(f'Unknown data source: {data_source}')
            return None

        self.logger.info(f'Stream initialized successfully. Stream info: {stream_info.sampling_frequency}Hz, {stream_info.num_eeg_channels} EEG channels')
        return stream_info

    def initialize_decider(self, session_config, session_id, stream_info):
        """Initialize the decider component."""
        request = InitializeDecider.Request()
        request.session_id = session_id
        request.stream_info = stream_info

        request.project_name = session_config.project_name
        request.subject_id = session_config.subject_id

        request.module_filename = session_config.decider_module
        request.enabled = session_config.decider_enabled

        request.preprocessor_enabled = session_config.preprocessor_enabled

        response = self.call_service(self.decider_init_client, request, '/pipeline/decider/initialize')

        if response is None or not response.success:
            self.logger.error('Decider initialization failed')
            return False

        self.logger.info('Decider initialized successfully')
        return True

    def initialize_preprocessor(self, session_config, session_id, stream_info):
        """Initialize the preprocessor component."""
        request = InitializePreprocessor.Request()
        request.session_id = session_id
        request.stream_info = stream_info

        request.project_name = session_config.project_name
        request.subject_id = session_config.subject_id

        request.module_filename = session_config.preprocessor_module
        request.enabled = session_config.preprocessor_enabled

        response = self.call_service(self.preprocessor_init_client, request, '/pipeline/preprocessor/initialize')

        if response is None or not response.success:
            self.logger.error('Preprocessor initialization failed')
            return False

        self.logger.info('Preprocessor initialized successfully')
        return True

    def initialize_presenter(self, session_config, session_id, stream_info):
        """Initialize the presenter component."""
        request = InitializePresenter.Request()
        request.session_id = session_id

        request.project_name = session_config.project_name
        request.subject_id = session_config.subject_id

        request.module_filename = session_config.presenter_module
        request.enabled = session_config.presenter_enabled

        response = self.call_service(self.presenter_init_client, request, '/pipeline/presenter/initialize')

        if response is None or not response.success:
            self.logger.error('Presenter initialization failed')
            return False

        self.logger.info('Presenter initialized successfully')
        return True

    def initialize_stimulation_tracer(self, session_id):
        """Initialize the stimulation tracer component."""
        request = InitializeStimulationTracer.Request()
        request.session_id = session_id

        response = self.call_service(self.stimulation_tracer_init_client, request, '/pipeline/stimulation_tracer/initialize')

        if response is None or not response.success:
            self.logger.error('StimulationTracer initialization failed')
            return False

        self.logger.info('StimulationTracer initialized successfully')
        return True

    def initialize_protocol(self, session_id, session_config):
        protocol_filename = session_config.protocol_filename
        project_name = session_config.project_name

        request = InitializeProtocol.Request()
        request.session_id = session_id
        request.project_name = project_name
        request.protocol_filename = protocol_filename

        response = self.call_service(self.protocol_init_client, request, '/pipeline/protocol/initialize')

        if response is None:
            return False

        if not response.success:
            self.logger.error('Protocol initialization failed')
            return False

        self.logger.info(f'Protocol initialized successfully')
        return True

    # Finalization functions
    def finalize_session_components(self, session_id):
        """Finalize all session components in reverse order."""
        # Finalize presenter
        try:
            request = FinalizePresenter.Request()
            request.session_id = session_id
            self.call_service(self.presenter_finalize_client, request, '/pipeline/presenter/finalize')
        except Exception as e:
            self.logger.error(f'Error finalizing presenter: {e}')

        # Finalize decider
        try:
            request = FinalizeDecider.Request()
            request.session_id = session_id
            self.call_service(self.decider_finalize_client, request, '/pipeline/decider/finalize')
        except Exception as e:
            self.logger.error(f'Error finalizing decider: {e}')

        # Finalize preprocessor
        try:
            request = FinalizePreprocessor.Request()
            request.session_id = session_id
            self.call_service(self.preprocessor_finalize_client, request, '/pipeline/preprocessor/finalize')
        except Exception as e:
            self.logger.error(f'Error finalizing preprocessor: {e}')

        # Finalize stimulation tracer
        try:
            request = FinalizeStimulationTracer.Request()
            request.session_id = session_id
            self.call_service(self.stimulation_tracer_finalize_client, request, '/pipeline/stimulation_tracer/finalize')
        except Exception as e:
            self.logger.error(f'Error finalizing stimulation tracer: {e}')

        self.logger.info(f'Session finalization completed')

    # Streaming functions
    def start_data_streaming(self, session_config, session_id):
        """Start the data streaming service."""
        data_source = session_config.data_source

        # Choose the appropriate streaming service based on data source
        if data_source == 'playback':
            client = self.playback_streaming_start_client
            service_name = '/playback/streaming/start'
        elif data_source == 'simulator':
            client = self.eeg_simulator_streaming_start_client
            service_name = '/eeg_simulator/streaming/start'
        elif data_source == 'eeg_device':
            client = self.eeg_device_streaming_start_client
            service_name = '/eeg_device/streaming/start'
        else:
            self.logger.error(f'Unknown data source: {data_source}')
            return False

        # Call the streaming service
        request = StartStreaming.Request()
        request.session_id = session_id
        response = self.call_service(client, request, service_name)

        if response is None:
            return False

        if not response.success:
            self.logger.error(f'Data streaming start failed: {response.message}')
            return False

        return True

    def stop_data_streaming(self, session_config, session_id):
        """Stop the data streaming service."""
        data_source = session_config.data_source

        # Choose the appropriate streaming stop service based on data source
        if data_source == 'playback':
            client = self.playback_streaming_stop_client
            service_name = '/eeg_simulator/streaming/stop'  # TODO: Replace with playback service once implemented
        elif data_source == 'simulator':
            client = self.eeg_simulator_streaming_stop_client
            service_name = '/eeg_simulator/streaming/stop'
        elif data_source == 'eeg_device':
            client = self.eeg_device_streaming_stop_client
            service_name = '/eeg_device/streaming/stop'
        else:
            self.logger.error(f'Unknown data source for stopping: {data_source}')
            return

        # Call the stop streaming service
        try:
            request = StopStreaming.Request()
            request.session_id = session_id
            self.call_service(client, request, service_name)
            self.logger.info(f'Stopped streaming for data source: {data_source}')
        except Exception as e:
            self.logger.error(f'Error stopping streaming for {data_source}: {e}')

    # Recording functions
    def start_recording(self, session_id, session_config, stream_info):
        """Start session recording."""
        request = StartRecording.Request()
        request.session_id = session_id

        request.config = session_config
        request.stream_info = stream_info

        response = self.call_service(self.recording_start_client, request, '/session_recorder/start')

        if response is None:
            return False

        if not response.success:
            self.logger.error('Recording start failed')
            return False
 
        self.logger.info('Recording started')
        return True

    def stop_recording(self, session_id):
        """Stop session recording."""
        request = StopRecording.Request()
        request.session_id = session_id

        response = self.call_service(self.recording_stop_client, request, '/session_recorder/stop')

        if response is None:
            self.logger.warn('Failed to stop recording (no response)')
            return

        if response.success:
            self.logger.info(f'Recording stopped: {response.bag_path}')
        else:
            self.logger.warn('Recording stop returned failure')

    def call_action(self, client, goal, action_name, timeout_sec=30.0):
        """Call an action."""
        if not client.wait_for_server(timeout_sec=1.0):
            return None

        # Send the goal
        send_goal_future = client.send_goal_async(goal)
        start_time = time.time()

        # Wait for goal acceptance
        while not send_goal_future.done():
            if self._stop_event.is_set():
                return None

            if time.time() - start_time > timeout_sec:
                self.logger.error(f'Action goal send to {action_name} timed out')
                return None

            time.sleep(0.01)

        try:
            goal_handle_response = send_goal_future.result()
            if not goal_handle_response.accepted:
                self.logger.error(f'Action goal to {action_name} was rejected')
                return None

            action_goal_handle = goal_handle_response
            result_future = action_goal_handle.get_result_async()

            # Wait for result
            while not result_future.done():
                if self._stop_event.is_set():
                    try:
                        action_goal_handle.cancel_goal_async()
                    except:
                        pass
                    return None

                if time.time() - start_time > timeout_sec:
                    self.logger.error(f'Action {action_name} timed out')
                    try:
                        action_goal_handle.cancel_goal_async()
                    except:
                        pass
                    return None

                time.sleep(0.01)

            result = result_future.result().result
            return result

        except Exception as e:
            self.logger.error(f'Action call to {action_name} failed: {e}')
            return None

    def call_service(self, client, request, service_name, timeout_sec=30.0):
        """Call a service."""
        if not client.wait_for_service(timeout_sec=5.0):
            return None

        future = client.call_async(request)
        start_time = time.time()

        # Poll the future
        while not future.done():
            if time.time() - start_time > timeout_sec:
                self.logger.error(f'Service call to {service_name} timed out')
                return None

            time.sleep(0.01)

        try:
            response = future.result()
            return response
        except Exception as e:
            self.logger.error(f'Service call to {service_name} failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SessionManagerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()