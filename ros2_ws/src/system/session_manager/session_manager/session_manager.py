import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionClient

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from system_interfaces.msg import SessionState
from pipeline_interfaces.action import InitializeDecider, InitializePreprocessor, InitializePresenter
from pipeline_interfaces.srv import InitializeProtocol
from eeg_interfaces.action import InitializeSimulator
from eeg_interfaces.msg import EegInfo

from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from threading import Lock
import time


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
            '/session/stop',
            self.stop_session_callback,
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

        # Subscribe to active project topic
        project_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.active_project_subscriber = self.create_subscription(
            String,
            '/projects/active',
            self.active_project_callback,
            project_qos,
            callback_group=self.callback_group
        )

        # Subscribe to EEG device info
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.eeg_info_subscriber = self.create_subscription(
            EegInfo,
            '/eeg_device/info',
            self.eeg_info_callback,
            qos,
            callback_group=self.callback_group
        )

        # EEG device information
        self.eeg_info = EegInfo()

        # Create clients for initialization operations
        self.protocol_init_client = self.create_client(
            InitializeProtocol, '/pipeline/protocol/initialize', callback_group=self.callback_group)
        self.preprocessor_init_client = ActionClient(
            self, InitializePreprocessor, '/pipeline/preprocessor/initialize', callback_group=self.callback_group)
        self.decider_init_client = ActionClient(
            self, InitializeDecider, '/pipeline/decider/initialize', callback_group=self.callback_group)
        self.presenter_init_client = ActionClient(
            self, InitializePresenter, '/pipeline/presenter/initialize', callback_group=self.callback_group)

        # Create clients for streaming start operations
        self.playback_streaming_start_client = self.create_client(
            # TODO: Replace with playback service once implemented
            Trigger, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_simulator_streaming_start_client = self.create_client(
            Trigger, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_device_streaming_start_client = self.create_client(
            Trigger, '/eeg_device/streaming/start', callback_group=self.callback_group)

        # Create client for getting parameters from session configurator
        self.get_parameters_client = self.create_client(
            GetParameters, '/session_configurator/get_parameters', callback_group=self.callback_group)

        # Session state
        self.session_lock = Lock()
        self.session_running = False
        self.current_data_source = None
        self.current_project_name = None
        self.session_phase = ''

        # Session configuration (fetched from ROS parameters)
        self.session_config = {}

        # Publish initial idle state
        self.publish_session_state()

        self.logger.info("Session Manager initialized")

    def publish_session_state(self):
        """Publish current session state."""
        msg = SessionState()
        msg.is_running = self.session_running
        msg.phase = self.session_phase
        self.session_state_publisher.publish(msg)

    def start_session_callback(self, request, response):
        """Handle start session service calls."""
        self.logger.info('Received start session request')

        with self.session_lock:
            if self.session_running:
                self.logger.warn('Session already running, ignoring start request')
                response.success = False
                response.message = 'Session already running'
                return response

            # Start session in background thread
            import threading
            session_thread = threading.Thread(target=self.run_session)
            session_thread.daemon = True
            session_thread.start()

            response.success = True
            response.message = 'Session started'
            self.logger.info('Session start request accepted')

        return response

    def stop_session_callback(self, request, response):
        """Handle stop session service calls."""
        self.logger.info('Received stop session request')

        with self.session_lock:
            if not self.session_running:
                self.logger.info('No session running to stop')
                response.success = False
                response.message = 'No session running'
                return response

            # Signal session to stop
            self.session_running = False
            self.publish_session_state()

            response.success = True
            response.message = 'Session stop requested'
            self.logger.info('Session stop request accepted')

        return response

    def active_project_callback(self, msg):
        """Callback for active project topic updates."""
        self.current_project_name = msg.data
        self.logger.info(f"Active project updated: {self.current_project_name}")

    def eeg_info_callback(self, msg):
        """Callback for EEG device info updates."""
        self.eeg_info = msg
        self.logger.debug(f"EEG info updated: sampling_freq={msg.sampling_frequency}, eeg_channels={msg.num_eeg_channels}, emg_channels={msg.num_emg_channels}")

    def run_session(self):
        """Run a complete session lifecycle."""
        with self.session_lock:
            self.session_running = True
            self.session_phase = 'FETCHING_PARAMS'
            self.publish_session_state()

        try:
            # PHASE: FETCHING_PARAMS
            if not self.fetch_session_parameters():
                self.session_phase = ''
                self.session_running = False
                self.publish_session_state()
                return

            # PHASE: INITIALIZING
            self.session_phase = 'INITIALIZING'
            self.publish_session_state()
            if not self.initialize_session_components():
                self.session_phase = ''
                self.session_running = False
                self.publish_session_state()
                return

            # PHASE: STARTING_STREAMING
            self.session_phase = 'STARTING_STREAMING'
            self.publish_session_state()
            if not self.start_data_streaming():
                self.session_phase = ''
                self.session_running = False
                self.publish_session_state()
                return

            # PHASE: RUNNING
            self.session_phase = 'RUNNING'
            self.publish_session_state()
            self.run_session_loop()

        except Exception as e:
            self.logger.error(f'Error during session run: {str(e)}')
        finally:
            self.session_running = False
            self.session_phase = ''
            self.publish_session_state()

    def fetch_session_parameters(self):
        """Fetch session parameters from session configurator."""
        try:
            # Parameter names to fetch from session configurator
            param_names = [
                'notes', 'subject_id',
                'decider.module', 'decider.enabled',
                'preprocessor.module', 'preprocessor.enabled',
                'presenter.module', 'presenter.enabled',
                'experiment.protocol', 'data_source'
            ]

            # Create service request
            request = GetParameters.Request()
            request.names = param_names

            # Call service on session configurator
            response = self.call_service(request, self.get_parameters_client, '/session_configurator/get_parameters')

            if response is None:
                self.logger.error('Failed to get parameters from session configurator')
                return False

            # Extract parameter values
            # ROS2 parameter types: NOT_SET=0, BOOL=1, INTEGER=2, DOUBLE=3, STRING=4
            for i, param in enumerate(response.values):
                param_name = param_names[i]
                self.logger.debug(f"Received parameter {param_name}: type={param.type}, value={param}")
                if param.type == 1:  # BOOL
                    self.session_config[param_name] = param.bool_value
                elif param.type == 2:  # INTEGER
                    self.session_config[param_name] = param.integer_value
                elif param.type == 3:  # DOUBLE
                    self.session_config[param_name] = param.double_value
                elif param.type == 4:  # STRING
                    self.session_config[param_name] = param.string_value
                else:
                    self.logger.error(f'Unsupported parameter type {param.type} for {param_name}')
                    return False

            self.logger.info(f'Fetched session config: subject={self.session_config["subject_id"]}, '
                           f'data_source={self.session_config["data_source"]}, '
                           f'protocol={self.session_config["experiment.protocol"]}')

            return True

        except Exception as e:
            self.logger.error(f'Failed to fetch session parameters: {e}')
            return False

    def initialize_session_components(self):
        """Initialize all session components in sequence."""
        # Initialize protocol
        if not self.initialize_protocol():
            return False

        # Initialize preprocessor
        if not self.initialize_component(self.preprocessor_init_client, '/pipeline/preprocessor/initialize', 'preprocessor'):
            return False

        # Initialize decider
        if not self.initialize_component(self.decider_init_client, '/pipeline/decider/initialize', 'decider'):
            return False

        # Initialize presenter
        if not self.initialize_component(self.presenter_init_client, '/pipeline/presenter/initialize', 'presenter'):
            return False

        return True

    def initialize_component(self, client, action_name, component_name):
        """Initialize a single pipeline component."""
        if self.current_project_name is None:
            self.logger.error(f'Cannot initialize {component_name}: no active project set')
            return False

        # Get ROS parameters
        module_filename = self.session_config.get(f'{component_name}.module', '')
        enabled = self.session_config.get(f'{component_name}.enabled', False)
        subject_id = self.session_config.get('subject_id', '')

        if not module_filename:
            self.logger.error(f'Cannot initialize {component_name}: no module filename configured')
            return False

        if component_name == 'preprocessor':
            goal = InitializePreprocessor.Goal()
        elif component_name == 'decider':
            goal = InitializeDecider.Goal()
        elif component_name == 'presenter':
            goal = InitializePresenter.Goal()
        else:
            self.logger.error(f'Unknown component name: {component_name}')
            return False

        goal.project_name = self.current_project_name
        goal.module_filename = module_filename
        goal.enabled = enabled
        goal.subject_id = subject_id

        if component_name == 'preprocessor' or component_name == 'decider':
            goal.sampling_frequency = self.eeg_info.sampling_frequency
            goal.num_eeg_channels = self.eeg_info.num_eeg_channels
            goal.num_emg_channels = self.eeg_info.num_emg_channels

        if component_name == 'decider':
            goal.preprocessor_enabled = self.session_config.get('preprocessor.enabled', False)

        result = self.call_action(client, goal, action_name)

        if result is None:
            return False

        if not result.success:
            self.logger.error(f'{component_name} initialization failed')
            return False

        self.logger.info(f'{component_name} initialized successfully')
        return True

    def initialize_protocol(self):
        """Initialize protocol with project name and protocol filename."""
        if self.current_project_name is None:
            self.logger.error('No active project set')
            return False

        protocol_filename = self.session_config.get('experiment.protocol', '')
        if not protocol_filename:
            self.logger.error('No protocol filename configured')
            return False

        request = InitializeProtocol.Request()
        request.project_name = self.current_project_name
        request.protocol_filename = protocol_filename

        response = self.call_service(request, self.protocol_init_client, '/pipeline/protocol/initialize')

        if response is None:
            return False

        if not response.success:
            self.logger.error('Protocol initialization failed')
            return False

        self.logger.info(f'Protocol initialized successfully')
        return True

    def start_data_streaming(self):
        """Start the data streaming service."""
        data_source = self.session_config['data_source']

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
        request = Trigger.Request()
        response = self.call_service(request, client, service_name)

        if response is None:
            return False

        if not response.success:
            self.logger.error(f'Data streaming start failed: {response.message}')
            return False

        self.current_data_source = data_source
        return True

    def run_session_loop(self):
        """Run the main session loop until completion or cancellation."""
        while rclpy.ok() and self.session_running:
            time.sleep(0.1)  # Check every 100ms

    def call_action(self, client, goal, action_name, timeout_sec=30.0):
        """Call an action."""
        if not client.wait_for_server(timeout_sec=1.0):
            return None

        # Send the goal
        send_goal_future = client.send_goal_async(goal)
        start_time = time.time()

        # Wait for goal acceptance
        while not send_goal_future.done():
            if not self.session_running:
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
                if not self.session_running:
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

    def call_service(self, request, client, service_name, timeout_sec=30.0):
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