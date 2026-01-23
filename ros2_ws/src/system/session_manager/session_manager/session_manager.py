import os
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.srv import (
    ListProjects,
    SetActiveProject,
)
from project_interfaces.msg import FilenameList
from system_interfaces.action import RunSession
from pipeline_interfaces.action import InitializeComponent
from pipeline_interfaces.srv import InitializeProtocol
from eeg_interfaces.action import InitializeSimulator

from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from threading import Event, Lock
import time


class SessionManagerNode(Node):
    def __init__(self):
        super().__init__('session_manager')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Create action server for session management
        self._action_server = ActionServer(
            self,
            RunSession,
            '/session/run',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Create service server for finishing sessions
        self.create_service(
            Trigger,
            '/session/finish',
            self.finish_session_callback,
            callback_group=self.callback_group
        )

        # Subscribe to active project topic
        self.active_project_subscriber = self.create_subscription(
            String,
            '/projects/active',
            self.active_project_callback,
            1,
            callback_group=self.callback_group
        )

        # Create clients for initialization operations
        self.protocol_init_client = self.create_client(
            InitializeProtocol, '/pipeline/protocol/initialize', callback_group=self.callback_group)
        self.preprocessor_init_client = ActionClient(
            self, InitializeComponent, '/pipeline/preprocessor/initialize', callback_group=self.callback_group)
        self.decider_init_client = ActionClient(
            self, InitializeComponent, '/pipeline/decider/initialize', callback_group=self.callback_group)
        self.presenter_init_client = ActionClient(
            self, InitializeComponent, '/pipeline/presenter/initialize', callback_group=self.callback_group)
        self.simulator_init_client = ActionClient(
            self, InitializeSimulator, '/eeg/simulator/initialize', callback_group=self.callback_group)

        # Create clients for streaming start operations
        self.playback_streaming_start_client = self.create_client(
            # TODO: Replace with playback streaming start client once implemented
            Trigger, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_simulator_streaming_start_client = self.create_client(
            Trigger, '/eeg_simulator/streaming/start', callback_group=self.callback_group)
        self.eeg_device_streaming_start_client = self.create_client(
            Trigger, '/eeg_device/streaming/start', callback_group=self.callback_group)

        # Wait for all action servers and services to be available
        actions_to_wait = [
            (self.preprocessor_init_client, '/pipeline/preprocessor/initialize'),
            (self.decider_init_client, '/pipeline/decider/initialize'),
            (self.presenter_init_client, '/pipeline/presenter/initialize'),
            (self.simulator_init_client, '/eeg/simulator/initialize'),
        ]

        services_to_wait = [
            (self.protocol_init_client, '/pipeline/protocol/initialize'),
            (self.eeg_simulator_streaming_start_client, '/eeg_simulator/streaming/start'),
            (self.playback_streaming_start_client, '/playback/streaming/start'),
            (self.eeg_device_streaming_start_client, '/eeg_device/streaming/start'),
        ]

        self.logger.info("Waiting for all servers to be available...")
        for client, name in actions_to_wait:
            while not client.wait_for_server(timeout_sec=1.0):
                self.logger.warn(f"Action server {name} not available, waiting...")

        for client, name in services_to_wait:
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.warn(f"Service {name} not available, waiting...")

        # Session state
        self.session_lock = Lock()
        self.session_running = False
        self.current_goal_handle = None
        self.current_data_source = None
        self.finish_requested = False
        self.current_project_name = None

        # Session configuration (fetched from ROS parameters)
        self.session_config = {}

        self.logger.info("Session Manager initialized")

    def call_action_with_cancel_propagation(self, client, goal, goal_handle, action_name, timeout_sec=30.0):
        """Call an action while checking for and propagating cancellation requests."""
        if not client.wait_for_server(timeout_sec=1.0):
            return None

        # Send the goal
        send_goal_future = client.send_goal_async(goal)
        start_time = time.time()

        # Wait for goal acceptance
        while not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self.logger.info(f'Action goal send to {action_name} cancelled')
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

            # Wait for result while checking for cancellation
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    self.logger.info(f'Action {action_name} cancelled, propagating cancellation')
                    # Cancel the action goal
                    cancel_future = action_goal_handle.cancel_goal_async()
                    # Wait a bit for cancellation to complete but don't block indefinitely
                    cancel_start = time.time()
                    while not cancel_future.done() and time.time() - cancel_start < 2.0:
                        time.sleep(0.01)
                    return None

                if time.time() - start_time > timeout_sec:
                    self.logger.error(f'Action {action_name} timed out')
                    # Try to cancel the action
                    try:
                        action_goal_handle.cancel_goal_async()
                    except:
                        pass
                    return None

                time.sleep(0.01)

            result = result_future.result()
            return result

        except Exception as e:
            self.logger.error(f'Action call to {action_name} failed: {e}')
            return None

    def call_service_with_cancel_check(self, client, request, goal_handle, service_name, timeout_sec=30.0):
        """Call a service while checking for cancellation requests."""
        if not client.wait_for_service(timeout_sec=5.0):
            return None

        future = client.call_async(request)
        start_time = time.time()

        # Poll the future while checking for cancellation
        while not future.done():
            if goal_handle.is_cancel_requested:
                self.logger.info(f'Service call to {service_name} cancelled')
                # Cancel the future if possible
                if hasattr(future, 'cancel'):
                    future.cancel()
                return None

            # Check for timeout
            if time.time() - start_time > timeout_sec:
                self.logger.error(f'Service call to {service_name} timed out')
                if hasattr(future, 'cancel'):
                    future.cancel()
                return None

            time.sleep(0.01)  # Small sleep to avoid busy waiting

        try:
            response = future.result()
            return response
        except Exception as e:
            self.logger.error(f'Service call to {service_name} failed: {e}')
            return None

    def goal_callback(self, goal_request):
        """Accept or reject action goals."""
        self.logger.info('Received RunSession goal request')

        # Check for concurrent sessions
        with self.session_lock:
            if self.session_running:
                self.logger.warn('Rejecting goal: session already running')
                return GoalResponse.REJECT

        self.logger.info('Accepting RunSession goal')
        return GoalResponse.ACCEPT

    def active_project_callback(self, msg):
        """Callback for active project topic updates."""
        self.current_project_name = msg.data
        self.logger.info(f"Active project updated: {self.current_project_name}")

    def finish_session_callback(self, request, response):
        """Service callback to finish/stop an ongoing session."""
        self.logger.info('Received finish session request')

        with self.session_lock:
            if not self.session_running or self.current_goal_handle is None:
                self.logger.info('No ongoing session to finish')
                response.success = False
                response.message = 'No ongoing session'
                return response

            # Set the finish requested flag to signal graceful completion
            self.finish_requested = True
            response.success = True
            response.message = 'Session finish requested'
            self.logger.info('Session finish request accepted.')

        return response

    def cancel_callback(self, goal_handle):
        """Handle action cancellation."""
        self.logger.info('Received cancel request for session run')
        # Always accept cancellation - the execute_callback will handle stopping the session
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the session run action."""
        goal = goal_handle.request
        self.logger.info('Executing RunSession')

        result = RunSession.Result()

        try:
            success = self.run_session(goal_handle)
            result.success = success

        except Exception as e:
            self.logger.error(f'Error during session run: {str(e)}')
            result.success = False

        # Check if we were cancelled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.logger.info('Session run was cancelled')
        elif result.success:
            goal_handle.succeed()
            self.logger.info('Session run completed successfully')
        else:
            goal_handle.abort()
            self.logger.info('Session run failed')

        return result


    def run_session(self, goal_handle):
        """Run a complete session lifecycle."""
        goal = goal_handle.request
        self.current_goal_handle = goal_handle
        self.session_running = True
        self.finish_requested = False

        try:
            # PHASE: FETCHING_PARAMS
            self.publish_feedback(goal_handle, 'FETCHING_PARAMS')
            if not self.fetch_session_parameters(goal_handle):
                return False

            # Check for cancellation after parameter fetching
            if goal_handle.is_cancel_requested:
                self.logger.info('Session cancelled during parameter fetching')
                return False

            # PHASE: INITIALIZING
            self.publish_feedback(goal_handle, 'INITIALIZING')
            if not self.initialize_session_components(goal_handle, goal):
                return False

            # Check for cancellation after initialization
            if goal_handle.is_cancel_requested:
                self.logger.info('Session cancelled during initialization')
                self.cleanup_session()
                return False

            # PHASE: RUNNING
            self.publish_feedback(goal_handle, 'RUNNING')
            run_result = self.run_session_loop(goal_handle)

            # PHASE: FINALIZING
            self.publish_feedback(goal_handle, 'FINALIZING')
            self.cleanup_session()

            if run_result == 'cancelled':
                return False
            elif run_result == 'error':
                return False
            else:
                return True

        finally:
            self.session_running = False
            self.current_goal_handle = None
            self.finish_requested = False

    def fetch_session_parameters(self, goal_handle):
        """Fetch session parameters from ROS parameter server."""
        try:
            # Session parameters
            self.session_config['notes'] = self.get_parameter('notes').get_parameter_value().string_value
            self.session_config['subject_id'] = self.get_parameter('subject_id').get_parameter_value().string_value

            # Decider parameters
            self.session_config['decider.module'] = self.get_parameter('decider.module').get_parameter_value().string_value
            self.session_config['decider.enabled'] = self.get_parameter('decider.enabled').get_parameter_value().bool_value

            # Preprocessor parameters
            self.session_config['preprocessor.module'] = self.get_parameter('preprocessor.module').get_parameter_value().string_value
            self.session_config['preprocessor.enabled'] = self.get_parameter('preprocessor.enabled').get_parameter_value().bool_value

            # Presenter parameters
            self.session_config['presenter.module'] = self.get_parameter('presenter.module').get_parameter_value().string_value
            self.session_config['presenter.enabled'] = self.get_parameter('presenter.enabled').get_parameter_value().bool_value

            # Experiment parameters
            self.session_config['experiment.protocol'] = self.get_parameter('experiment.protocol').get_parameter_value().string_value

            # Simulator parameters
            self.session_config['simulator.dataset_filename'] = self.get_parameter('simulator.dataset_filename').get_parameter_value().string_value
            self.session_config['simulator.start_time'] = self.get_parameter('simulator.start_time').get_parameter_value().double_value

            # Playback parameters
            self.session_config['playback.bag_filename'] = self.get_parameter('playback.bag_filename').get_parameter_value().string_value
            self.session_config['playback.is_preprocessed'] = self.get_parameter('playback.is_preprocessed').get_parameter_value().bool_value

            # Data source parameter
            self.session_config['data_source'] = self.get_parameter('data_source').get_parameter_value().string_value

            self.logger.info(f'Fetched session config: subject={self.session_config["subject_id"]}, '
                           f'data_source={self.session_config["data_source"]}, '
                           f'protocol={self.session_config["experiment.protocol"]}')

            return True

        except Exception as e:
            self.logger.error(f'Failed to fetch session parameters: {e}')
            return False

    def publish_feedback(self, goal_handle, phase):
        """Publish feedback with current phase."""
        feedback = RunSession.Feedback()
        feedback.phase = phase
        goal_handle.publish_feedback(feedback)
        self.logger.info(f'Phase: {phase}')

    def initialize_session_components(self, goal_handle, goal):
        """Initialize all session components in sequence."""
        # Check for cancellation
        if goal_handle.is_cancel_requested:
            return False

        # Initialize protocol
        if not self.initialize_protocol(goal_handle):
            return False

        # Check for cancellation
        if goal_handle.is_cancel_requested:
            return False

        # Initialize preprocessor
        if not self.initialize_component(goal_handle, self.preprocessor_init_client, '/pipeline/preprocessor/initialize', 'preprocessor'):
            return False

        # Check for cancellation
        if goal_handle.is_cancel_requested:
            return False

        # Initialize decider
        if not self.initialize_component(goal_handle, self.decider_init_client, '/pipeline/decider/initialize', 'decider'):
            return False

        # Check for cancellation
        if goal_handle.is_cancel_requested:
            return False

        # Initialize presenter
        if not self.initialize_component(goal_handle, self.presenter_init_client, '/pipeline/presenter/initialize', 'presenter'):
            return False

        return True

    def initialize_component(self, goal_handle, client, action_name, component_name):
        """Initialize a single pipeline component."""
        if self.current_project_name is None:
            self.logger.error(f'Cannot initialize {component_name}: no active project set')
            return False

        # Get module filename and enabled status from ROS parameters
        module_filename = self.session_config.get(f'{component_name}.module', '')
        enabled = self.session_config.get(f'{component_name}.enabled', False)

        if not module_filename:
            self.logger.error(f'Cannot initialize {component_name}: no module filename configured')
            return False

        goal = InitializeComponent.Goal()
        goal.project_name = self.current_project_name
        goal.module_filename = module_filename
        goal.enabled = enabled

        result = self.call_action_with_cancel_propagation(client, goal, goal_handle, action_name)

        if result is None:
            return False

        if not result.success:
            self.logger.error(f'{component_name} initialization failed')
            return False

        self.logger.info(f'{component_name} initialized successfully: project={self.current_project_name}, module={module_filename}, enabled={enabled}')
        return True

    def initialize_protocol(self, goal_handle):
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

        response = self.call_service_with_cancel_check(
            self.protocol_init_client, request, goal_handle, '/pipeline/protocol/initialize')

        if response is None:
            return False

        if not response.success:
            self.logger.error('Protocol initialization failed')
            return False

        self.logger.info(f'Protocol initialized successfully: project={self.current_project_name}, protocol={protocol_filename}')
        return True

    def start_data_streaming(self, goal_handle, goal):
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
        response = self.call_service_with_cancel_check(client, request, goal_handle, service_name)

        if response is None:
            return False

        if not response.success:
            self.logger.error(f'Data streaming start failed: {response.message}')
            return False

        self.current_data_source = data_source
        return True

    def run_session_loop(self, goal_handle):
        """Run the main session loop until completion or cancellation."""
        start_time = time.time()
        while rclpy.ok() and not goal_handle.is_cancel_requested and not self.finish_requested:
            time.sleep(0.1)  # Check every 100ms

        if goal_handle.is_cancel_requested:
            return 'cancelled'
        elif self.finish_requested:
            self.logger.info('Session finished')
            return 'completed'
        else:
            return 'error'

    def cleanup_session(self):
        """Clean up session resources."""
        self.logger.info('Starting session cleanup...')

        self.logger.info('Session cleanup completed')


def main(args=None):
    rclpy.init(args=args)
    node = SessionManagerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
