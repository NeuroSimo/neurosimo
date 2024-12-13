import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from experiment_interfaces.action import PerformExperiment
from experiment_interfaces.srv import ValidateTrial, CancelExperiment, PauseExperiment, ResumeExperiment


class ExperimentHandler:
    ROS_ACTION_PERFORM_EXPERIMENT = ('/experiment/perform', PerformExperiment)

    ROS_SERVICE_CANCEL_EXPERIMENT = ('/experiment/cancel', CancelExperiment)
    ROS_SERVICE_PAUSE_EXPERIMENT = ('/experiment/pause', PauseExperiment)
    ROS_SERVICE_RESUME_EXPERIMENT = ('/experiment/resume', ResumeExperiment)

    ROS_SERVICE_VALIDATE_TRIAL = ('/trial/validate', ValidateTrial)

    ROS_SERVICES = (
        ROS_SERVICE_CANCEL_EXPERIMENT,
        ROS_SERVICE_PAUSE_EXPERIMENT,
        ROS_SERVICE_RESUME_EXPERIMENT,
        ROS_SERVICE_VALIDATE_TRIAL,
    )

    ROS_ACTIONS = (
        ROS_ACTION_PERFORM_EXPERIMENT,
    )

    def __init__(self, node):
        self.node = node
        self.logger = node.logger

        self.feedback = None
        self.result = None

        # Use ReentrantCallbackGroup to allow simultaneous actions and services.
        callback_group = ReentrantCallbackGroup()

        self.experiment_completed = False

        # Service clients

        self.ros_service_clients = {}

        for topic, service_type in self.ROS_SERVICES:
            client = self.node.create_client(service_type, topic, callback_group=callback_group)
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.info('Service {} not available, waiting...'.format(topic))

            self.ros_service_clients[topic] = client

        # Action clients
        self.ros_action_clients = {}

        for topic, action_type in self.ROS_ACTIONS:
            client = ActionClient(self.node, action_type, topic, callback_group=callback_group)
            while not client.wait_for_server(timeout_sec=1.0):
                self.logger.info('Action {} not available, waiting...'.format(topic))

            self.ros_action_clients[topic] = client

    # Internal callbacks
    def _feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error('Perform experiment goal rejected.')
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        self.experiment_completed = True

        self.result = future.result().result
        if self.result is None:
            self.logger.error('Perform experiment goal failed.')
            return

        success = self.result.success
        if success:
            self.logger.info('Experiment succeeded.')

    # Public methods
    def pause_experiment(self):
        topic, service_type = self.ROS_SERVICE_PAUSE_EXPERIMENT

        client = self.ros_service_clients[topic]
        request = service_type.Request()

        self.node.call_service(client, request)

    def resume_experiment(self):
        topic, service_type = self.ROS_SERVICE_RESUME_EXPERIMENT

        client = self.ros_service_clients[topic]
        request = service_type.Request()

        self.node.call_service(client, request)

    def cancel_experiment(self):
        topic, service_type = self.ROS_SERVICE_CANCEL_EXPERIMENT

        client = self.ros_service_clients[topic]
        request = service_type.Request()

        self.node.call_service(client, request)

    def validate_trial(self, trial):
        topic, service_type = self.ROS_SERVICE_VALIDATE_TRIAL

        client = self.ros_service_clients[topic]
        request = service_type.Request()
        request.trial = trial

        response = self.node.call_service(client, request)

        assert response.success, 'Trial validation failed.'

        return response.is_trial_valid

    def perform_experiment(self, experiment):
        """
        Perform an experiment by passing the experiment object.

        Parameters
        ----------
        experiment : object
            The experiment object.

        Returns
        -------
        object
            The result of the experiment.
        """
        topic, action_type = self.ROS_ACTION_PERFORM_EXPERIMENT

        client = self.ros_action_clients[topic]

        # Define goal
        goal = action_type.Goal()
        goal.experiment = experiment

        # Send goal asynchronously
        send_goal_future = client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def get_trial_results(self):
        if self.result is None:
            return

        return self.result.trial_results

    def print_feedback(self):
        if self.feedback is None:
            return

        self.node.printer.print_experiment_feedback(self.feedback)

    def get_feedback(self):
        return self.feedback

    def empty_feedback(self):
        self.feedback = None

    def wait_for_feedback(self):
        self.feedback = None
        try:
            while rclpy.ok() and self.feedback is None and not self.experiment_completed:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.logger.info('Keyboard interrupt.')

        return self.feedback

    def wait_for_completion(self):
        try:
            while rclpy.ok() and not self.experiment_completed:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.logger.info('Keyboard interrupt.')

    def is_done(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        return self.experiment_completed
