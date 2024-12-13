import time
from threading import Event, Lock

import numpy as np

from experiment_interfaces.msg import ExperimentState, TrialTiming, TrialConfig
from experiment_interfaces.action import PerformExperiment, PerformTrial
from experiment_interfaces.srv import ValidateTrial, CountValidTrials, PauseExperiment, ResumeExperiment, CancelExperiment, LogTrial

from std_msgs.msg import Bool

from mtms_device_interfaces.msg import SystemState, DeviceState

from system_interfaces.msg import Session, SessionState
from system_interfaces.srv import StartSession, StopSession

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class ExperimentPerformerNode(Node):

    ROS_ACTION_PERFORM_TRIAL = ('/trial/perform', PerformTrial)

    FIRST_TRIAL_TIME_S = 2.0
    TRIAL_REDO_INTERVAL_S = 3.0

    def __init__(self):
        super().__init__('experiment_performer_node')

        self.logger = self.get_logger()

        # Enable calling services within actions. See also MultiThreadExecutor.
        self.callback_group = ReentrantCallbackGroup()

        # Create action server for performing experiment.

        self.action_server = ActionServer(
            self,
            PerformExperiment,
            '/experiment/perform',
            self.perform_experiment_action_handler,
            callback_group=self.callback_group,
        )

        # Create service for counting valid trials in an experiment.

        self.count_valid_trials_service = self.create_service(
            CountValidTrials,
            '/experiment/count_valid_trials',
            self.count_valid_trials_callback,
            callback_group=self.callback_group,
        )

        # Create service for pausing an ongoing experiment.

        self.pause_experiment_service = self.create_service(
            PauseExperiment,
            '/experiment/pause',
            self.pause_experiment_callback,
            callback_group=self.callback_group,
        )

        # Create service for resuming an ongoing experiment.

        self.resume_experiment_service = self.create_service(
            ResumeExperiment,
            '/experiment/resume',
            self.resume_experiment_callback,
            callback_group=self.callback_group,
        )

        # Create service for canceling an ongoing experiment.

        self.cancel_experiment_service = self.create_service(
            CancelExperiment,
            '/experiment/cancel',
            self.cancel_experiment_callback,
            callback_group=self.callback_group,
        )

        # Create action client for performing a trial.

        topic, action_type = self.ROS_ACTION_PERFORM_TRIAL

        self.perform_trial_client = ActionClient(self, action_type, topic, callback_group=self.callback_group)
        while not self.perform_trial_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action {} not available, waiting...'.format(topic))

        # Create service client for validating a trial.

        self.validate_trial_client = self.create_client(ValidateTrial, '/trial/validate', callback_group=self.callback_group)
        while not self.validate_trial_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /trial/validate not available, waiting...')

        # Create service client for logging a trial.

        self.log_trial_client = self.create_client(LogTrial, '/trial/log', callback_group=self.callback_group)
        while not self.log_trial_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /trial/log not available, waiting...')

        # Create service client to start a session.

        self.start_session_client = self.create_client(StartSession, '/system/session/start', callback_group=self.callback_group)
        while not self.start_session_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /system/session/start not available, waiting...')

        # Create service client to stop a session.

        self.stop_session_client = self.create_client(StopSession, '/system/session/stop', callback_group=self.callback_group)
        while not self.stop_session_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /system/session/stop not available, waiting...')

        # Create subscriber for system state.
        self.system_state_subscriber = self.create_subscription(SystemState, '/mtms_device/system_state', self.handle_system_state, 1)
        self.system_state = None

        # Create subscriber for session.
        self.session_subscriber = self.create_subscription(Session, '/system/session', self.handle_session, 1)
        self.session = None

        # Subscriber for pedal.
        self.pedal_subscriber = self.create_subscription(Bool, "/pedal/right_button/pressed", self.handle_pedal_pressed, 10)
        self.is_pedal_pressed = False

        # Create a lock so that service and action calls can modify the experiment state concurrently.
        self.experiment_state_lock = Lock()
        self.experiment_state = ExperimentState.NOT_RUNNING

    # Experiment state

    def set_experiment_state(self, state):
        with self.experiment_state_lock:
            self.experiment_state = state

    def get_experiment_state(self):
        with self.experiment_state_lock:
            return self.experiment_state

    # ROS callbacks and callers

    # System state

    def handle_system_state(self, system_state):
        self.system_state = system_state

    def get_device_state(self):
        if self.system_state is None:
            return None

        return self.system_state.device_state.value

    def is_device_started(self):
        return self.get_device_state() == DeviceState.OPERATIONAL

    # Session

    def handle_session(self, session):
        self.session = session

    def get_session_state(self):
        if self.session is None:
            return None

        return self.session.state.value

    def is_session_started(self):
        return self.get_session_state() == SessionState.STARTED

    def is_session_stopped(self):
        return self.get_session_state() == SessionState.STOPPED

    def get_current_time(self):
        if self.session is None:
            return None

        return self.session.time

    # Other subscribers

    def handle_pedal_pressed(self, msg):
        state = msg.data

        if state:
            self.logger.info('Pedal pressed.')
            self.is_pedal_pressed = True
        else:
            self.logger.info('Pedal released.')
            self.is_pedal_pressed = False

    # Logging

    def log_experiment_config(self, goal_id, experiment):
        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: Experiment configuration:'.format(goal_id))

        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: Metadata:'.format(goal_id))
        self.logger.info('{}:   - Experiment name: {}'.format(goal_id, experiment.metadata.experiment_name))
        self.logger.info('{}:   - Subject name: {}'.format(goal_id, experiment.metadata.subject_name))
        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: Trials:'.format(goal_id))
        self.logger.info('{}:   - Total # of trials: {}'.format(goal_id, len(experiment.trials)))
        # TODO
        self.logger.info('{}:   - # of valid trials: {}'.format(goal_id, len(experiment.trials)))
        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: Intertrial interval:'.format(goal_id))
        self.logger.info('{}:   - Minimum: {}'.format(goal_id, experiment.intertrial_interval.min))
        self.logger.info('{}:   - Maximum: {}'.format(goal_id, experiment.intertrial_interval.max))
        self.logger.info('{}:'.format(goal_id))

    ## Asynchronous action and service callers

    def async_action_call(self, client, goal_msg):
        send_goal_event = Event()
        get_result_event = Event()

        goal_response = [None]
        action_result = [None]

        # Send goal to ROS action.
        def send_goal_callback(future):
            nonlocal send_goal_event
            nonlocal goal_response
            goal_response[0] = future.result()
            send_goal_event.set()

        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(send_goal_callback)

        # Wait for the goal to be sent
        send_goal_event.wait()

        goal_handle = goal_response[0]
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return None

        # Get result from ROS action.
        def get_result_done_callback(future):
            nonlocal get_result_event
            nonlocal action_result
            result_response = future.result()
            if result_response is None:
                self.get_logger().info('Action result failed.')
                action_result[0] = None
            else:
                action_result[0] = result_response.result
            get_result_event.set()

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(get_result_done_callback)

        return get_result_event, action_result

    def get_result_from_container(self, result_container):
        return result_container[0]

    def async_service_call(self, client, request):
        call_service_event = Event()
        response_value = [None]

        def service_call_callback(future):
            nonlocal response_value
            response_value[0] = future.result()
            call_service_event.set()

        service_call_future = client.call_async(request)
        service_call_future.add_done_callback(service_call_callback)

        # Wait for the service call to complete
        call_service_event.wait()

        response = response_value[0]
        return response

    # Action callers

    def async_perform_trial_action(self, trial):
        client = self.perform_trial_client
        goal = PerformTrial.Goal()

        goal.trial = trial

        event, result_container = self.async_action_call(client, goal)

        return event, result_container

    def sync_perform_trial_action(self, trial):
        event, result_container = self.async_perform_trial_action(
            trial=trial,
        )
        event.wait()

        result = self.get_result_from_container(result_container)

        return result

    # Service callers

    def start_session(self):
        request = StartSession.Request()

        response = self.async_service_call(self.start_session_client, request)

        assert response.success, "Starting session was not successful."

    def stop_session(self):
        request = StopSession.Request()

        response = self.async_service_call(self.stop_session_client, request)

        assert response.success, "Stopping session was not successful."

    def validate_trial(self, trial):
        request = ValidateTrial.Request()
        request.trial = trial

        response = self.async_service_call(self.validate_trial_client, request)

        assert response.success, "Validating trial was not successful."

        return response.is_trial_valid

    def log_trial(self, metadata, trial, trial_number, trial_result, num_of_attempts):
        request = LogTrial.Request()

        request.metadata = metadata
        request.trial = trial
        request.trial_number = trial_number
        request.trial_result = trial_result
        request.num_of_attempts = num_of_attempts

        response = self.async_service_call(self.log_trial_client, request)

        assert response.success, "Logging trial was not successful."

    # Performing experiment

    def perform_experiment_action_handler(self, goal_handle):
        request = goal_handle.request
        experiment = request.experiment

        metadata = experiment.metadata
        trials = experiment.trials
        intertrial_interval = experiment.intertrial_interval
        wait_for_pedal_press = experiment.wait_for_pedal_press
        randomize_trials = experiment.randomize_trials
        autopause = experiment.autopause
        autopause_interval = experiment.autopause_interval

        # Use short version of goal ID (2 first bytes as hex) for logging.
        #
        uuid = goal_handle.goal_id.uuid
        goal_id = "{:02x}{:02x}".format(uuid[0], uuid[1])

        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: New goal received: {}.'.format(goal_id, goal_id))

        # XXX: There should probably be an Experiment message so the request wouldn't have
        #   to be passed directly.
        self.log_experiment_config(
            goal_id=goal_id,
            experiment=experiment
        )

        valid_trials = self.get_valid_trials(
            goal_id=goal_id,
            trials=trials,
        )

        success, trial_results = self.perform_experiment(
            goal_handle=goal_handle,
            goal_id=goal_id,
            metadata=metadata,
            valid_trials=valid_trials,
            intertrial_interval=intertrial_interval,
            wait_for_pedal_press=wait_for_pedal_press,
            randomize_trials=randomize_trials,
            autopause=autopause,
            autopause_interval=autopause_interval,
        )

        # Create and return a Result object.

        result = PerformExperiment.Result()

        goal_handle.succeed()

        result.trial_results = trial_results
        result.success = success

        self.logger.info('{}: Done.'.format(goal_id))

        return result

    def pause_experiment_callback(self, request, response):
        # TODO: This service callback, as well as resuming and canceling an experiment, need to be
        #   more disciplined about the experiment that they affect. Overall, need to think about
        #   how to communicate state within the node: is it enough to use node-wide variables for
        #   the state, as long as only one experiment is going on at a time? Should starting
        #   several experiments at the same time be prevented? Probably these service calls should
        #   also check that there is an ongoing experiment in the first place. Before doing any of
        #   this, it might be worthwhile to wait until rosbridge supports actions, and first modify
        #   front-end to use PerformExperiment action instead of the equivalent service.
        #
        goal_id = "abcd"

        self.logger.info('{}:'.format(goal_id))

        if self.get_experiment_state() == ExperimentState.NOT_RUNNING:
            self.logger.error('{}: Trying to pause while experiment not running.'.format(goal_id))

            response.success = False
            return response

        self.logger.info('{}: Pausing the experiment after the current trial attempt...'.format(goal_id))

        self.set_experiment_state(ExperimentState.PAUSED)

        response.success = True
        return response

    def resume_experiment_callback(self, request, response):
        goal_id = "abcd"

        self.logger.info('{}:'.format(goal_id))

        if self.get_experiment_state() == ExperimentState.NOT_RUNNING:
            self.logger.error('{}: Trying to resume while experiment not running.'.format(goal_id))

            response.success = False
            return response

        self.logger.info('{}: Resuming the experiment.'.format(goal_id))

        self.set_experiment_state(ExperimentState.RUNNING)

        response.success = True
        return response

    def cancel_experiment_callback(self, request, response):
        goal_id = "abcd"

        self.logger.info('{}:'.format(goal_id))

        if self.get_experiment_state() == ExperimentState.NOT_RUNNING:
            self.logger.error('{}: Trying to cancel while experiment not running.'.format(goal_id))

            response.success = False
            return response

        self.logger.info('{}: Canceling the experiment after the current trial attempt...'.format(goal_id))

        self.set_experiment_state(ExperimentState.CANCELED)

        response.success = True
        return response

    def get_valid_trials(self, goal_id, trials):
        valid_trials = []
        for trial in trials:
            if self.validate_trial(trial):
                valid_trials.append(trial)

        self.logger.info('{}: {}/{} of the trials are valid.'.format(
            goal_id,
            len(valid_trials),
            len(trials),
        ))
        return valid_trials

    def count_valid_trials_callback(self, request, response):
        trials = request.trials

        num_of_trials = len(trials)
        self.logger.info('Received a request for counting valid trials out of {} trials'.format(
            num_of_trials
        ))

        # XXX: This is not very clean. ValidTrialCounter should probably be another ROS node?
        valid_trials = self.get_valid_trials('0000', trials)

        num_of_valid_trials = len(valid_trials)

        response.num_of_valid_trials = num_of_valid_trials
        response.success = True

        return response

    def check_experiment_feasible(self, goal_id, valid_trials):
        # Check that the mTMS device is started.
        if not self.is_device_started():
            self.logger.info('{}: mTMS device not started, aborting.'.format(goal_id))
            return False

        # Check that there is at least one valid trial.
        if len(valid_trials) == 0:
            self.logger.info('{}: None of the trials are valid, aborting.'.format(goal_id))
            return False

        return True

    def get_time_to_next_trial(self, num_of_attempts, is_first_trial, intertrial_interval):
        if num_of_attempts > 1:
            return self.TRIAL_REDO_INTERVAL_S

        # Start the first trial at FIRST_TRIAL_TIME_S seconds, otherwise follow intertrial interval.
        if is_first_trial:
            return self.FIRST_TRIAL_TIME_S
        else:
            return np.random.uniform(
                low=intertrial_interval.min,
                high=intertrial_interval.max,
            )

    def send_feedback(self, goal_handle, experiment_state, trial, num_of_attempts, trial_number, total_trials):
        feedback_msg = PerformExperiment.Feedback()

        feedback_msg.experiment_state = ExperimentState(value=experiment_state)
        feedback_msg.trial = trial
        feedback_msg.attempt_number = num_of_attempts
        feedback_msg.trial_number = trial_number
        feedback_msg.total_trials = total_trials

        goal_handle.publish_feedback(feedback_msg)

    def initialize_session(self, goal_id):

        # If session is already started, stop it first.
        if self.is_session_started():
            self.logger.info('{}: Session already started on the mTMS device, stopping...'.format(goal_id))
            self.stop_session()

            while not self.is_session_stopped():
                time.sleep(0.1)

        self.logger.info('{}: Starting session.'.format(goal_id))
        self.start_session()

        while not self.is_session_started():
            time.sleep(0.1)

    def finalize_session(self, goal_id):
        self.logger.info('{}: Stopping session...'.format(goal_id))
        self.stop_session()

        while not self.is_session_stopped():
            time.sleep(0.1)

    def perform_experiment(self, goal_handle, goal_id, metadata, valid_trials, intertrial_interval, wait_for_pedal_press, randomize_trials, autopause, autopause_interval):

        # Initialize experiment state
        self.set_experiment_state(ExperimentState.RUNNING)

        # Check that the experiment is feasible
        feasible = self.check_experiment_feasible(
            goal_id=goal_id,
            valid_trials=valid_trials
        )
        if not feasible:
            trial_results = []
            success = False

            return success, trial_results

        num_of_valid_trials = len(valid_trials)
        trial_results = []

        if randomize_trials is True:
            # XXX: Set seed to a constant value for now; think about how to properly make experiments
            #   reproducible but, e.g., not give the same trial sequence to all subjects. Should the
            #   seed be based on the experiment name and the subject name?
            np.random.seed(1234)
            valid_trials = np.random.permutation(valid_trials)

        self.initialize_session(goal_id)

        # Loop over trials and perform each.
        success = True
        last_resume_time = self.get_current_time()

        i = 0
        num_of_attempts = 0
        while i < num_of_valid_trials:
            num_of_attempts += 1

            trial = valid_trials[i]
            trial_number = i + 1

            # Check if autopause interval has passed.
            if autopause:
                current_time = self.get_current_time()
                if current_time > last_resume_time + autopause_interval:
                    self.set_experiment_state(ExperimentState.PAUSED)

            experiment_state = self.get_experiment_state()

            # Send feedback at the start of a new trial attempt.
            self.send_feedback(
                goal_handle=goal_handle,
                experiment_state=experiment_state,
                trial=trial,
                num_of_attempts=num_of_attempts,
                trial_number=trial_number,
                total_trials=num_of_valid_trials,
            )

            # Check if the experiment was paused, wait until resumed.
            if experiment_state == ExperimentState.PAUSED:
                self.logger.info('{}: Experiment paused...'.format(goal_id))
                while experiment_state == ExperimentState.PAUSED:
                    time.sleep(0.1)
                    experiment_state = self.get_experiment_state()

                self.logger.info('{}: Experiment resumed.'.format(goal_id))
                last_resume_time = self.get_current_time()

                # Re-send feedback after pausing has been finished.
                #
                # XXX: Is this needed? If it is, please document why.
                self.send_feedback(
                    goal_handle=goal_handle,
                    experiment_state=experiment_state,
                    trial=trial,
                    num_of_attempts=num_of_attempts,
                    trial_number=trial_number,
                    total_trials=num_of_valid_trials,
                )

            # Check if the experiment was canceled.
            if experiment_state == ExperimentState.CANCELED:
                self.logger.info('{}: Experiment canceled.'.format(goal_id))

                success = False
                break

            # The starting time of the first trial is handled differently, hence the check.
            is_first_trial = i == 0

            if wait_for_pedal_press:
                self.logger.info('{}: Waiting for a pedal press...'.format(goal_id))

                # Wait for a pedal press.
                while not (self.is_pedal_pressed or experiment_state == ExperimentState.CANCELED):
                    time.sleep(0.1)
                    experiment_state = self.get_experiment_state()

                # Check if the experiment was canceled.
                if experiment_state == ExperimentState.CANCELED:
                    self.logger.info('{}: Experiment canceled while waiting for pedal press.'.format(goal_id))

                    success = False
                    break

                time_to_next_trial = 0.0
            else:
                time_to_next_trial = self.get_time_to_next_trial(
                    num_of_attempts=num_of_attempts,
                    is_first_trial=is_first_trial,
                    intertrial_interval=intertrial_interval,
                )

            # Determine the time of the next trial.
            trial_time = self.get_current_time() + time_to_next_trial

            # When performing an (rTMS style) experiment, always allow late trials, as the exact timing of the pulse is unimportant.
            allow_late = True

            timing = TrialTiming(
                desired_start_time=trial_time,
                allow_late=allow_late,
            )

            self.logger.info('{}: Performing trial {} / {}, attempt number {}'.format(
                goal_id,
                i + 1,
                num_of_valid_trials,
                num_of_attempts,
            ))

            use_pulse_width_modulation_approximation = True if len(trial.targets) > 1 else False
            dry_run = False
            recharge_after_trial = True
            voltage_tolerance_proportion_for_precharging = 0.03

            config = TrialConfig(
                use_pulse_width_modulation_approximation=use_pulse_width_modulation_approximation,
                dry_run=dry_run,
                recharge_after_trial=recharge_after_trial,
                voltage_tolerance_proportion_for_precharging=voltage_tolerance_proportion_for_precharging,
            )

            trial.timing = timing
            trial.config = config

            result = self.sync_perform_trial_action(
                trial=trial,
            )
            trial_result = result.trial_result
            success = result.success

            if success:
                self.logger.info('{}: Successfully performed trial {} / {}.'.format(
                    goal_id,
                    i + 1,
                    num_of_valid_trials
                ))
                trial_results.append(trial_result)

                self.log_trial(
                    metadata=metadata,
                    trial=trial,
                    trial_number=trial_number,
                    trial_result=trial_result,
                    num_of_attempts=num_of_attempts,
                )

                i += 1
                num_of_attempts = 0
            else:
                self.logger.info('{}: Trial not successful, attempting again in {} seconds.'.format(
                    goal_id,
                    self.TRIAL_REDO_INTERVAL_S,
                ))

            # Add a delay to allow other ROS service calls to run.
            time.sleep(0.1)

        self.finalize_session(goal_id)

        return success, trial_results

def main(args=None):
    rclpy.init(args=args)

    experiment_performer_node = ExperimentPerformerNode()

    # Enable calling services within actions. See also ReentrantCallbackGroup.
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(experiment_performer_node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
