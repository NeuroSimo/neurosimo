import time
from threading import Event

import numpy as np

from eeg_interfaces.msg import EegInfo, GatherEegError
from eeg_interfaces.action import GatherEeg

from mep_interfaces.msg import AnalyzeMepErrors, MepError
from mep_interfaces.action import AnalyzeMep
from mep_interfaces.srv import AnalyzeMepService

from system_interfaces.msg import Healthcheck, HealthcheckStatus

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class GatheredEegBuffer():
    def __init__(self):
        self.eeg_buffer = None
        self.error = None
        self.event = Event()

    def wait(self):
        # Ensure that we terminate properly in case of ctrl-c (SIGINT) or another signal;
        # therefore, do not use self.event.wait(), which may block indefinitely. Also sleep
        # for a short time to avoid busy-looping.
        #
        while rclpy.ok() and not self.event.is_set():
            time.sleep(0.01)

    def set(self):
        self.event.set()


class AnalyzeMepNode(Node):

    ROS_ACTION_GATHER_EEG = ('/eeg/gather', GatherEeg)

    def __init__(self):
        super().__init__('analyze_mep_node')

        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Create action client for gathering EEG data.

        topic, action_type = self.ROS_ACTION_GATHER_EEG

        client = ActionClient(self, action_type, topic, callback_group=self.callback_group)
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action {} not available, waiting...'.format(topic))

        self.gather_eeg_client = client

        # Create action server for analyzing MEP.

        self.action_server = ActionServer(
            self,
            AnalyzeMep,
            '/mep/analyze',
            self.analyze_mep_action_handler,
            callback_group=self.callback_group,
        )

        # HACK: MATLAB does not currently (version R2022b) support ROS2 actions. Work around
        #   it by creating a service, which produces the same output as the action.
        #
        self.analyze_mep_service = self.create_service(
            AnalyzeMepService,
            '/mep/analyze_service',
            self.analyze_mep_service_handler,
            callback_group=self.callback_group,
        )

        # Create subscriber for EEG healthcheck
        self.healthcheck_publisher = self.create_publisher(
            Healthcheck,
            '/mep/healthcheck',
            10,
            callback_group=self.callback_group,
        )

        self.eeg_healthcheck_subscriber = self.create_subscription(
            Healthcheck,
            '/eeg/healthcheck',
            self.eeg_healthcheck_handler,
            10,
            callback_group=self.callback_group,
        )

    # Healthcheck

    def publish_healthcheck(self):
        msg = Healthcheck()
        if self.eeg_available:
            msg.status.value = msg.status.READY
            msg.status_message = ""
            msg.actionable_message = ""
        else:
            msg.status.value = msg.status.DISABLED
            msg.status_message = "EEG not available"
            msg.actionable_message = ""

        self.healthcheck_publisher.publish(msg)

    def eeg_healthcheck_handler(self, msg):
        self.eeg_available = msg.status.value == msg.status.READY
        self.publish_healthcheck()

    # ROS callbacks and callers
    def async_gather_eeg(self, goal_id, start_time, end_time, gathered_eeg_buffer):
        topic, action_type = self.ROS_ACTION_GATHER_EEG

        client = self.gather_eeg_client

        # Define goal.
        goal = action_type.Goal()

        goal.time_window.start = start_time
        goal.time_window.end = end_time

        # Send goal to ROS action.
        send_goal_event = Event()

        def send_goal_callback(future):
            nonlocal send_goal_event
            send_goal_event.set()

        send_goal_future = client.send_goal_async(goal)

        send_goal_future.add_done_callback(send_goal_callback)
        send_goal_event.wait()

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Gather EEG goal rejected.')
            return None

        # Get result from ROS action.
        get_result_future = goal_handle.get_result_async()

        def get_result_done_callback(future):
            nonlocal gathered_eeg_buffer

            result = future.result()
            if result is None:
                self.get_logger().info('Gather EEG result failed.')
                return None

            eeg_buffer = result.result.eeg_buffer
            error = result.result.error

            gathered_eeg_buffer.eeg_buffer = eeg_buffer
            gathered_eeg_buffer.error = error

            if error.value != GatherEegError.NO_ERROR:
                self.get_logger().warn('{}: Failure: Error while gathering data from {:.3f} to {:.3f} (s), error code: {}.'.format(goal_id, start_time, end_time, error.value))

            gathered_eeg_buffer.set()

        get_result_future.add_done_callback(get_result_done_callback)

    def async_gather_eeg_in_time_window(self, goal_id, baseline_time, time_window, gathered_eeg_buffer):
        start_time = baseline_time + time_window.start
        end_time = baseline_time + time_window.end

        self.async_gather_eeg(
            goal_id=goal_id,
            start_time=start_time,
            end_time=end_time,
            gathered_eeg_buffer=gathered_eeg_buffer,
        )

    # Logging

    def log_mep_configuration(self, goal_id, mep_configuration):
        time_window = mep_configuration.time_window

        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: Configuration:'.format(goal_id))
        self.logger.info('{}:   - MEP time window: {:.1f} to {:.1f} (ms) after stimulation.'.format(goal_id, 1000 * time_window.start, 1000 * time_window.end))

        preactivation_check = mep_configuration.preactivation_check
        self.logger.info('{}:   - Preactivation check {}'.format(goal_id, "enabled:" if preactivation_check.enabled else "disabled."))

        if preactivation_check.enabled:
            time_window = preactivation_check.time_window
            self.logger.info('{}:     - Time window: {:.1f} to {:.1f} (ms) before stimulation.'.format(goal_id, 1000 * (-time_window.end), 1000 * (-time_window.start)))
            self.logger.info('{}:     - Voltage range limit: {:.1f} \u03BCV.'.format(goal_id, preactivation_check.voltage_range_limit))

        self.logger.info('{}:'.format(goal_id))

    # MEP analysis

    def validate_emg_channel(self, goal_id, emg_channel, eeg_buffer):

        # Assuming that all datapoints have the same number of channels.
        num_of_emg_channels = len(eeg_buffer[0].emg_data)

        # EMG channel indexing starts from 0, hence decrement.
        maximum_channel_index = num_of_emg_channels - 1

        if emg_channel > maximum_channel_index:
            self.logger.warn('{}: Failure: Requested channel ({}) larger than the maximum channel ({}). Please check the channel counts defined in .env.'.format(
                goal_id,
                emg_channel,
                maximum_channel_index,
            ))
            return MepError(value=MepError.INVALID_EMG_CHANNEL)

        return MepError(value=MepError.NO_ERROR)

    def get_channel_from_eeg_buffer(self, emg_channel, eeg_buffer):
        channel_data = np.array([d.emg_data[emg_channel] for d in eeg_buffer])
        timestamps = np.array([d.time for d in eeg_buffer])

        return channel_data, timestamps

    def check_preactivation(self, goal_id, mep_configuration, emg_channel, preactivation_buffer):

        channel_data, timestamps = self.get_channel_from_eeg_buffer(
            emg_channel=emg_channel,
            eeg_buffer=preactivation_buffer,
        )

        voltage_range = np.max(channel_data) - np.min(channel_data)
        voltage_range_limit = mep_configuration.preactivation_check.voltage_range_limit

        if voltage_range > voltage_range_limit:
           self.logger.warn('{}: Failure: Preactivation check failed, the voltage range ({:.1f} \u03BCV) is above the limit ({:.1f} \u03BCV).'.format(goal_id, voltage_range, voltage_range_limit))
           return MepError(value=MepError.PREACTIVATION_FAILED)

        self.logger.info('{}: Preactivation check passed.'.format(goal_id))

        return MepError(value=MepError.NO_ERROR)

    def compute_mep_amplitude_and_latency(self, goal_id, time, emg_channel, mep_buffer):

        channel_data, timestamps = self.get_channel_from_eeg_buffer(
            emg_channel=emg_channel,
            eeg_buffer=mep_buffer,
        )
        max_i = np.argmax(channel_data)
        min_i = np.argmin(channel_data)

        amplitude = channel_data[max_i] - channel_data[min_i]
        latency = timestamps[max_i] - time

        self.logger.info('{}: Computed MEP with amplitude {:.1f} (\u03BCV) and latency {:.1f} (ms).'.format(goal_id, amplitude, 1000 * latency))

        return channel_data, amplitude, latency

    def analyze_mep(self, goal_id, time, mep_configuration):
        self.logger.info('{}: Analyzing MEP at time: {:.3f} (s)'.format(goal_id, time))

        self.log_mep_configuration(goal_id, mep_configuration)

        errors = AnalyzeMepErrors()

        # Check that EEG is available.
        if not self.eeg_available:
            self.logger.error('{}: EEG not available, aborting.'.format(goal_id))

            success = False
            amplitude = None
            latency = None
            channel_data = None

            return success, errors, amplitude, latency, channel_data

        gathered_preactivation_buffer = GatheredEegBuffer()
        gathered_mep_buffer = GatheredEegBuffer()

        preactivation_check_enabled = mep_configuration.preactivation_check.enabled

        # Gather preactivation data asynchronously.

        if preactivation_check_enabled:
            self.logger.info('{}: Gathering preactivation data...'.format(goal_id))

            self.async_gather_eeg_in_time_window(
                goal_id=goal_id,
                baseline_time=time,
                time_window=mep_configuration.preactivation_check.time_window,
                gathered_eeg_buffer=gathered_preactivation_buffer,
            )

        # Gather MEP data asynchronously.

        self.logger.info('{}: Gathering MEP data...'.format(goal_id))

        self.async_gather_eeg_in_time_window(
            goal_id=goal_id,
            baseline_time=time,
            time_window=mep_configuration.time_window,
            gathered_eeg_buffer=gathered_mep_buffer,
        )

        # Wait for gathering to be finished.

        if preactivation_check_enabled:
            gathered_preactivation_buffer.wait()

        gathered_mep_buffer.wait()

        if not rclpy.ok():
            # TODO: Send an error if ROS node shuts down.

            success = False
            amplitude = None
            latency = None
            channel_data = None

            return success, errors, amplitude, latency, channel_data

        # Collect errors from gatherers.

        if preactivation_check_enabled:
            errors.gather_preactivation_error = gathered_preactivation_buffer.error

        errors.gather_mep_error = gathered_mep_buffer.error

        # Combine and check errors before continuing.

        if errors.gather_mep_error.value != GatherEegError.NO_ERROR or errors.gather_preactivation_error.value != GatherEegError.NO_ERROR:
            errors.mep_error = MepError(value=MepError.GATHERING_DATA_FAILED)

        if errors.mep_error.value != MepError.NO_ERROR:
            success = False
            amplitude = None
            latency = None
            channel_data = None

            return success, errors, amplitude, latency, channel_data

        self.logger.info('{}: Successfully gathered data.'.format(goal_id))

        # Validate EMG channel.

        emg_channel = mep_configuration.emg_channel

        mep_error = self.validate_emg_channel(
            goal_id=goal_id,
            emg_channel=emg_channel,
            eeg_buffer=gathered_mep_buffer.eeg_buffer,
        )
        errors.mep_error = mep_error

        if errors.mep_error.value != MepError.NO_ERROR:
            success = False
            amplitude = None
            latency = None
            channel_data = None

            return success, errors, amplitude, latency, channel_data

        # Check preactivation

        if preactivation_check_enabled:
            mep_error = self.check_preactivation(
                goal_id=goal_id,
                mep_configuration=mep_configuration,
                emg_channel=emg_channel,
                preactivation_buffer=gathered_preactivation_buffer.eeg_buffer,
            )
            errors.mep_error = mep_error

            if errors.mep_error.value != MepError.NO_ERROR:
                success = False
                amplitude = None
                latency = None
                channel_data = None

                return success, errors, amplitude, latency, channel_data

        # Compute MEP based on gathered data.

        channel_data, amplitude, latency = self.compute_mep_amplitude_and_latency(
            goal_id=goal_id,
            time=time,
            emg_channel=emg_channel,
            mep_buffer=gathered_mep_buffer.eeg_buffer,
        )

        # Combine errors into a single success indicator.
        success = \
            errors.gather_mep_error.value == GatherEegError.NO_ERROR and \
            errors.gather_preactivation_error.value == GatherEegError.NO_ERROR and \
            errors.mep_error.value == MepError.NO_ERROR

        return success, errors, amplitude, latency, channel_data

    def analyze_mep_action_handler(self, goal_handle):
        request = goal_handle.request

        time = request.time
        mep_configuration = request.mep_configuration

        # Use short version of goal ID (2 first bytes as hex) for logging.
        #
        uuid = goal_handle.goal_id.uuid
        goal_id = "{:02x}{:02x}".format(uuid[0], uuid[1])

        self.logger.info('{}:'.format(goal_id))
        self.logger.info('{}: New goal received: {}.'.format(goal_id, goal_id))

        success, errors, amplitude, latency, mep_buffer = self.analyze_mep(
            goal_id=goal_id,
            time=time,
            mep_configuration=mep_configuration
        )

        # Create and return a Result object.

        result = AnalyzeMep.Result()

        goal_handle.succeed()

        # HACK: ROS doesn't seem to support NaNs in floats, work around it by encoding None as 0.0.
        result.mep.amplitude = amplitude if amplitude is not None else 0.0
        result.mep.latency = latency if latency is not None else 0.0
        result.mep.buffer = mep_buffer if mep_buffer is not None else []
        result.errors = errors
        result.success = success

        self.logger.info('{}: Done.'.format(goal_id))

        return result

    def analyze_mep_service_handler(self, request, response):
        time = request.time
        mep_configuration = request.mep_configuration

        # HACK: Service calls are not assigned an ID by ROS, therefore assign a constant ID here.
        #   It is used only as the prefix for the log messages.
        #
        goal_id = 'abcd'

        self.logger.info('{}: '.format(goal_id))
        self.logger.info('{}: New goal received: {}.'.format(goal_id, goal_id))

        success, errors, amplitude, latency, mep_buffer = self.analyze_mep(
            goal_id=goal_id,
            time=time,
            mep_configuration=mep_configuration
        )

        # HACK: ROS doesn't seem to support NaNs in floats, work around it by encoding None as 0.0.
        response.mep.amplitude = amplitude if amplitude is not None else 0.0
        response.mep.latency = latency if latency is not None else 0.0
        response.mep.buffer = mep_buffer if mep_buffer is not None else []
        response.errors = errors
        response.success = success

        self.logger.info('{}: Done.'.format(goal_id))

        return response


def main(args=None):
    rclpy.init(args=args)

    analyze_mep_node = AnalyzeMepNode()

    # Allow several actions to be executed concurrently.
    #
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(analyze_mep_node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
