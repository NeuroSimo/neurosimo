import time
from threading import Event

import numpy as np

from experiment_interfaces.msg import Trial
from experiment_interfaces.srv import ValidateTrial

from event_interfaces.msg import WaveformsForCoilSet
from targeting_interfaces.srv import GetMaximumIntensity, GetMultipulseWaveforms, GetDefaultWaveform

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

class TrialValidatorNode(Node):
    # TODO: Channel count hardcoded for now.
    NUM_OF_CHANNELS = 5

    def __init__(self):
        super().__init__('trial_validator_node')

        self.logger = self.get_logger()

        # Needed to enable calling another service inside a service handler. See also using MultiThreadedExecutor.
        self.callback_group = ReentrantCallbackGroup()

        # Service for validating trial.

        self.service = self.create_service(
            ValidateTrial,
            '/trial/validate',
            self.validate_trial_callback,
            callback_group=self.callback_group,
        )

        # Service client for getting maximum intensity.

        self.get_maximum_intensity_client = self.create_client(GetMaximumIntensity, '/targeting/get_maximum_intensity', callback_group=self.callback_group)
        while not self.get_maximum_intensity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /targeting/get_maximum_intensity not available, waiting...')

        # Service client for getting default waveform.

        self.get_default_waveform_client = self.create_client(GetDefaultWaveform, '/waveforms/get_default')
        while not self.get_default_waveform_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /waveforms/get_default not available, waiting...')

        # Service client for getting multipulse waveforms.

        self.get_multipulse_waveforms_client = self.create_client(GetMultipulseWaveforms, '/waveforms/get_multipulse_waveforms')
        while not self.get_multipulse_waveforms_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /waveforms/get_multipulse_waveforms not available, waiting...')

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

    # Waveform services

    def get_default_waveform(self, channel):
        request = GetDefaultWaveform.Request()

        request.channel = channel

        response = self.async_service_call(self.get_default_waveform_client, request)
        assert response.success, "Invalid channel."

        waveform = response.waveform

        return waveform

    def get_multipulse_waveforms(self, targets, target_waveforms):
        request = GetMultipulseWaveforms.Request()

        request.targets = targets
        request.target_waveforms = target_waveforms

        response = self.async_service_call(self.get_multipulse_waveforms_client, request)

        success = response.success
        initial_voltages = response.initial_voltages
        approximated_waveforms = response.approximated_waveforms

        return success, initial_voltages, approximated_waveforms

    def get_maximum_intensity(self, displacement_x, displacement_y, rotation_angle, algorithm):
        request = GetMaximumIntensity.Request()
        request.displacement_x = displacement_x
        request.displacement_y = displacement_y
        request.rotation_angle = rotation_angle
        request.algorithm = algorithm

        response = self.async_service_call(self.get_maximum_intensity_client, request)

        assert response.success, "Get maximum intensity call failed."

        return response.maximum_intensity

    def validate_trial_callback(self, request, response):
        targets = request.trial.targets

        if len(targets) == 1:
            target = targets[0]

            displacement_x = target.displacement_x
            displacement_y = target.displacement_y
            rotation_angle = target.rotation_angle
            algorithm = target.algorithm
            intensity = target.intensity

            self.get_logger().info('')
            self.get_logger().info('Validation requested for target at (x, y, angle) = ({}, {}, {}) with intensity {} V/m.'.format(
                displacement_x,
                displacement_y,
                rotation_angle,
                intensity,
            ))
            max_intensity = self.get_maximum_intensity(
                displacement_x=displacement_x,
                displacement_y=displacement_y,
                rotation_angle=rotation_angle,
                algorithm=algorithm,
            )

            is_trial_valid = intensity <= max_intensity

            self.get_logger().info('Maximum intensity for the requested target: {} V/m'.format(max_intensity))

        else:
            self.get_logger().info('')
            self.get_logger().info('Validation requested for {} targets.'.format(len(targets)))
            for i, target in enumerate(targets):
                displacement_x = target.displacement_x
                displacement_y = target.displacement_y
                rotation_angle = target.rotation_angle
                algorithm = target.algorithm
                intensity = target.intensity

                self.get_logger().info('Target {}: (x, y, angle) = ({}, {}, {}) with intensity {} V/m.'.format(
                    i,
                    displacement_x,
                    displacement_y,
                    rotation_angle,
                    intensity,
                ))

            target_waveforms = [WaveformsForCoilSet() for _ in range(len(targets))]
            for i in range(len(targets)):
                target_waveforms[i].waveforms = [self.get_default_waveform(channel) for channel in range(self.NUM_OF_CHANNELS)]

            success, _, _ = self.get_multipulse_waveforms(
                targets=targets,
                target_waveforms=target_waveforms,
            )
            is_trial_valid = success

        self.get_logger().info('Requested trial is {}.'.format('valid' if is_trial_valid else 'invalid'))

        response.is_trial_valid = is_trial_valid
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)

    trial_validator_node = TrialValidatorNode()

    # Needed to enable calling another service within a service handler. See also using ReentrantCallbackGroup.
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(trial_validator_node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
