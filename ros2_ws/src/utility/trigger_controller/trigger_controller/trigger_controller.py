import os
import termios

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Bool
from ui_interfaces.srv import TriggerPulse

class TriggerControllerNode(Node):

    RECONNECT_PERIOD_IN_SECONDS = 1.0
    PULSE_DURATION_IN_MILLISECONDS = 0.2

    def __init__(self):
        super().__init__('trigger_controller')

        self._port = os.getenv("TRIGGER_PORT")
        self._baud_rate = int(os.getenv("TRIGGER_BAUD_RATE"))

        # Persist the latest sample.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._trigger_connected_publisher = self.create_publisher(
            Bool,
            "/trigger/connected",
            qos
        )
        self._triggered_publisher = self.create_publisher(
            Bool,
            "/trigger/triggered",
            10
        )

        self._serial = None
        self._reconnect_timer = self.create_timer(self.RECONNECT_PERIOD_IN_SECONDS, self._reconnect)

        self._trigger_service = self.create_service(TriggerPulse, '/trigger/trigger_pulse', self.trigger_pulse_callback)

    def trigger_pulse_callback(self, request, response):
        success = False
        if self._serial is not None:
            try:
                self._serial.send_break(self.PULSE_DURATION_IN_MILLISECONDS / 1000)
                success = True

            except (serial.serialutil.SerialException, termios.error):
                self._handle_disconnected()

        if success:
            self.get_logger().info('Triggered pulse successfully.')

            # Only publish in case of a success. An arbitrary design decision in a way; could as well
            # publish whether the triggering was successful, but doing it like this avoids the potential
            # bug in which the published message is misinterpreted.
            #
            msg = Bool()
            msg.data = True
            self._triggered_publisher.publish(msg)
        else:
            self.get_logger().error('Failed to trigger pulse.')

        response.success = success
        return response

    def _handle_disconnected(self):
        self._serial = None

        msg = Bool()
        msg.data = False
        self._trigger_connected_publisher.publish(msg)

        self.get_logger().info("Trigger device disconnected.")

    def _handle_connected(self):
        msg = Bool()
        msg.data = True
        self._trigger_connected_publisher.publish(msg)

        self.get_logger().info("Trigger device connected.")

    def _reconnect(self):
        if self._serial is not None:
            return

        try:
            self.get_logger().info("Connecting to trigger device in port {} using baud rate {}.".format(self._port, self._baud_rate))
            self._serial = serial.Serial(self._port, self._baud_rate)
            self._handle_connected()

        except serial.serialutil.SerialException:
            self._handle_disconnected()


def main():
    rclpy.init()
    trigger_controller_node = TriggerControllerNode()
    rclpy.spin(trigger_controller_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
