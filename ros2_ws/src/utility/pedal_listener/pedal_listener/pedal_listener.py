import os

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Bool

class PedalListenerNode(Node):

    RECONNECT_PERIOD_IN_SECONDS = 1.0
    READER_PERIOD_IN_SECONDS = 0.02

    def __init__(self):
        super().__init__('pedal_listener')

        self.port = os.getenv("PEDAL_PORT")
        self.baud_rate = int(os.getenv("PEDAL_BAUD_RATE"))

        # Persist the latest sample.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pedal_connected_publisher = self.create_publisher(
            Bool,
            "/pedal/connected",
            qos
        )
        self.left_button_publisher = self.create_publisher(
            Bool,
            "/pedal/left_button/pressed",
            qos
        )
        self.right_button_publisher = self.create_publisher(
            Bool,
            "/pedal/right_button/pressed",
            qos
        )

        self.serial = None
        self.reconnect_timer = self.create_timer(self.RECONNECT_PERIOD_IN_SECONDS, self.reconnect)
        self.reader_timer = self.create_timer(self.READER_PERIOD_IN_SECONDS, self.reader)

    def handle_left_button_pressed(self):
        msg = Bool()
        msg.data = True
        self.left_button_publisher.publish(msg)
        self.get_logger().info("Left button pressed.")

    def handle_left_button_released(self):
        msg = Bool()
        msg.data = False
        self.left_button_publisher.publish(msg)
        self.get_logger().info("Left button released.")

    def handle_right_button_pressed(self):
        msg = Bool()
        msg.data = True
        self.right_button_publisher.publish(msg)
        self.get_logger().info("Right button pressed.")

    def handle_right_button_released(self):
        msg = Bool()
        msg.data = False
        self.right_button_publisher.publish(msg)
        self.get_logger().info("Right button released.")

    def handle_disconnected(self):
        self.serial = None

        msg = Bool()
        msg.data = False
        self.pedal_connected_publisher.publish(msg)

        self.get_logger().info("Pedal disconnected.")

    def handle_connected(self):
        msg = Bool()
        msg.data = True
        self.pedal_connected_publisher.publish(msg)

        self.get_logger().info("Pedal connected.")

    def reader(self):
        try:
            if self.serial is None:
                return

            value = self.serial.read()

            if len(value) == 0:
                return

            if value[0] == 0x00:
                self.handle_left_button_pressed()
            elif value[0] == 0x01:
                self.handle_left_button_released()
            elif value[0] == 0x02:
                self.handle_right_button_pressed()
            elif value[0] == 0x03:
                self.handle_right_button_released()
            else:
                self.get_logger().error("Unknown message received from the pedal.")

        except serial.serialutil.SerialException:
            self.handle_disconnected()

    def reconnect(self):
        if self.serial is not None:
            return

        try:
            self.get_logger().info("Connecting to pedal in port {} using baud rate {}.".format(self.port, self.baud_rate))
            self.serial = serial.Serial(self.port, self.baud_rate)
            self.handle_connected()

        except serial.serialutil.SerialException:
            self.handle_disconnected()


def main():
    rclpy.init()
    pedal_listener_node = PedalListenerNode()
    rclpy.spin(pedal_listener_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
