import rclpy
from rclpy.node import Node

from ui_interfaces.msg import PlannerState, Pulse
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class MasterStateNode(Node):

    def __init__(self):
        super().__init__('master_state')

        # Persist the latest sample.
        publish_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        subscribe_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._state_publisher = self.create_publisher(
            PlannerState,
            "/planner/state",
            publish_qos
        )
        self._state_subscriber = self.create_subscription(
            PlannerState,
            '/planner/inner/state',
            self.state_updated,
            qos_profile=subscribe_qos
        )

    def state_updated(self, msg):
        self._state_publisher.publish(msg)
        self.get_logger().info(
            f'Updated state, nof targets: {len(msg.targets)}, '
            f'nof sequences: {len(msg.pulse_sequences)}')


def main():
    rclpy.init()
    master_state_node = MasterStateNode()
    rclpy.spin(master_state_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
