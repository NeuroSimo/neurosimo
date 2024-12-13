import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ui_interfaces.msg import PlannerState


class StateNode(Node):

    def __init__(self, name):
        super().__init__(name)

        # Persist the latest sample.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._state_publisher = self.create_publisher(
            PlannerState,
            "/planner/inner/state",
            qos
        )
        self._state_subscriber = self.create_subscription(
            PlannerState,
            '/planner/inner/state',
            self.state_updated,
            qos_profile=qos
        )
        self._state = None

    def state_updated(self, msg):
        self._state = msg
