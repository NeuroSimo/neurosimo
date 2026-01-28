import shutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from system_interfaces.msg import DiskStatus
from std_msgs.msg import Empty
from .utils import parse_size_string


# Hard-coded path to monitor disk space
MONITORED_PATH = '/app/projects'
CHECK_INTERVAL_SEC = 5.0
HEARTBEAT_INTERVAL_SEC = 0.5


class ResourceMonitorNode(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.logger = self.get_logger()

        # Declare ROS parameters (no defaults - will fail if not provided)
        self.declare_parameter('disk.warning_threshold', '')
        self.declare_parameter('disk.error_threshold', '')

        # Get parameter values and parse them to bytes
        warning_threshold_str = self.get_parameter('disk.warning_threshold').value
        error_threshold_str = self.get_parameter('disk.error_threshold').value

        self._warning_threshold_bytes = parse_size_string(warning_threshold_str)
        self._error_threshold_bytes = parse_size_string(error_threshold_str)

        # Create latched publisher for disk status
        status_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._disk_status_publisher = self.create_publisher(
            DiskStatus,
            '/system/disk_status',
            status_qos
        )

        # Create heartbeat publisher
        self._heartbeat_publisher = self.create_publisher(
            Empty,
            '/health/resource_monitor/heartbeat',
            10
        )

        # Create timer for periodic disk checks
        self._check_timer = self.create_timer(CHECK_INTERVAL_SEC, self._check_disk_space)

        # Create timer for heartbeat publishing
        self._heartbeat_timer = self.create_timer(HEARTBEAT_INTERVAL_SEC, self._publish_heartbeat)

        # Perform initial check immediately
        self._check_disk_space()

        self.logger.info(
            f'Resource Monitor initialized: monitoring {MONITORED_PATH}, '
            f'warning_threshold="{warning_threshold_str}" ({self._warning_threshold_bytes / (1024**3):.1f} GiB), '
            f'error_threshold="{error_threshold_str}" ({self._error_threshold_bytes / (1024**3):.1f} GiB)'
        )

    def _check_disk_space(self):
        """Check disk space and publish status."""
        usage = shutil.disk_usage(MONITORED_PATH)
        is_ok = usage.free >= self._error_threshold_bytes

        msg = DiskStatus()
        msg.warning_threshold_bytes = self._warning_threshold_bytes
        msg.error_threshold_bytes = self._error_threshold_bytes
        msg.free_bytes = usage.free
        msg.total_bytes = usage.total
        msg.is_ok = is_ok

        if usage.free < self._warning_threshold_bytes:
            threshold_name = 'warning' if is_ok else 'error'
            threshold_bytes = self._warning_threshold_bytes if is_ok else self._error_threshold_bytes

            message = (
                f'Low disk space: {usage.free / (1024**3):.1f} GiB free, '
                f'{threshold_name} threshold: {threshold_bytes / (1024**3):.1f} GiB'
            )
            if is_ok:
                self.logger.warn(message)
            else:
                self.logger.error(message)

        self._disk_status_publisher.publish(msg)

        # Publish heartbeat message
        self._publish_heartbeat()

    def _publish_heartbeat(self):
        """Publish heartbeat message."""
        heartbeat = Empty()
        self._heartbeat_publisher.publish(heartbeat)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
