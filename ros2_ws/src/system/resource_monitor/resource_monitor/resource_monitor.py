import shutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from system_interfaces.msg import DiskStatus


# Hard-coded configuration for MVP
MONITORED_PATH = '/app/projects'
THRESHOLD_BYTES = 20 * 1024 * 1024 * 1024  # 20 GiB
CHECK_INTERVAL_SEC = 5.0


class ResourceMonitorNode(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        self.logger = self.get_logger()

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

        # Create timer for periodic disk checks
        self._check_timer = self.create_timer(CHECK_INTERVAL_SEC, self._check_disk_space)

        # Perform initial check immediately
        self._check_disk_space()

        self.logger.info(
            f'Resource Monitor initialized: monitoring {MONITORED_PATH}, '
            f'threshold={THRESHOLD_BYTES / (1024**3):.1f} GiB'
        )

    def _check_disk_space(self):
        """Check disk space and publish status."""
        msg = DiskStatus()
        msg.path = MONITORED_PATH
        msg.threshold_bytes = THRESHOLD_BYTES

        try:
            usage = shutil.disk_usage(MONITORED_PATH)
            msg.free_bytes = usage.free
            msg.total_bytes = usage.total
            msg.free_percent = (usage.free / usage.total) * 100.0 if usage.total > 0 else 0.0
            msg.is_ok = usage.free >= THRESHOLD_BYTES

            if not msg.is_ok:
                self.logger.warn(
                    f'Low disk space: {msg.free_bytes / (1024**3):.1f} GiB free, '
                    f'threshold is {THRESHOLD_BYTES / (1024**3):.1f} GiB'
                )

        except OSError as e:
            self.logger.error(f'Failed to check disk space for {MONITORED_PATH}: {e}')
            msg.free_bytes = 0
            msg.total_bytes = 0
            msg.free_percent = 0.0
            msg.is_ok = False

        self._disk_status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
