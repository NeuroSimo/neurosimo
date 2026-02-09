import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.srv import ListProjects
from system_interfaces.msg import GlobalConfig

from rcl_interfaces.msg import SetParametersResult

from .global_storage_manager import GlobalStorageManager


class GlobalConfiguratorNode(Node):
    def __init__(self):
        super().__init__('global_configurator')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Initialize global storage manager
        self.storage_manager = GlobalStorageManager(self.logger)

        # Load global config
        global_config = self.storage_manager.get_global_config()

        # Declare ROS2 parameters
        self.declare_parameter('active_project', global_config['active_project'])
        
        # EEG Configuration
        self.declare_parameter('eeg_port', global_config['eeg_port'])
        self.declare_parameter('eeg_device', global_config['eeg_device'])
        self.declare_parameter('turbolink_sampling_frequency', global_config['turbolink_sampling_frequency'])
        self.declare_parameter('turbolink_eeg_channel_count', global_config['turbolink_eeg_channel_count'])
        self.declare_parameter('maximum_dropped_samples', global_config['maximum_dropped_samples'])
        
        # LabJack Configuration
        self.declare_parameter('simulate_labjack', global_config['simulate_labjack'])
        
        # Safety Configuration
        self.declare_parameter('minimum_intertrial_interval', global_config['minimum_intertrial_interval'])

        # Timing Configuration
        self.declare_parameter('maximum_loopback_latency', global_config['maximum_loopback_latency'])
        self.declare_parameter('maximum_timing_error', global_config['maximum_timing_error'])
        self.declare_parameter('trigger_to_pulse_delay', global_config['trigger_to_pulse_delay'])
        
        # Disk Space Monitoring Configuration
        self.declare_parameter('disk_warning_threshold', global_config['disk_warning_threshold'])
        self.declare_parameter('disk_error_threshold', global_config['disk_error_threshold'])
        
        # System Configuration
        self.declare_parameter('locale', global_config['locale'])

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.global_config_publisher = self.create_publisher(GlobalConfig, "/global_configurator/config", qos, callback_group=self.callback_group)

        # Add parameter change callback to save changes to global state
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Publish initial global config
        self.publish_global_config(global_config)

    def set_active_project(self, project_name):
        """Set the active project and save to storage."""
        if not project_name in self.storage_manager.list_projects():
            self.logger.error(f"Project does not exist: {project_name}")
            return False

        # Store the active project in the global config if it has changed
        current_config = self.storage_manager.load_global_config()
        if project_name != current_config.get('active_project'):
            self.storage_manager.save_active_project(project_name)
            self.logger.info(f"Updated global config with active_project={project_name}")

        return True
    
    def publish_global_config(self, global_config: dict):
        """Build and publish global config from dict."""
        config = GlobalConfig()
        config.active_project = global_config.get('active_project', '')
        
        # EEG Configuration
        config.eeg_port = global_config.get('eeg_port', 50000)
        config.eeg_device = global_config.get('eeg_device', 'neurone')
        config.turbolink_sampling_frequency = global_config.get('turbolink_sampling_frequency', 5000)
        config.turbolink_eeg_channel_count = global_config.get('turbolink_eeg_channel_count', 64)
        config.maximum_dropped_samples = global_config.get('maximum_dropped_samples', 2)
        
        # LabJack Configuration
        config.simulate_labjack = global_config.get('simulate_labjack', False)
        
        # Safety Configuration
        config.minimum_intertrial_interval = global_config.get('minimum_intertrial_interval', 2.0)

        # Timing Configuration
        config.maximum_loopback_latency = global_config.get('maximum_loopback_latency', 0.005)
        config.maximum_timing_error = global_config.get('maximum_timing_error', 0.0)
        config.trigger_to_pulse_delay = global_config.get('trigger_to_pulse_delay', 0.0)
        
        # Disk Space Monitoring Configuration
        config.disk_warning_threshold = global_config.get('disk_warning_threshold', '100GiB')
        config.disk_error_threshold = global_config.get('disk_error_threshold', '50GiB')
        
        # System Configuration
        config.locale = global_config.get('locale', 'en-US')
        
        self.global_config_publisher.publish(config)

    # Service callbacks

    def list_projects_callback(self, request, response):
        try:
            response.projects = self.storage_manager.list_projects()
            response.success = True
            self.logger.info("Projects successfully listed.")
        except Exception as e:
            self.logger.error(f"Error listing projects: {e}")
            response.success = False
        return response
    
    def parameter_change_callback(self, params):
        """Callback to handle global parameter changes and save them to global state."""
        try:
            # Load current global config
            global_config = self.storage_manager.load_global_config()
            
            for param in params:
                if param.name == 'active_project':
                    project_name = param.value
                    
                    # Validate and save using set_active_project
                    success = self.set_active_project(project_name)
                    if not success:
                        return SetParametersResult(successful=False, reason=f"Project does not exist: {project_name}")
                
                # Update the global config dict with the new value
                global_config[param.name] = param.value
            
            # Save the updated global config
            self.storage_manager.save_global_config(global_config)
            self.logger.info(f"Updated global config with parameter changes")
            
            # Publish the updated global config immediately (from the dict, not from ROS params)
            self.publish_global_config(global_config)

        except Exception as e:
            self.logger.error(f"Error handling parameter changes: {e}")
            return SetParametersResult(successful=False, reason=str(e))

        # Return success to allow parameter changes
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()