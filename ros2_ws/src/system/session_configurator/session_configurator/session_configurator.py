import os
import json
import threading
from threading import Event

from project_interfaces.srv import (
    ListProjects,
    SetActiveProject,
)

from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class SessionConfiguratorNode(Node):
    PROJECTS_ROOT = '/app/projects'

    def __init__(self):
        super().__init__('session_configurator_node')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # The project state is properly initialized later in the constructor.
        self.project_state = None
        self.project_state_lock = threading.Lock()

        # Declare ROS2 parameters
        self.declare_parameter('project', 'example')

        # Decider parameters
        self.declare_parameter('decider.module', 'example')
        self.declare_parameter('decider.enabled', False)

        # Preprocessor parameters
        self.declare_parameter('preprocessor.module', 'example')
        self.declare_parameter('preprocessor.enabled', False)

        # Presenter parameters
        self.declare_parameter('presenter.module', 'example')
        self.declare_parameter('presenter.enabled', False)

        # Experiment parameters
        self.declare_parameter('experiment.protocol', 'example')

        # Simulator parameters
        self.declare_parameter('simulator.dataset_filename', 'random_data_1_khz.json')
        self.declare_parameter('simulator.start_time', 0.0)

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetActiveProject, '/projects/active/set', self.set_active_project_callback, callback_group=self.callback_group)

        # Publisher
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos, callback_group=self.callback_group)

        # Initialize state
        self.global_state = self.load_global_state()

        if self.global_state is None:
            self.logger.info("Global state not found, creating new one.")
            self.global_state = self.initialize_global_state()
            self.save_global_state(self.global_state)

        # Set active project
        active_project = self.global_state['active_project']
        self.set_active_project(active_project)

    # State management

    def load_state(self, path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return None

    def save_state(self, path, state):
        tmp = path + ".tmp"
        with open(tmp, 'w') as f:
            json.dump(state, f, indent=2)
        os.replace(tmp, path)

    # Global state

    def load_global_state(self):
        path = os.path.join(self.PROJECTS_ROOT, "global_state.json")
        state = self.load_state(path)
        return state

    def save_global_state(self, state):
        path = os.path.join(self.PROJECTS_ROOT, "global_state.json")
        self.save_state(path, state)

    def initialize_global_state(self):
        return {
            "active_project": 'example',
        }

    # Project state

    def load_project_state(self):
        path = os.path.join(self.PROJECTS_ROOT, self.active_project, "state.json")
        state = self.load_state(path)
        return state

    def save_project_state(self, state):
        path = os.path.join(self.PROJECTS_ROOT, self.active_project, "state.json")
        self.save_state(path, state)

    def initialize_project_state(self):
        return {
            "decider": {
                "module": 'example',
                "enabled": False
            },
            "preprocessor": {
                "module": 'example',
                "enabled": False
            },
            "presenter": {
                "module": 'example',
                "enabled": False
            },
            "simulator": {
                "dataset_filename": 'random_data_1_khz.json',
                "start_time": 0.0
            },
            "experiment": {
                "protocol": 'example',
            },
        }

    def validate_project_state(self, state):
        required_keys = ["decider", "preprocessor", "presenter", "simulator", "experiment"]
        for key in required_keys:
            if key not in state:
                self.logger.error(f"State file is missing required key: {key}")
                return False

        if not isinstance(state["decider"], dict) or not isinstance(state["preprocessor"], dict) or not isinstance(state["presenter"], dict):
            self.logger.error("State file has invalid structure for decider, preprocessor, or presenter.")
            return False

        if not isinstance(state["simulator"], dict):
            self.logger.error("State file has invalid structure for simulator.")
            return False

        if not isinstance(state["experiment"], dict):
            self.logger.error("State file has invalid structure for experiment.")
            return False

        if not all(key in state["decider"] for key in ["module", "enabled"]):
            self.logger.error("State file is missing required keys in decider.")
            return False

        if not all(key in state["preprocessor"] for key in ["module", "enabled"]):
            self.logger.error("State file is missing required keys in preprocessor.")
            return False

        if not all(key in state["presenter"] for key in ["module", "enabled"]):
            self.logger.error("State file is missing required keys in presenter.")
            return False

        if not all(key in state["simulator"] for key in ["dataset_filename", "start_time"]):
            self.logger.error("State file is missing required keys in simulator.")
            return False

        if not all(key in state["experiment"] for key in ["protocol"]):
            self.logger.error("State file is missing required keys in experiment.")
            return False

        return True

    # Project selection

    def set_active_project(self, project_name):
        self.active_project = project_name

        # Publish active project
        msg = String(data=project_name)
        self.active_project_publisher.publish(msg)

        # Load or initialize state
        with self.project_state_lock:
            self.project_state = self.load_project_state()
            if self.project_state is None:
                self.project_state = self.initialize_project_state()
                self.save_project_state(self.project_state)

        # Validate state
        if not self.validate_project_state(self.project_state):
            self.logger.error("Reinitializing state.")
            self.project_state = self.initialize_project_state()
            self.save_project_state(self.project_state)

        self.logger.info(f"Active project set to: {self.active_project}")

        # Set ROS2 parameters from the loaded state
        self.set_parameters([
            rclpy.parameter.Parameter('project', rclpy.parameter.Parameter.Type.STRING, project_name),
            rclpy.parameter.Parameter('decider.module', rclpy.parameter.Parameter.Type.STRING, self.project_state["decider"]["module"]),
            rclpy.parameter.Parameter('decider.enabled', rclpy.parameter.Parameter.Type.BOOL, self.project_state["decider"]["enabled"]),
            rclpy.parameter.Parameter('preprocessor.module', rclpy.parameter.Parameter.Type.STRING, self.project_state["preprocessor"]["module"]),
            rclpy.parameter.Parameter('preprocessor.enabled', rclpy.parameter.Parameter.Type.BOOL, self.project_state["preprocessor"]["enabled"]),
            rclpy.parameter.Parameter('presenter.module', rclpy.parameter.Parameter.Type.STRING, self.project_state["presenter"]["module"]),
            rclpy.parameter.Parameter('presenter.enabled', rclpy.parameter.Parameter.Type.BOOL, self.project_state["presenter"]["enabled"]),
            rclpy.parameter.Parameter('experiment.protocol', rclpy.parameter.Parameter.Type.STRING, self.project_state["experiment"]["protocol"]),
            rclpy.parameter.Parameter('simulator.dataset_filename', rclpy.parameter.Parameter.Type.STRING, self.project_state["simulator"]["dataset_filename"]),
            rclpy.parameter.Parameter('simulator.start_time', rclpy.parameter.Parameter.Type.DOUBLE, self.project_state["simulator"]["start_time"]),
        ])

        self.logger.info(f"State loaded for project: {self.active_project}")

    def list_projects(self):
        all_dirs = [
            d for d in os.listdir(self.PROJECTS_ROOT)
            if os.path.isdir(os.path.join(self.PROJECTS_ROOT, d))
        ]
        if "example" in all_dirs:
            all_dirs.remove("example")
            return ["example"] + sorted(all_dirs)
        return sorted(all_dirs)

    # Service callbacks

    def list_projects_callback(self, request, response):
        try:
            response.projects = self.list_projects()
            response.success = True
            self.logger.info("Projects successfully listed.")
        except Exception as e:
            self.logger.error(f"Error listing projects: {e}")
            response.success = False
        return response

    def set_active_project_callback(self, request, response):
        project = request.project
        if project in self.list_projects():
            self.set_active_project(project)
            response.success = True
            self.logger.info(f"Active project set to: {project}")
        else:
            response.success = False
            self.logger.error(f"Project does not exist: {project}")

        # Store the active project in the global state
        self.global_state["active_project"] = project
        self.save_global_state(self.global_state)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SessionConfiguratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()