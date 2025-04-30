import os
import json
import threading
from threading import Event

from project_interfaces.srv import (
    ListProjects,
    SetActiveProject,
    SetDeciderModule,
    SetDeciderEnabled,
    SetPreprocessorModule,
    SetPreprocessorEnabled,
    SetPresenterModule,
    SetPresenterEnabled,
)

from std_msgs.msg import String, Bool

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ProjectManagerNode(Node):
    PROJECTS_ROOT = '/app/projects'

    def __init__(self):
        super().__init__('project_manager_node')
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        self.active_project = None

        # Initialize state
        self.state_lock = threading.Lock()
        self.state = {}

        # Services
        self.create_service(ListProjects, '/projects/list', self.list_projects_callback, callback_group=self.callback_group)
        self.create_service(SetActiveProject, '/projects/active/set', self.set_active_project_callback, callback_group=self.callback_group)

        # Clients
        self.decider_module_client = self.create_client(SetDeciderModule, "/pipeline/decider/module/set", callback_group=self.callback_group)
        self.decider_enabled_client = self.create_client(SetDeciderEnabled, "/pipeline/decider/enabled/set", callback_group=self.callback_group)
        self.preprocessor_module_client = self.create_client(SetPreprocessorModule, "/pipeline/preprocessor/module/set", callback_group=self.callback_group)
        self.preprocessor_enabled_client = self.create_client(SetPreprocessorEnabled, "/pipeline/preprocessor/enabled/set", callback_group=self.callback_group)
        self.presenter_module_client = self.create_client(SetPresenterModule, "/pipeline/presenter/module/set", callback_group=self.callback_group)
        self.presenter_enabled_client = self.create_client(SetPresenterEnabled, "/pipeline/presenter/enabled/set", callback_group=self.callback_group)

        # Wait for services to be available
        while not self.preprocessor_module_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Preprocessor module service not available, waiting...")

        while not self.preprocessor_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Preprocessor enabled service not available, waiting...")

        while not self.presenter_module_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Presenter module service not available, waiting...")

        while not self.presenter_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Presenter enabled service not available, waiting...")

        while not self.decider_module_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Decider module service not available, waiting...")
        
        while not self.decider_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Decider enabled service not available, waiting...")

        # Publisher
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.active_project_publisher = self.create_publisher(String, "/projects/active", qos, callback_group=self.callback_group)

        # Subscribers
        self.create_subscription(String, "/pipeline/decider/module", self.decider_module_callback, 10)
        self.create_subscription(Bool, "/pipeline/decider/enabled", self.decider_enabled_callback, 10)
        self.create_subscription(String, "/pipeline/preprocessor/module", self.preprocessor_module_callback, 10)
        self.create_subscription(Bool, "/pipeline/preprocessor/enabled", self.preprocessor_enabled_callback, 10)
        self.create_subscription(String, "/pipeline/presenter/module", self.presenter_module_callback, 10)
        self.create_subscription(Bool, "/pipeline/presenter/enabled", self.presenter_enabled_callback, 10)

        # Initialize active project to first in list
        projects = self.list_projects()
        if projects:
            self.set_active_project(projects[0])

    # State management

    def state_file_path(self):
        return os.path.join(self.PROJECTS_ROOT, self.active_project, "state.json")

    def load_state(self):
        try:
            with open(self.state_file_path(), 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return None

    def initialize_state(self):
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
            }
        }

    def save_state(self, state):
        path = self.state_file_path()
        tmp = path + ".tmp"
        with open(tmp, 'w') as f:
            json.dump(state, f, indent=2)
        os.replace(tmp, path)

    # Project selection

    def set_active_project(self, project_name):
        self.active_project = project_name

        # Publish active project
        msg = String(data=project_name)
        self.active_project_publisher.publish(msg)

        # Load or initialize state
        with self.state_lock:
            self.state = self.load_state()
            if self.state is None:
                self.state = self.initialize_state()
                self.save_state(self.state)

        self.logger.info(f"Active project set to: {self.active_project}")

        # Set the state for the decider, preprocessor, and presenter
        decider_module = self.state["decider"]["module"]
        decider_enabled = self.state["decider"]["enabled"]
        preprocessor_module = self.state["preprocessor"]["module"]
        preprocessor_enabled = self.state["preprocessor"]["enabled"]
        presenter_module = self.state["presenter"]["module"]
        presenter_enabled = self.state["presenter"]["enabled"]

        # XXX: If the pipeline stage is enabled, first set the module and then enable it
        # so that the module is set before the enabling the stage. Reverse the order
        # if the pipeline stage is disabled. A better solution would be to send the new state
        # in a single message so that the pipeline stage could be set to the new state in one go.
        if decider_enabled:
            self.set_decider_module(decider_module)
            self.set_decider_enabled(decider_enabled)
        else:
            self.set_decider_enabled(decider_enabled)
            self.set_decider_module(decider_module)

        if preprocessor_enabled:
            self.set_preprocessor_module(preprocessor_module)
            self.set_preprocessor_enabled(preprocessor_enabled)
        else:
            self.set_preprocessor_enabled(preprocessor_enabled)
            self.set_preprocessor_module(preprocessor_module)

        if presenter_enabled:
            self.set_presenter_module(presenter_module)
            self.set_presenter_enabled(presenter_enabled)
        else:
            self.set_presenter_enabled(presenter_enabled)
            self.set_presenter_module(presenter_module)

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
        return response
    
    # Service calls
    def set_decider_module(self, module_name):
        request = SetDeciderModule.Request()
        request.module = module_name

        self.logger.info(f"Setting decider module to {module_name}...")
        future = self.decider_module_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set decider module to {module_name}.")

        future.add_done_callback(callback)

    def set_decider_enabled(self, enabled):
        if not self.decider_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Decider enabled service not available!")
            return

        request = SetDeciderEnabled.Request()
        request.enabled = enabled

        future = self.decider_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set decider enabled to {enabled}.")
            
        future.add_done_callback(callback)

    def set_preprocessor_module(self, module_name):
        if not self.preprocessor_module_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Preprocessor module service not available!")
            return

        request = SetPreprocessorModule.Request()
        request.module = module_name

        future = self.preprocessor_module_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set preprocessor module to {module_name}.")
        
        future.add_done_callback(callback)
    
    def set_preprocessor_enabled(self, enabled):
        if not self.preprocessor_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Preprocessor enabled service not available!")
            return

        request = SetPreprocessorEnabled.Request()
        request.enabled = enabled

        future = self.preprocessor_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set preprocessor enabled to {enabled}.")
        
        future.add_done_callback(callback)

    def set_presenter_module(self, module_name):
        if not self.presenter_module_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Presenter module service not available!")
            return

        request = SetPresenterModule.Request()
        request.module = module_name

        future = self.presenter_module_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set presenter module to {module_name}.")
        
        future.add_done_callback(callback)

    def set_presenter_enabled(self, enabled):
        if not self.presenter_enabled_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("Presenter enabled service not available!")
            return

        request = SetPresenterEnabled.Request()
        request.enabled = enabled

        future = self.presenter_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set presenter enabled to {enabled}.")

        future.add_done_callback(callback)

    # Subscriber callbacks

    def decider_module_callback(self, msg: String):
        with self.state_lock:
            self.state["decider"]["module"] = msg.data
            self.save_state(self.state)

    def decider_enabled_callback(self, msg: Bool):
        flag = msg.data
        with self.state_lock:
            self.state["decider"]["enabled"] = flag
            self.save_state(self.state)

    def preprocessor_module_callback(self, msg: String):
        with self.state_lock:
            self.state["preprocessor"]["module"] = msg.data
            self.save_state(self.state)

    def preprocessor_enabled_callback(self, msg: Bool):
        flag = msg.data
        with self.state_lock:
            self.state["preprocessor"]["enabled"] = flag
            self.save_state(self.state)

    def presenter_module_callback(self, msg: String):
        with self.state_lock:
            self.state["presenter"]["module"] = msg.data
            self.save_state(self.state)

    def presenter_enabled_callback(self, msg: Bool):
        flag = msg.data
        with self.state_lock:
            self.state["presenter"]["enabled"] = flag
            self.save_state(self.state)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
