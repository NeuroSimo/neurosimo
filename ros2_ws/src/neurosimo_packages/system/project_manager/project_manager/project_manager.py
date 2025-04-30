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
    SetDataset,
    SetPlayback,
    SetLoop,
    SetStartTime,
    SetRecordData,
)

from std_msgs.msg import String, Bool, Float64

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

        # The project state is properly initialized later in the constructor.
        self.project_state = None
        self.project_state_lock = threading.Lock()

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

        self.set_dataset_service = self.create_client(SetDataset, "/eeg_simulator/dataset/set", callback_group=self.callback_group)
        self.set_playback_service = self.create_client(SetPlayback, "/eeg_simulator/playback/set", callback_group=self.callback_group)
        self.set_loop_service = self.create_client(SetLoop, "/eeg_simulator/loop/set", callback_group=self.callback_group)
        self.set_start_time_service = self.create_client(SetStartTime, "/eeg_simulator/start_time/set", callback_group=self.callback_group)
        self.record_data_service = self.create_client(SetRecordData, "/eeg_recorder/record_data/set", callback_group=self.callback_group)

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

        while not self.set_dataset_service.wait_for_service(timeout_sec=2.0):
            self.logger.error("Set dataset service not available, waiting...")

        while not self.set_playback_service.wait_for_service(timeout_sec=2.0):
            self.logger.error("Set playback service not available, waiting...")
        
        while not self.set_loop_service.wait_for_service(timeout_sec=2.0):
            self.logger.error("Set loop service not available, waiting...")

        while not self.set_start_time_service.wait_for_service(timeout_sec=2.0):
            self.logger.error("Set start time service not available, waiting...")

        while not self.record_data_service.wait_for_service(timeout_sec=2.0):
            self.logger.error("Set record data service not available, waiting...")

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

        self.create_subscription(String, "/eeg_simulator/dataset", self.dataset_callback, 10)
        self.create_subscription(Bool, "/eeg_simulator/playback", self.playback_callback, 10)
        self.create_subscription(Bool, "/eeg_simulator/loop", self.loop_callback, 10)
        self.create_subscription(Float64, "/eeg_simulator/start_time", self.start_time_callback, 10)
        self.create_subscription(Bool, "/eeg_recorder/record_data", self.record_data_callback, 10)

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
                "playback": False,
                "loop": False,
                "start_time": 0.0
            },
            "recorder": {
                "record_data": False
            }
        }

    def validate_project_state(self, state):
        required_keys = ["decider", "preprocessor", "presenter", "simulator", "recorder"]
        for key in required_keys:
            if key not in state:
                self.logger.error(f"State file is missing required key: {key}")
                return False

        if not isinstance(state["decider"], dict) or not isinstance(state["preprocessor"], dict) or not isinstance(state["presenter"], dict):
            self.logger.error("State file has invalid structure for decider, preprocessor, or presenter.")
            return False
    
        if not isinstance(state["simulator"], dict) or not isinstance(state["recorder"], dict):
            self.logger.error("State file has invalid structure for simulator or recorder.")
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
    
        if not all(key in state["simulator"] for key in ["dataset_filename", "playback", "loop", "start_time"]):
            self.logger.error("State file is missing required keys in simulator.")
            return False
    
        if not all(key in state["recorder"] for key in ["record_data"]):
            self.logger.error("State file is missing required keys in recorder.")
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

        # Set the state for the decider, preprocessor, and presenter
        decider_module = self.project_state["decider"]["module"]
        decider_enabled = self.project_state["decider"]["enabled"]
        preprocessor_module = self.project_state["preprocessor"]["module"]
        preprocessor_enabled = self.project_state["preprocessor"]["enabled"]
        presenter_module = self.project_state["presenter"]["module"]
        presenter_enabled = self.project_state["presenter"]["enabled"]

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

        # Set the state for the simulator
        dataset_filename = self.project_state["simulator"]["dataset_filename"]
        playback = self.project_state["simulator"]["playback"]
        loop = self.project_state["simulator"]["loop"]
        start_time = self.project_state["simulator"]["start_time"]
        record_data = self.project_state["recorder"]["record_data"]

        self.set_dataset(dataset_filename)
        self.set_playback(playback)
        self.set_loop(loop)
        self.set_start_time(start_time)
        self.set_record_data(record_data)

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
        request = SetDeciderEnabled.Request()
        request.enabled = enabled

        future = self.decider_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set decider enabled to {enabled}.")
            
        future.add_done_callback(callback)

    def set_preprocessor_module(self, module_name):
        request = SetPreprocessorModule.Request()
        request.module = module_name

        future = self.preprocessor_module_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set preprocessor module to {module_name}.")
        
        future.add_done_callback(callback)
    
    def set_preprocessor_enabled(self, enabled):
        request = SetPreprocessorEnabled.Request()
        request.enabled = enabled

        future = self.preprocessor_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set preprocessor enabled to {enabled}.")
        
        future.add_done_callback(callback)

    def set_presenter_module(self, module_name):
        request = SetPresenterModule.Request()
        request.module = module_name

        future = self.presenter_module_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set presenter module to {module_name}.")
        
        future.add_done_callback(callback)

    def set_presenter_enabled(self, enabled):
        request = SetPresenterEnabled.Request()
        request.enabled = enabled

        future = self.presenter_enabled_client.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set presenter enabled to {enabled}.")

        future.add_done_callback(callback)

    def set_dataset(self, dataset_filename):
        request = SetDataset.Request()
        request.filename = dataset_filename

        future = self.set_dataset_service.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set dataset to {dataset_filename}.")
        
        future.add_done_callback(callback)
    
    def set_playback(self, playback):
        request = SetPlayback.Request()
        request.playback = playback

        future = self.set_playback_service.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set playback to {playback}.")
        
        future.add_done_callback(callback)
    
    def set_loop(self, loop):
        request = SetLoop.Request()
        request.loop = loop

        future = self.set_loop_service.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set loop to {loop}.")
        
        future.add_done_callback(callback)

    def set_start_time(self, start_time):
        request = SetStartTime.Request()
        request.start_time = start_time

        future = self.set_start_time_service.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set start time to {start_time}.")
        
        future.add_done_callback(callback)
    
    def set_record_data(self, record_data):
        request = SetRecordData.Request()
        request.record_data = record_data

        future = self.record_data_service.call_async(request)

        def callback(future):
            result = future.result()
            if result is None or not result.success:
                self.logger.error(f"Failed to set record data to {record_data}.")
        
        future.add_done_callback(callback)

    # Subscriber callbacks

    def decider_module_callback(self, msg: String):
        if self.project_state is None:
            return
        with self.project_state_lock:
            self.project_state["decider"]["module"] = msg.data
            self.save_project_state(self.project_state)

    def decider_enabled_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["decider"]["enabled"] = flag
            self.save_project_state(self.project_state)

    def preprocessor_module_callback(self, msg: String):
        if self.project_state is None:
            return
        with self.project_state_lock:
            self.project_state["preprocessor"]["module"] = msg.data
            self.save_project_state(self.project_state)

    def preprocessor_enabled_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["preprocessor"]["enabled"] = flag
            self.save_project_state(self.project_state)

    def presenter_module_callback(self, msg: String):
        if self.project_state is None:
            return
        with self.project_state_lock:
            self.project_state["presenter"]["module"] = msg.data
            self.save_project_state(self.project_state)

    def presenter_enabled_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["presenter"]["enabled"] = flag
            self.save_project_state(self.project_state)

    def dataset_callback(self, msg: String):
        if self.project_state is None:
            return
        with self.project_state_lock:
            self.project_state["simulator"]["dataset_filename"] = msg.data
            self.save_project_state(self.project_state)

    def playback_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["simulator"]["playback"] = flag
            self.save_project_state(self.project_state)

    def loop_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["simulator"]["loop"] = flag
            self.save_project_state(self.project_state)

    def start_time_callback(self, msg: Float64):
        if self.project_state is None:
            return
        with self.project_state_lock:
            self.project_state["simulator"]["start_time"] = msg.data
            self.save_project_state(self.project_state)
    
    def record_data_callback(self, msg: Bool):
        if self.project_state is None:
            return
        flag = msg.data
        with self.project_state_lock:
            self.project_state["recorder"]["record_data"] = flag
            self.save_project_state(self.project_state)


def main(args=None):
    rclpy.init(args=args)
    node = ProjectManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
