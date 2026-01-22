import os
import json
import threading


class ProjectManager:
    PROJECTS_ROOT = '/app/projects'

    def __init__(self, logger):
        self.logger = logger

    # Helper functions

    def _load_json(self, path):
        try:
            with open(path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return None

    def _save_json(self, path, state):
        tmp = path + ".tmp"
        with open(tmp, 'w') as f:
            json.dump(state, f, indent=2)
        os.replace(tmp, path)

    # Project state

    def load_project_state(self):
        path = os.path.join(self.PROJECTS_ROOT, "project_state.json")
        state = self._load_json(path)

        if state is None:
            self.logger.info("Project state not found, creating new one.")
            state = self.initialize_project_state()

        return state

    def save_project_state(self, state):
        path = os.path.join(self.PROJECTS_ROOT, "project_state.json")
        self._save_json(path, state)

    def initialize_project_state(self):
        state = {
            "active_project": 'example',
        }
        self.save_project_state(state)
        return state

    # Session state

    def load_session_state(self, project_name):
        path = os.path.join(self.PROJECTS_ROOT, project_name, "session_state.json")
        state = self._load_json(path)

        if state is None:
            self.logger.info(f"Session state not found for project: {project_name}, initializing new one.")
            state = self.initialize_session_state(project_name)

        return state

    def save_session_state(self, project_name, state):
        path = os.path.join(self.PROJECTS_ROOT, project_name, "session_state.json")
        self._save_json(path, state)

    def initialize_session_state(self, project_name):
        state = {
            "decider.module": 'example',
            "decider.enabled": False,
            "preprocessor.module": 'example',
            "preprocessor.enabled": False,
            "presenter.module": 'example',
            "presenter.enabled": False,
            "simulator.dataset_filename": 'random_data_1_khz.json',
            "simulator.start_time": 0.0,
            "experiment.protocol": 'example',
        }
        self.save_session_state(project_name, state)
        return state

    def validate_session_state(self, state):
        required_keys = [
            "decider.module", "decider.enabled",
            "preprocessor.module", "preprocessor.enabled",
            "presenter.module", "presenter.enabled",
            "simulator.dataset_filename", "simulator.start_time",
            "experiment.protocol"
        ]
        for key in required_keys:
            if key not in state:
                self.logger.error(f"State file is missing required key: {key}")
                return False

        return True

    # Project selection

    def get_session_state_for_project(self, project_name):
        # Load or initialize session state
        session_state = self.load_session_state(project_name)

        # Validate state
        if not self.validate_session_state(session_state):
            self.logger.error("Reinitializing state.")
            session_state = self.initialize_session_state(project_name)

        return session_state

    def list_projects(self):
        all_dirs = [
            d for d in os.listdir(self.PROJECTS_ROOT)
            if os.path.isdir(os.path.join(self.PROJECTS_ROOT, d))
        ]
        if "example" in all_dirs:
            all_dirs.remove("example")
            return ["example"] + sorted(all_dirs)
        return sorted(all_dirs)

    def get_active_project(self):
        state = self.load_project_state()
        return state["active_project"]

    def save_active_project(self, project_name):
        """Update the project state with the active project and save it."""
        state = self.load_project_state()
        state["active_project"] = project_name
        self.save_project_state(state)
