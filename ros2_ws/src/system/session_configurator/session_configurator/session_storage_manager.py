import os
import json


class SessionStorageManager:
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

    # Session config

    def load_session_config(self, project_name):
        path = os.path.join(self.PROJECTS_ROOT, project_name, "session_config.json")
        config = self._load_json(path)

        if config is None:
            self.logger.info(f"Session config not found for project: {project_name}, initializing new one.")
            config = self.initialize_session_config(project_name)

        return config

    def save_session_config(self, project_name, config):
        path = os.path.join(self.PROJECTS_ROOT, project_name, "session_config.json")
        self._save_json(path, config)

    def initialize_session_config(self, project_name):
        config = {
            "notes": '',
            "subject_id": 'S001',
            "decider.module": 'example.py',
            "decider.enabled": False,
            "preprocessor.module": 'example.py',
            "preprocessor.enabled": False,
            "presenter.module": 'example.py',
            "presenter.enabled": False,
            "simulator.dataset_filename": 'random_data_1_khz.json',
            "simulator.start_time": 0.0,
            "experiment.protocol": 'example.yaml',
            "recording.bag_filename": '',
            "recording.is_preprocessed": False,
            "data_source": 'simulator',
        }
        self.save_session_config(project_name, config)
        return config

    def validate_session_config(self, config):
        required_keys = [
            "notes", "subject_id",
            "decider.module", "decider.enabled",
            "preprocessor.module", "preprocessor.enabled",
            "presenter.module", "presenter.enabled",
            "simulator.dataset_filename", "simulator.start_time",
            "experiment.protocol",
            "recording.bag_filename", "recording.is_preprocessed",
            "data_source"
        ]
        for key in required_keys:
            if key not in config:
                self.logger.error(f"Config file is missing required key: {key}")
                return False

        return True

    def get_session_config_for_project(self, project_name):
        # Load or initialize session config
        session_config = self.load_session_config(project_name)

        # Validate config
        if not self.validate_session_config(session_config):
            self.logger.error("Reinitializing config.")
            session_config = self.initialize_session_config(project_name)

        return session_config
