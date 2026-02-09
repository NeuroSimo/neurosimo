import os
import json


class GlobalStorageManager:
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

    # Global config

    def load_global_config(self):
        path = os.path.join(self.PROJECTS_ROOT, "global_config.json")
        config = self._load_json(path)

        if config is None:
            self.logger.info("Global config not found, creating new one.")
            config = self.initialize_global_config()

        return config

    def save_global_config(self, config):
        path = os.path.join(self.PROJECTS_ROOT, "global_config.json")
        self._save_json(path, config)

    def initialize_global_config(self):
        config = {
            "active_project": 'example',
        }
        self.save_global_config(config)
        return config

    # Project selection

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
        config = self.load_global_config()
        return config["active_project"]

    def save_active_project(self, project_name):
        """Update the global config with the active project and save it."""
        config = self.load_global_config()
        config["active_project"] = project_name
        self.save_global_config(config)
