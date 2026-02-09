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
            # EEG Configuration
            "eeg_port": 50000,
            "eeg_device": 'neurone',
            "turbolink_sampling_frequency": 5000,
            "turbolink_eeg_channel_count": 64,
            "num_of_tolerated_dropped_samples": 2,
            # LabJack Configuration
            "simulate_labjack": False,
            # Safety Configuration
            "minimum_intertrial_interval": 2.0,
            "maximum_loopback_latency": 0.005,
            "maximum_timing_error": 0.0,
            # Disk Space Monitoring Configuration
            "disk_warning_threshold": '100GiB',
            "disk_error_threshold": '50GiB',
        }
        self.save_global_config(config)
        return config

    def validate_global_config(self, config):
        required_keys = [
            "active_project",
            "eeg_port", "eeg_device",
            "turbolink_sampling_frequency", "turbolink_eeg_channel_count",
            "num_of_tolerated_dropped_samples",
            "simulate_labjack",
            "minimum_intertrial_interval", "maximum_loopback_latency",
            "maximum_timing_error",
            "disk_warning_threshold", "disk_error_threshold"
        ]
        for key in required_keys:
            if key not in config:
                self.logger.error(f"Global config is missing required key: {key}")
                return False
        return True

    def get_global_config(self):
        """Load and validate global config, reinitializing if needed."""
        config = self.load_global_config()
        if not self.validate_global_config(config):
            self.logger.error("Reinitializing global config.")
            config = self.initialize_global_config()
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
