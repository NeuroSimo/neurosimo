import os
import json


class SystemStorageManager:
    PROJECTS_ROOT = '/app/projects'

    CONFIG_FILENAME = 'system_config.json'

    # Previous name of the config file, migrated automatically on first load.
    LEGACY_CONFIG_FILENAME = 'global_config.json'

    # Default config. Doubles as the schema: these keys are what a valid config
    # must contain, and missing keys are filled from here when loading.
    DEFAULT_CONFIG = {
        "active_project": 'example',
        # EEG Configuration
        "eeg_port": 50000,
        "eeg_device": 'neurone',
        "turbolink_sampling_frequency": 5000,
        "turbolink_eeg_channel_count": 64,
        "maximum_dropped_samples": 2,
        # LabJack Configuration
        "enable_labjack": True,
        # Timing Configuration
        "maximum_loopback_latency": 0.005,
        "maximum_timing_error": 0.0,
        "trigger_to_pulse_delay": 0.0,
        # Disk Space Monitoring Configuration
        "disk_warning_threshold": '100GiB',
        "disk_error_threshold": '50GiB',
        # System Configuration
        "locale": 'en-US',
    }

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

    # System config

    def config_path(self):
        return os.path.join(self.PROJECTS_ROOT, self.CONFIG_FILENAME)

    def legacy_config_path(self):
        return os.path.join(self.PROJECTS_ROOT, self.LEGACY_CONFIG_FILENAME)

    def migrate_legacy_config(self):
        """Migrate global_config.json to system_config.json if needed.

        Does nothing if the new file already exists or the legacy file is absent.
        The legacy file is kept as a '.migrated' backup rather than deleted, so the
        migration is reversible and never runs twice.

        Returns the migrated config, or None if no migration was performed.
        """
        path = self.config_path()
        legacy_path = self.legacy_config_path()

        if os.path.exists(path) or not os.path.exists(legacy_path):
            return None

        config = self._load_json(legacy_path)
        if config is None:
            return None

        self._save_json(path, config)
        backup_path = legacy_path + '.migrated'
        os.replace(legacy_path, backup_path)

        self.logger.info(
            f"Migrated {self.LEGACY_CONFIG_FILENAME} to {self.CONFIG_FILENAME}; "
            f"previous file kept as {os.path.basename(backup_path)}."
        )
        return config

    def load_system_config(self):
        config = self.migrate_legacy_config()

        if config is None:
            config = self._load_json(self.config_path())

        if config is None:
            self.logger.info("System config not found, creating new one.")
            return self.initialize_system_config()

        # Fill in any keys missing from the stored file, so callers can rely on
        # every field being present.
        return {**self.DEFAULT_CONFIG, **config}

    def save_system_config(self, config):
        self._save_json(self.config_path(), config)

    def initialize_system_config(self):
        config = dict(self.DEFAULT_CONFIG)
        self.save_system_config(config)
        return config

    def validate_system_config(self, config):
        for key in self.DEFAULT_CONFIG:
            if key not in config:
                self.logger.error(f"System config is missing required key: {key}")
                return False
        return True

    def get_system_config(self):
        """Load and validate system config, reinitializing if needed."""
        config = self.load_system_config()
        if not self.validate_system_config(config):
            self.logger.error("Reinitializing system config.")
            config = self.initialize_system_config()
        self.reconcile_active_project(config)
        return config

    def reconcile_active_project(self, config):
        """Ensure active_project points to a project that still exists on disk.

        If the stored active project has been deleted, fall back to the first
        available project (or '' if none exist) and persist the correction, so
        downstream nodes never receive a project name without a directory.
        """
        projects = self.list_projects()
        active_project = config.get("active_project")

        if active_project in projects:
            return

        fallback = projects[0] if projects else ''
        self.logger.warning(
            f"Active project '{active_project}' no longer exists; "
            f"falling back to '{fallback}'."
        )
        config["active_project"] = fallback
        self.save_system_config(config)

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
        config = self.load_system_config()
        return config["active_project"]

    def save_active_project(self, project_name):
        """Update the system config with the active project and save it."""
        config = self.load_system_config()
        config["active_project"] = project_name
        self.save_system_config(config)
