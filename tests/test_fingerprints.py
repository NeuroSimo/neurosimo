import json
import os
import time
from pathlib import Path
from typing import Any

import pytest
import rclpy
from rclpy.node import Node

from neurosimo_project_interfaces.srv import ListProjects
from neurosimo_system_interfaces.msg import SessionState, SessionConfig
from neurosimo_system_interfaces.srv import GetSystemConfig, SetSystemConfig, StartSession


pytestmark = [pytest.mark.tests]


def require_env(name: str) -> str:
    value = os.getenv(name, "").strip()
    if not value:
        raise RuntimeError(f"{name} must be set")
    return value


class SessionTestsHarness:
    def __init__(self, node: Node) -> None:
        self.node = node
        self._states: list[int] = []
        self._last_abort_reason = ""
        self._session_state_sub = self.node.create_subscription(
            SessionState,
            "/neurosimo/session/state",
            self.session_state_callback,
            10,
        )

    def session_state_callback(self, msg: SessionState) -> None:
        self._states.append(msg.state)
        self._last_abort_reason = msg.abort_reason

    def wait_for_service(self, client: Any, name: str, timeout_sec: float = 30.0) -> None:
        started = time.time()
        while time.time() - started < timeout_sec:
            if client.wait_for_service(timeout_sec=1.0):
                return
        raise TimeoutError(f"Service not available: {name}")

    def call_service(self, client: Any, request: Any, timeout_sec: float = 30.0) -> Any:
        future = client.call_async(request)
        started = time.time()
        while time.time() - started < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                return future.result()
        raise TimeoutError("Timed out waiting for service response")

    def assert_project_exists(self, project_name: str) -> None:
        list_client = self.node.create_client(ListProjects, "/neurosimo/projects/list")
        self.wait_for_service(list_client, "/neurosimo/projects/list")
        list_response = self.call_service(list_client, ListProjects.Request())
        if not list_response.success:
            raise RuntimeError("Failed to list projects")
        if project_name not in list_response.projects:
            raise RuntimeError(
                f"Expected project '{project_name}' to be created by run-fingerprint-tests.sh. "
                f"Available projects: {list_response.projects}"
            )

    def update_system_config(self, updates: dict[str, Any]) -> None:
        """Read the current system config, apply updates, and write it back.

        The set service is a full replace, so the request has to be based on the
        current configuration.
        """
        get_name = "/neurosimo/system_configurator/config/get"
        get_client = self.node.create_client(GetSystemConfig, get_name)
        self.wait_for_service(get_client, get_name)
        get_response = self.call_service(get_client, GetSystemConfig.Request())
        if not get_response.success:
            raise RuntimeError("Failed to get system config")

        config = get_response.config
        for key, value in updates.items():
            if not hasattr(config, key):
                raise AttributeError(f"SystemConfig has no field '{key}'")
            setattr(config, key, value)

        set_name = "/neurosimo/system_configurator/config/set"
        set_client = self.node.create_client(SetSystemConfig, set_name)
        self.wait_for_service(set_client, set_name)
        request = SetSystemConfig.Request()
        request.config = config
        set_response = self.call_service(set_client, request)
        if not set_response.success:
            raise RuntimeError(f"Failed to set system config: {set_response.message}")

    def start_session(self, config: SessionConfig) -> None:
        client = self.node.create_client(StartSession, "/neurosimo/session/start")
        self.wait_for_service(client, "/neurosimo/session/start")
        request = StartSession.Request()
        request.config = config
        response = self.call_service(client, request)
        if not response.success:
            raise RuntimeError(f"Session start failed: {response.message}")

    def wait_for_state(self, expected_state: int, timeout_sec: float) -> None:
        started = time.time()
        while time.time() - started < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if expected_state in self._states:
                return
        if expected_state == SessionState.RUNNING:
            raise TimeoutError(
                "Session failed to start: RUNNING state was never observed. "
                f"seen={self._states}, last_abort_reason={self._last_abort_reason!r}"
            )
        raise TimeoutError(
            f"Timed out waiting for state={expected_state}, seen={self._states}, "
            f"last_abort_reason={self._last_abort_reason!r}"
        )

    def wait_for_stopped_after_running(self, timeout_sec: float) -> None:
        started = time.time()
        saw_running = SessionState.RUNNING in self._states
        while time.time() - started < timeout_sec:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if SessionState.RUNNING in self._states:
                saw_running = True
            if saw_running and self._states and self._states[-1] == SessionState.STOPPED:
                return
        if not saw_running:
            raise TimeoutError(
                "Session failed to start: RUNNING state was never observed while waiting for "
                "STOPPED. "
                f"seen={self._states}, last_abort_reason={self._last_abort_reason!r}"
            )
        raise TimeoutError(
            "Timed out waiting for STOPPED after RUNNING. "
            f"seen={self._states}, last_abort_reason={self._last_abort_reason!r}"
        )

    @property
    def states(self) -> list[int]:
        return list(self._states)


@pytest.fixture(scope="session")
def ros_node() -> Node:
    rclpy.init()
    node = Node("session_fingerprint_tests_test")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope="session")
def projects_root() -> Path:
    return Path(require_env("PROJECTS_ROOT"))


def find_latest_recording_metadata(recordings_dir: Path) -> Path:
    candidates = sorted(recordings_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    for candidate in candidates:
        if (recordings_dir / candidate.stem).is_dir():
            return candidate
    raise FileNotFoundError(f"No recording metadata JSON found in {recordings_dir}")


def test_fingerprints(ros_node: Node, projects_root: Path) -> None:
    harness = SessionTestsHarness(ros_node)

    project_name = require_env("PROJECT_NAME")
    harness.assert_project_exists(project_name)

    harness.update_system_config(
        {
            "active_project": project_name,
            "enable_labjack": True,
        },
    )

    # The system config still reaches the session manager over a latched topic, so allow it
    # to propagate before starting. The session config no longer needs this: it is carried
    # by the start request itself.
    time.sleep(0.5)

    decider_enabled = require_env("DECIDER_ENABLED") == "true"
    preprocessor_enabled = require_env("PREPROCESSOR_ENABLED") == "true"

    decider_module = os.getenv("DECIDER_MODULE", "").strip()
    preprocessor_module = os.getenv("PREPROCESSOR_MODULE", "").strip()

    if decider_enabled and not decider_module:
        raise RuntimeError("DECIDER_MODULE must be set when decider is enabled")

    if preprocessor_enabled and not preprocessor_module:
        raise RuntimeError("PREPROCESSOR_MODULE must be set when preprocessor is enabled")

    simulator_dataset_filename = require_env("SIMULATOR_DATASET_FILENAME")
    experiment_protocol_filename = require_env("EXPERIMENT_PROTOCOL_FILENAME")

    config = SessionConfig()
    config.subject_id = 777
    config.notes = "fingerprint_test"
    config.data_source = "simulator"
    config.simulator_dataset_filename = simulator_dataset_filename
    config.simulator_start_time = 0.0
    config.simulator_playback_speed = 1.0
    config.protocol_filename = experiment_protocol_filename
    config.runtime_parameters = "{}"
    config.decider_enabled = decider_enabled
    config.decider_module = decider_module
    config.preprocessor_enabled = preprocessor_enabled
    config.preprocessor_module = preprocessor_module
    config.presenter_enabled = False
    config.presenter_module = "example.py"
    config.replay_bag_id = ""
    config.replay_play_preprocessed = False

    harness.start_session(config)
    harness.wait_for_state(SessionState.RUNNING, timeout_sec=60.0)
    harness.wait_for_stopped_after_running(timeout_sec=180.0)

    recordings_dir = projects_root / project_name / "recordings"
    metadata_path = find_latest_recording_metadata(recordings_dir)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    fingerprints = metadata.get("provenance", {}).get("fingerprints", {})

    expected_data_source_fingerprint = int(require_env("DATA_SOURCE_FINGERPRINT"), 0)
    expected_preprocessor_fingerprint = int(require_env("PREPROCESSOR_FINGERPRINT"), 0)
    expected_decision_fingerprint = int(require_env("DECISION_FINGERPRINT"), 0)

    assert fingerprints.get("data_source") == expected_data_source_fingerprint
    if preprocessor_enabled:
        assert fingerprints.get("preprocessor") == expected_preprocessor_fingerprint
    if decider_enabled:
        assert fingerprints.get("decision") == expected_decision_fingerprint

    assert SessionState.INITIALIZING in harness.states
    assert SessionState.RUNNING in harness.states
    assert SessionState.FINALIZING in harness.states
    assert harness.states[-1] == SessionState.STOPPED
