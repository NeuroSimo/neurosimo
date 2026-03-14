# Session fingerprint tests

This suite drives the system through the same public ROS2 interfaces used by the frontend:

- `/<node>/set_parameters` for configuration
- `/neurosimo/session/start` to start a session
- `/neurosimo/session/state` to wait for lifecycle completion

After the session finishes, it reads the latest recording metadata JSON from:

- `/app/projects/<active_project>/recordings/*.json`

and verifies expected fingerprint values:

- `data_source`
- `preprocessor`
- `decision`

## Running via docker compose

While NeuroSimo is operational, run:

```bash
docker compose -f docker-compose.tests.yml down tests && \
docker compose -f docker-compose.tests.yml build tests && \
docker compose -f docker-compose.tests.yml up tests
```

The test uses the project name `test`, and will recreate it at the start.

The container entrypoint runs `tests/helpers/run-fingerprint-tests.sh`, which executes multiple fingerprint test cases with different decider/preprocessor settings.
