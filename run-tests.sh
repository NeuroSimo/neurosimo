#!/usr/bin/env bash
# Run NeuroSimo session fingerprint tests (see tests/README.md).
#
# Prerequisite: the main NeuroSimo stack must already be running, e.g.
#   docker compose up
#
# Usage:
#   ./run-tests.sh              # rebuild test image and run tests
#   ./run-tests.sh --skip-build # run tests without rebuilding the image

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

SKIP_BUILD=false

usage() {
    cat <<'EOF'
Usage: ./run-tests.sh [--skip-build]

Runs session fingerprint integration tests in Docker.
NeuroSimo must already be running (docker compose up).

Environment:
  PROJECTS_ROOT  Host path mounted as /app/projects (default: ~/projects)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if ! command -v docker >/dev/null 2>&1; then
    echo "Error: docker is not installed or not on PATH." >&2
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo "Error: docker compose is not available." >&2
    exit 1
fi

if [[ ! -f .env ]]; then
    echo "Error: missing .env in repo root (see ansible/files/.env.example)." >&2
    exit 1
fi

if [[ -z "${PROJECTS_ROOT:-}" ]]; then
    export PROJECTS_ROOT="${HOME}/projects"
    echo "PROJECTS_ROOT not set; using ${PROJECTS_ROOT}"
fi

mkdir -p "${PROJECTS_ROOT}"

echo "Running fingerprint tests (project name: test)."
echo "The test project at ${PROJECTS_ROOT}/test will be recreated."
echo

COMPOSE=(docker compose -f docker-compose.tests.yml)

"${COMPOSE[@]}" down tests 2>/dev/null || true

if [[ "${SKIP_BUILD}" == false ]]; then
    "${COMPOSE[@]}" build tests
fi

set +e
"${COMPOSE[@]}" run --rm tests
exit_code=$?
set -e

if [[ "${exit_code}" -eq 0 ]]; then
    echo
    echo "All tests passed."
else
    echo >&2
    echo "Tests failed (exit code ${exit_code})." >&2
fi

exit "${exit_code}"
