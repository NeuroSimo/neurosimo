#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /opt/neurosimo_tests/install/setup.bash

set -euo pipefail

cd /opt/neurosimo_tests/tests

export PROJECTS_ROOT="/app/projects"
export PROJECT_NAME="test"

safe_delete_test_project() {
    local target_project_dir="${PROJECTS_ROOT}/${PROJECT_NAME}"

    if [ "${PROJECTS_ROOT}" != "/app/projects" ] || [ "${PROJECT_NAME}" != "test" ]; then
        echo "Error: refusing to remove unexpected project path: ${target_project_dir}"
        exit 1
    fi

    rm -rf -- "${target_project_dir}"
}

safe_delete_test_project

(
    cd helpers
    bash ./create-test-project
)

run_case() {
    local case_name="$1"
    local preprocessor_enabled="$2"
    local preprocessor_module="$3"
    local decider_enabled="$4"
    local decider_module="$5"
    local simulator_dataset_filename="$6"
    local protocol_filename="$7"
    local data_source_fingerprint="$8"
    local preprocessor_fingerprint="$9"
    local decision_fingerprint="${10}"

    echo "=== Running fingerprint case: ${case_name} ==="
    if [ "${preprocessor_enabled}" = "true" ]; then
        echo "Preprocessor: ${preprocessor_module}"
    fi
    if [ "${decider_enabled}" = "true" ]; then
        echo "Decider: ${decider_module}"
    fi

    export DECIDER_MODULE="${decider_module}"
    export PREPROCESSOR_MODULE="${preprocessor_module}"
    export DECIDER_ENABLED="${decider_enabled}"
    export PREPROCESSOR_ENABLED="${preprocessor_enabled}"
    export SIMULATOR_DATASET_FILENAME="${simulator_dataset_filename}"
    export EXPERIMENT_PROTOCOL_FILENAME="${protocol_filename}"

    export DATA_SOURCE_FINGERPRINT="${data_source_fingerprint}"
    export PREPROCESSOR_FINGERPRINT="${preprocessor_fingerprint}"
    export DECISION_FINGERPRINT="${decision_fingerprint}"

    if ! pytest -c pytest.ini -o cache_dir=/tmp/pytest_cache -m tests -vv test_fingerprints.py; then
        echo "Test case '${case_name}' failed. Sleeping forever for container debugging."
        sleep infinity
    fi
}

# XXX: example_long.yaml consisting of 100 trials needs to be used here, otherwise the protocol will reach its end before the dataset ends,
#   causing the data source fingerprint to be non-deterministic: there can be a few sample differences in when the session stops if the
#   protocol ends before the dataset ends. The indeterminism should be eventually fixed, but for now just use example_long.yaml to avoid the issue.
run_case \
    "rtsound_with_phastimate" \
    "true" \
    "rtsound_deterministic.py" \
    "true" \
    "phastimate.py" \
    "test_data_with_pulse_62_eeg_channels.json" \
    "example_long.yaml" \
    "10231642243859112162" \
    "4523506742077868031" \
    "4045430255371611672"

# Note that sample window tester (sample_window.py) only works with the test data with timestamp values.
run_case \
    "sample_window" \
    "false" \
    "" \
    "true" \
    "sample_window.py" \
    "test_data_with_timestamp_values.json" \
    "example_long.yaml" \
    "13667919222937123031" \
    "0" \
    "17608087463065513595"

run_case \
    "example" \
    "true" \
    "example.py" \
    "true" \
    "example.py" \
    "test_data_with_pulse.json" \
    "example_long.yaml" \
    "1947103796442606795" \
    "2440193878910607350" \
    "16539190759114767510"

# TODO: RTsound default lead field matrix works only with 62 EEG channels. Unify so that we don't need to generate separate test data to test RTsound.
run_case \
    "rtsound_with_block_at_pulse" \
    "true" \
    "rtsound_deterministic.py" \
    "true" \
    "block_at_pulse.py" \
    "test_data_with_pulse_62_eeg_channels.json" \
    "example_long.yaml" \
    "10231642243859112162" \
    "4523506742077868031" \
    "16539190759114767510"

#tests-1  | >       assert fingerprints.get("data_source") == expected_data_source_fingerprint
#tests-1  | E       AssertionError: assert 2901333299821517825 == 2690643853576505113
#tests-1  | E        +  where 2901333299821517825 = <built-in method get of dict object at 0x74f8a9e7dc80>('data_source')
#tests-1  | E        +    where <built-in method get of dict object at 0x74f8a9e7dc80> = {'data_source': 2901333299821517825, 'decision': 13072849512627603760, 'preprocessor': 10003941458884318604}.get

#tests-1  | >       assert fingerprints.get("data_source") == expected_data_source_fingerprint
#tests-1  | E       AssertionError: assert 12266791253706778131 == 2690643853576505113
#tests-1  | E        +  where 12266791253706778131 = <built-in method get of dict object at 0x76d9f0269f80>('data_source')
#tests-1  | E        +    where <built-in method get of dict object at 0x76d9f0269f80> = {'data_source': 12266791253706778131, 'decision': 4045430255371611672, 'preprocessor': 14919080678851062454}.get

#tests-1  | E       AssertionError: assert 1435526355411471748 == 2690643853576505113
#tests-1  | E        +  where 1435526355411471748 = <built-in method get of dict object at 0x750e27512000>('data_source')
#tests-1  | E        +    where <built-in method get of dict object at 0x750e27512000> = {'data_source': 1435526355411471748, 'decision': 13072849512627603760, 'preprocessor': 879394693176833235}.get
#tests-1  | 

echo "All fingerprint test cases completed."
