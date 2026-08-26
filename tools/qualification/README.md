# NeuroSimo temporal qualification (proposed PR 1)

This standalone, offline tool creates a deterministic temporal-qualification report from NeuroSimo's current `stimulation_decisions.csv` export. It does not modify ROS 2 nodes or make medical-safety claims.

## Why

NeuroSimo records finalized `AttemptTrace` messages with timing errors, loopback latency at scheduling, decision latency components and terminal statuses. Existing fingerprint tests cover functional reproducibility; this analyzer makes temporal behavior reviewable and archivable.

## Install and run

From the NeuroSimo repository root:

```bash
python3 -m venv .venv-qualification
. .venv-qualification/bin/activate
python -m pip install -e tools/qualification

neurosimo-qualify /path/to/stimulation_decisions.csv \
  --metadata /path/to/session-sidecar.json \
  --json qualification.json \
  --markdown qualification.md
```

The metadata file is optional. Use the JSON sidecar next to a NeuroSimo recording when available.

Run directly without installation:

```bash
PYTHONPATH=tools/qualification \
python -m neurosimo_qualification.cli \
  /path/to/stimulation_decisions.csv \
  --json qualification.json \
  --markdown qualification.md
```

## Test

```bash
python -m pip install pytest
PYTHONPATH=tools/qualification pytest -q tools/qualification/tests
```

The committed fixture is derived from real, checksum-verified data in Zenodo record [14398634](https://doi.org/10.5281/zenodo.14398634). See `tests/fixtures/PROVENANCE.md`. It is a software fixture, not an estimate based on the complete experiment.

## Output and method

- Source durations are seconds; report durations are milliseconds.
- Quantiles use Hyndman-Fan type 7 linear interpolation.
- Median, p95 and p99 receive deterministic percentile-bootstrap 95% confidence intervals.
- The default bootstrap uses 2,000 resamples and seed `20260826`; both are recorded.
- Signed timing error detects bias; absolute timing error measures precision.
- Every terminal status remains in the denominator. Failed attempts are not silently removed.
- A warning is emitted when fewer than 1,000 finite timing errors are available for p99.
- Reports contain the input filename and SHA-256 and are byte-stable for identical inputs and parameters.

## Current scope

PR 1 intentionally analyzes the existing CSV boundary. It does **not** yet:

- export continuous `/neurosimo/pipeline/latency/loopback` samples, because the current exporter has no loopback export type;
- automate ROS 2 sessions;
- inject deterministic network faults;
- define universal pass/fail thresholds;
- certify a medical device.

Those should be discussed separately after the maintainers approve this analysis boundary.
