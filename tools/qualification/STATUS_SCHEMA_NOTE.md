# Current status-schema discrepancy (verified 2026-08-26)

The fresh `main` snapshot contains inconsistent numeric status definitions:

- `interfaces/neurosimo_pipeline_interfaces/msg/AttemptTrace.msg` defines six statuses: `1 scheduled`, `2 fired`, `3 pulse_processed`, `4 loopback_latency_exceeded`, `5 too_late`, `6 error`.
- `src/session_exporter/session_exporter/session_exporter.py` maps seven values: `1 scheduled`, `2 fired`, `3 pulse_observed`, `4 missed`, `5 loopback_latency_exceeded`, `6 too_late`, `7 error`.

The PR-1 analyzer consumes the exported text labels and retains all of these terminal labels. It cannot reconstruct the intended meaning after an inconsistent numeric-to-text conversion. This discrepancy should be confirmed with maintainers and fixed in a separate, minimal core PR before qualification reports are interpreted across versions.

No timing results in this archive were fabricated to work around the discrepancy.
