# Status-schema resolution (verified 2026-08-30)

`interfaces/neurosimo_pipeline_interfaces/msg/AttemptTrace.msg` is authoritative.
NeuroSimo commit
[`b969266`](https://github.com/NeuroSimo/neurosimo/commit/b969266cd7cfc23c7ad76f6da865e9e2ed3db125)
updated the CSV exporter to match it and removed the extra `missed` status.

The analyzer now accepts the finalized CSV labels `pulse_observed`,
`loopback_latency_exceeded`, `too_late`, and `error`. It rejects the obsolete
`missed` label and non-final states rather than silently interpreting them.
