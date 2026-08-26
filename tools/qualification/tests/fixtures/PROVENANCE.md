# Fixture provenance

`official_phastimate_subset.csv` is a 12-row schema-adaptation of real records from:

- Kahilakoski, O.-P. (2024). *Data and code for NeuroSimo: an open-source software for closed-loop EEG- or EMG-guided TMS*, Zenodo record 14398634, version v1.
- Source DOI: https://doi.org/10.5281/zenodo.14398634
- Source files: `phastimate_error_batch_1.csv`, `phastimate_trigger_info_batch_1.csv`, and stimulated rows of `phastimate_decision_info_batch_1.csv`.
- Source archive `data.zip` MD5: `b0dc91e132d7c38a01f6fc9a3bd2caf8` (verified 2026-08-26).
- Source dataset license: CC BY 4.0.

Rows preserve source values. `status=pulse_observed` and `has_timing_error=True` are inferred because each selected record has a successful trigger and measured timing error. Current-schema identifiers are sequential test identifiers. The fixture is for software verification, not for estimating the full experiment's performance.
