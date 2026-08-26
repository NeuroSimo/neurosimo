"""Analyze NeuroSimo stimulation-decision CSV exports.

All durations in the source CSV are seconds. Public report metrics are milliseconds.
"""
from __future__ import annotations

import csv
import math
import random
from collections import Counter
from pathlib import Path
from typing import Iterable, Sequence

REQUIRED_COLUMNS = {
    "attempt_in_session",
    "status",
    "timing_error",
    "has_timing_error",
    "loopback_latency_at_scheduling",
    "decision_eeg_device_processing_duration",
    "decision_decider_duration",
    "decision_preprocessor_duration",
    "decision_overhead_duration",
}
TERMINAL_STATUSES = {
    "pulse_observed",
    "missed",
    "loopback_latency_exceeded",
    "too_late",
    "error",
}
TRUE_VALUES = {"1", "true", "yes", "y"}

class QualificationError(ValueError):
    """Raised when an export cannot support a valid qualification report."""


def _bool(value: str) -> bool:
    return str(value).strip().lower() in TRUE_VALUES


def _finite_seconds(rows: Sequence[dict[str, str]], column: str, *, predicate=None) -> list[float]:
    values: list[float] = []
    for row in rows:
        if predicate is not None and not predicate(row):
            continue
        raw = str(row.get(column, "")).strip()
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError as exc:
            raise QualificationError(f"Column {column!r} contains a non-numeric value: {raw!r}") from exc
        if math.isfinite(value):
            values.append(value)
    return values


def quantile(values: Sequence[float], probability: float) -> float:
    """Linear-interpolated sample quantile (Hyndman-Fan type 7)."""
    if not values:
        raise QualificationError("Cannot calculate a quantile from zero observations")
    if not 0.0 <= probability <= 1.0:
        raise QualificationError("Quantile probability must be between zero and one")
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _bootstrap_quantile_ci(
    values: Sequence[float], probability: float, confidence: float, resamples: int, seed: int
) -> list[float] | None:
    if len(values) < 2 or resamples == 0:
        return None
    rng = random.Random(seed)
    n = len(values)
    estimates = []
    for _ in range(resamples):
        sample = [values[rng.randrange(n)] for _ in range(n)]
        estimates.append(quantile(sample, probability))
    alpha = 1.0 - confidence
    return [quantile(estimates, alpha / 2.0), quantile(estimates, 1.0 - alpha / 2.0)]


def summarize_seconds(
    values: Sequence[float], *, absolute: bool = False, resamples: int = 2000, seed: int = 20260826
) -> dict:
    """Summarize seconds as milliseconds with deterministic percentile bootstrap CIs."""
    transformed = [abs(v) if absolute else v for v in values]
    if not transformed:
        return {"unit": "ms", "n": 0, "median": None, "p95": None, "p99": None, "max": None,
                "bootstrap_95_ci": {"median": None, "p95": None, "p99": None}}
    ms = [v * 1000.0 for v in transformed]
    return {
        "unit": "ms",
        "n": len(ms),
        "median": quantile(ms, 0.50),
        "p95": quantile(ms, 0.95),
        "p99": quantile(ms, 0.99),
        "max": max(ms),
        "bootstrap_95_ci": {
            "median": _bootstrap_quantile_ci(ms, 0.50, 0.95, resamples, seed + 50),
            "p95": _bootstrap_quantile_ci(ms, 0.95, 0.95, resamples, seed + 95),
            "p99": _bootstrap_quantile_ci(ms, 0.99, 0.95, resamples, seed + 99),
        },
    }


def load_attempts(path: str | Path) -> list[dict[str, str]]:
    path = Path(path)
    if not path.is_file():
        raise QualificationError(f"Stimulation-decision CSV not found: {path}")
    with path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or [])
        missing = sorted(REQUIRED_COLUMNS - columns)
        if missing:
            raise QualificationError("Incompatible NeuroSimo export; missing columns: " + ", ".join(missing))
        rows = list(reader)
    if not rows:
        raise QualificationError("Stimulation-decision CSV contains no attempts")
    return rows


def analyze_attempts(
    rows: Sequence[dict[str, str]], *, bootstrap_resamples: int = 2000, seed: int = 20260826
) -> dict:
    statuses = Counter(str(row.get("status", "")).strip() or "unknown" for row in rows)
    nonterminal = sorted(status for status in statuses if status not in TERMINAL_STATUSES)
    if nonterminal:
        raise QualificationError("Expected finalized attempt traces, found non-terminal/unknown statuses: " + ", ".join(nonterminal))

    has_error = lambda row: _bool(row.get("has_timing_error", ""))
    signed_error = _finite_seconds(rows, "timing_error", predicate=has_error)
    loopback = _finite_seconds(rows, "loopback_latency_at_scheduling")
    eeg_device = _finite_seconds(rows, "decision_eeg_device_processing_duration")
    decider = _finite_seconds(rows, "decision_decider_duration")
    preprocessor = _finite_seconds(rows, "decision_preprocessor_duration")
    overhead = _finite_seconds(rows, "decision_overhead_duration")
    decision_total = [sum(parts) for parts in zip(eeg_device, decider, preprocessor, overhead)] if (
        len({len(eeg_device), len(decider), len(preprocessor), len(overhead)}) == 1
    ) else []

    usable = statuses.get("pulse_observed", 0)
    failures = len(rows) - usable
    warnings = []
    if len(signed_error) < 1000:
        warnings.append(
            f"Only {len(signed_error)} observed timing errors are available; tail estimates such as p99 are unstable."
        )
    if len(signed_error) != usable:
        warnings.append(
            f"{usable} pulse_observed attempts exist but {len(signed_error)} have a finite timing error."
        )
    if not loopback:
        warnings.append("No finite loopback_latency_at_scheduling values were available.")

    metric = lambda values, absolute=False: summarize_seconds(
        values, absolute=absolute, resamples=bootstrap_resamples, seed=seed
    )
    return {
        "attempts": {
            "total": len(rows),
            "status_counts": dict(sorted(statuses.items())),
            "successful": usable,
            "failed": failures,
            "failed_rate": failures / len(rows),
        },
        "metrics": {
            "signed_timing_error": metric(signed_error),
            "absolute_timing_error": metric(signed_error, absolute=True),
            "loopback_latency_at_scheduling": metric(loopback),
            "eeg_device_processing_duration": metric(eeg_device),
            "decider_duration": metric(decider),
            "preprocessor_duration": metric(preprocessor),
            "overhead_duration": metric(overhead),
            "decision_total_duration": metric(decision_total),
        },
        "warnings": warnings,
    }
