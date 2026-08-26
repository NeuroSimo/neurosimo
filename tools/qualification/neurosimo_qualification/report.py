"""Build stable JSON and Markdown qualification reports."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "1.0"

def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_report(*, analysis: dict, source: str | Path, metadata: dict | None, bootstrap_resamples: int, seed: int) -> dict:
    source = Path(source)
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_by": "neurosimo-qualification 0.1.0",
        "scope": "research_system_temporal_qualification_not_medical_device_certification",
        "source": {"filename": source.name, "sha256": sha256_file(source)},
        "method": {
            "quantile_estimator": "Hyndman-Fan type 7 linear interpolation",
            "bootstrap_interval": "deterministic percentile bootstrap, 95% confidence",
            "bootstrap_resamples": bootstrap_resamples,
            "random_seed": seed,
            "input_time_unit": "s",
            "report_time_unit": "ms",
        },
        "metadata": metadata or {},
        **analysis,
    }


def write_json(report: dict, path: str | Path) -> None:
    Path(path).write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def _number(value: Any) -> str:
    return "NA" if value is None else f"{value:.6g}"


def write_markdown(report: dict, path: str | Path) -> None:
    lines = [
        "# NeuroSimo temporal qualification report",
        "",
        "> Research-system qualification only. This report is not medical-device certification.",
        "",
        f"- Source SHA-256: `{report['source']['sha256']}`",
        f"- Attempts: {report['attempts']['total']}",
        f"- Failed-attempt rate: {report['attempts']['failed_rate']:.3%}",
        "",
        "| Metric | n | Median (ms) | p95 (ms) | p99 (ms) | Max (ms) |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, metric in report["metrics"].items():
        lines.append(
            f"| `{name}` | {metric['n']} | {_number(metric['median'])} | {_number(metric['p95'])} | "
            f"{_number(metric['p99'])} | {_number(metric['max'])} |"
        )
    lines += ["", "## Attempt statuses", ""]
    for status, count in report["attempts"]["status_counts"].items():
        lines.append(f"- `{status}`: {count}")
    if report["warnings"]:
        lines += ["", "## Warnings", ""] + [f"- {warning}" for warning in report["warnings"]]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
