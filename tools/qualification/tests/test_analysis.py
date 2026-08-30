import csv
import json
from pathlib import Path

import pytest

from neurosimo_qualification.analysis import QualificationError, analyze_attempts, load_attempts, quantile
from neurosimo_qualification.report import build_report, write_json, write_markdown

FIXTURE = Path(__file__).parent / "fixtures" / "official_phastimate_subset.csv"


def test_type7_quantile_hand_checkable():
    assert quantile([0.0, 1.0, 2.0, 3.0], 0.5) == pytest.approx(1.5)
    assert quantile([0.0, 1.0, 2.0, 3.0], 0.95) == pytest.approx(2.85)


def test_official_fixture_summary_preserves_denominators_and_units():
    result = analyze_attempts(load_attempts(FIXTURE), bootstrap_resamples=100, seed=17)
    assert result["attempts"] == {
        "total": 12,
        "status_counts": {"pulse_observed": 12},
        "successful": 12,
        "failed": 0,
        "failed_rate": 0.0,
    }
    signed = result["metrics"]["signed_timing_error"]
    absolute = result["metrics"]["absolute_timing_error"]
    assert signed["unit"] == "ms"
    assert signed["n"] == 12
    assert signed["median"] == pytest.approx(0.2)
    assert absolute["p99"] == pytest.approx(0.4)
    assert any("p99" in warning for warning in result["warnings"])


def test_failure_status_is_retained(tmp_path):
    rows = load_attempts(FIXTURE)
    rows[0]["status"] = "too_late"
    rows[0]["has_timing_error"] = "False"
    rows[0]["timing_error"] = ""
    result = analyze_attempts(rows, bootstrap_resamples=0)
    assert result["attempts"]["status_counts"]["too_late"] == 1
    assert result["attempts"]["failed"] == 1
    assert result["metrics"]["signed_timing_error"]["n"] == 11


def test_nonterminal_export_is_rejected():
    rows = load_attempts(FIXTURE)
    rows[0]["status"] = "scheduled"
    with pytest.raises(QualificationError, match="finalized"):
        analyze_attempts(rows)


def test_obsolete_missed_status_is_rejected():
    rows = load_attempts(FIXTURE)
    rows[0]["status"] = "missed"
    with pytest.raises(QualificationError, match="finalized"):
        analyze_attempts(rows)


def test_missing_columns_are_actionable(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("status,timing_error\npulse_observed,0.001\n")
    with pytest.raises(QualificationError, match="missing columns"):
        load_attempts(path)


def test_normalized_reports_are_byte_identical(tmp_path):
    analysis = analyze_attempts(load_attempts(FIXTURE), bootstrap_resamples=50, seed=42)
    metadata = {"session_id": "fixture", "note": "official Zenodo-derived test fixture"}
    report = build_report(analysis=analysis, source=FIXTURE, metadata=metadata, bootstrap_resamples=50, seed=42)
    first, second = tmp_path / "a.json", tmp_path / "b.json"
    write_json(report, first); write_json(report, second)
    assert first.read_bytes() == second.read_bytes()
    parsed = json.loads(first.read_text())
    assert parsed["source"]["sha256"]
    md = tmp_path / "report.md"
    write_markdown(report, md)
    assert "not medical-device certification" in md.read_text()
