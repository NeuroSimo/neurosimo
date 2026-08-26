"""Command-line interface for NeuroSimo temporal qualification."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .analysis import QualificationError, analyze_attempts, load_attempts
from .report import build_report, write_json, write_markdown


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Create a temporal qualification report from a NeuroSimo stimulation_decisions CSV export.")
    p.add_argument("attempts_csv", help="NeuroSimo stimulation_decisions CSV")
    p.add_argument("--metadata", help="Optional session sidecar JSON")
    p.add_argument("--json", dest="json_path", default="qualification.json", help="Output JSON path")
    p.add_argument("--markdown", dest="markdown_path", help="Optional Markdown report path")
    p.add_argument("--bootstrap-resamples", type=int, default=2000)
    p.add_argument("--seed", type=int, default=20260826)
    return p


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    if args.bootstrap_resamples < 0:
        print("error: --bootstrap-resamples must be non-negative", file=sys.stderr)
        return 2
    try:
        rows = load_attempts(args.attempts_csv)
        metadata = json.loads(Path(args.metadata).read_text()) if args.metadata else None
        analysis = analyze_attempts(rows, bootstrap_resamples=args.bootstrap_resamples, seed=args.seed)
        report = build_report(
            analysis=analysis, source=args.attempts_csv, metadata=metadata,
            bootstrap_resamples=args.bootstrap_resamples, seed=args.seed,
        )
        write_json(report, args.json_path)
        if args.markdown_path:
            write_markdown(report, args.markdown_path)
    except (QualificationError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
