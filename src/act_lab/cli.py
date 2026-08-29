"""Stable command-line entry point for all ACT Lab workflows."""

from __future__ import annotations

import argparse
import json
import platform
import sys
from collections.abc import Sequence

from act_lab import __version__


def _doctor(as_json: bool) -> int:
    report = {
        "act_lab_version": __version__,
        "python": platform.python_version(),
        "python_supported": sys.version_info[:2] in {(3, 11), (3, 12)},
        "status": "ok",
    }
    if not report["python_supported"]:
        report["status"] = "error"

    if as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        for key, value in report.items():
            print(f"{key}: {value}")
    return 0 if report["status"] == "ok" else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="act-lab")
    parser.add_argument("--version", action="version", version=__version__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor = subparsers.add_parser("doctor", help="check the runtime environment")
    doctor.add_argument(
        "--json", action="store_true", help="emit machine-readable JSON"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "doctor":
        return _doctor(as_json=args.json)
    raise AssertionError(f"unhandled command: {args.command}")
