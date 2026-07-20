#!/usr/bin/env python3
"""Regenerate all Week 1 outputs."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def run(command: list[str]) -> None:
    print(f"$ {' '.join(command)}")
    subprocess.run(command, cwd=PROJECT_ROOT, check=True)


def main() -> None:
    run([sys.executable, "scripts/run_week1_ascent.py"])
    run([sys.executable, "scripts/plot_week1_outputs.py"])
    run([sys.executable, "scripts/write_week1_report.py"])
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests"])


if __name__ == "__main__":
    main()
