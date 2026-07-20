#!/usr/bin/env python3
"""Create dependency-free SVG plots from the Week 1 ascent CSV."""

from __future__ import annotations

import csv
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = PROJECT_ROOT / "outputs"
CSV_PATH = OUTPUT_DIR / "week1_ascent.csv"
SVG_PATH = OUTPUT_DIR / "week1_ascent_plots.svg"


def read_column(rows: list[dict[str, str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def points_for_series(
    xs: list[float],
    ys: list[float],
    left: float,
    top: float,
    width: float,
    height: float,
) -> str:
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if xmax == xmin:
        xmax += 1.0
    if ymax == ymin:
        ymax += 1.0
    pad = 0.05 * (ymax - ymin)
    ymin -= pad
    ymax += pad

    coords = []
    for x, y in zip(xs, ys):
        px = left + (x - xmin) / (xmax - xmin) * width
        py = top + height - (y - ymin) / (ymax - ymin) * height
        coords.append(f"{px:.2f},{py:.2f}")
    return " ".join(coords)


def panel(
    title: str,
    xs: list[float],
    ys: list[float],
    y_label: str,
    top: float,
    color: str,
) -> str:
    left = 86.0
    width = 680.0
    height = 150.0
    axis_bottom = top + height
    points = points_for_series(xs, ys, left, top, width, height)
    ymin, ymax = min(ys), max(ys)

    return f"""
  <text x="{left}" y="{top - 22}" class="title">{title}</text>
  <text x="22" y="{top + height / 2}" class="axis-label" transform="rotate(-90 22 {top + height / 2})">{y_label}</text>
  <line x1="{left}" y1="{axis_bottom}" x2="{left + width}" y2="{axis_bottom}" class="axis"/>
  <line x1="{left}" y1="{top}" x2="{left}" y2="{axis_bottom}" class="axis"/>
  <text x="{left}" y="{axis_bottom + 22}" class="tick">{xs[0]:.0f}s</text>
  <text x="{left + width - 28}" y="{axis_bottom + 22}" class="tick">{xs[-1]:.0f}s</text>
  <text x="{left - 74}" y="{axis_bottom}" class="tick">{ymin:.3g}</text>
  <text x="{left - 74}" y="{top + 4}" class="tick">{ymax:.3g}</text>
  <polyline points="{points}" fill="none" stroke="{color}" stroke-width="2.5"/>
"""


def main() -> None:
    with CSV_PATH.open() as f:
        rows = list(csv.DictReader(f))

    time_s = read_column(rows, "time_s")
    altitude_m = read_column(rows, "z_m")
    vertical_velocity_mps = read_column(rows, "vz_mps")
    q_norm_error = [abs(float(row["q_norm"]) - 1.0) for row in rows]

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="820" height="650" viewBox="0 0 820 650">
  <style>
    .bg {{ fill: #f8fafc; }}
    .title {{ font: 700 18px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; fill: #111827; }}
    .subtitle {{ font: 500 13px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; fill: #475569; }}
    .axis {{ stroke: #334155; stroke-width: 1.2; }}
    .axis-label {{ font: 600 12px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; fill: #334155; }}
    .tick {{ font: 500 11px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; fill: #64748b; }}
  </style>
  <rect class="bg" width="820" height="650"/>
  <text x="34" y="42" class="title">Week 1 Baseline Ascent: 6-DOF Dynamics Core</text>
  <text x="34" y="64" class="subtitle">Open-loop straight ascent, aligned thrust, no drag, RK4 integration, quaternion attitude propagation.</text>
{panel("Altitude", time_s, altitude_m, "z (m)", 110.0, "#2563eb")}
{panel("Vertical Velocity", time_s, vertical_velocity_mps, "vz (m/s)", 305.0, "#059669")}
{panel("Quaternion Norm Error", time_s, q_norm_error, "|q|-1", 500.0, "#dc2626")}
</svg>
"""
    OUTPUT_DIR.mkdir(exist_ok=True)
    SVG_PATH.write_text(svg)
    print(f"Wrote {SVG_PATH}")


if __name__ == "__main__":
    main()
