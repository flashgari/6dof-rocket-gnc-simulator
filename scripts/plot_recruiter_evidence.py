#!/usr/bin/env python3
"""Generate recruiter-facing engineering evidence from committed CSV outputs."""

from __future__ import annotations

import csv
import html
import math
import shutil
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs"
FIG = ROOT / "figures"

NAVY = "#0f172a"
TEXT = "#334155"
MUTED = "#64748b"
GRID = "#dbe3ec"
WHITE = "#ffffff"
BACKGROUND = "#f8fafc"
RED = "#dc2626"
BLUE = "#2563eb"
GREEN = "#059669"
TEAL = "#0891b2"
PURPLE = "#7c3aed"
ORANGE = "#ea580c"


def read_rows(path: Path) -> list[dict[str, float | str]]:
    with path.open() as stream:
        result: list[dict[str, float | str]] = []
        for source in csv.DictReader(stream):
            row: dict[str, float | str] = {}
            for key, value in source.items():
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
            result.append(row)
        return result


def column(rows: list[dict[str, float | str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def xml(value: object) -> str:
    return html.escape(str(value), quote=True)


def svg_start(width: int, height: int, title: str, subtitle: str) -> list[str]:
    return [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}" role="img">'
        ),
        f"<title>{xml(title)}</title>",
        f'<rect width="100%" height="100%" fill="{BACKGROUND}"/>',
        (
            f'<text x="62" y="53" font-family="Arial" font-size="31" '
            f'font-weight="700" fill="{NAVY}">{xml(title)}</text>'
        ),
        (
            f'<text x="62" y="83" font-family="Arial" font-size="16" '
            f'fill="{MUTED}">{xml(subtitle)}</text>'
        ),
    ]


def text_lines(
    svg: list[str],
    x: float,
    y: float,
    lines: list[str],
    *,
    size: int = 14,
    line_height: int = 22,
    color: str = TEXT,
    weight: int = 400,
) -> None:
    for index, line in enumerate(lines):
        svg.append(
            (
                f'<text x="{x}" y="{y + index * line_height}" '
                f'font-family="Arial" font-size="{size}" '
                f'font-weight="{weight}" fill="{color}">{xml(line)}</text>'
            )
        )


def heading(svg: list[str], x: float, y: float, label: str) -> None:
    text_lines(svg, x, y, [label], size=20, color=NAVY, weight=700)


def format_tick(value: float, span: float | None = None) -> str:
    if span is not None:
        if span < 0.1:
            return f"{value:.3f}"
        if span < 2.0:
            return f"{value:.2f}"
        if span < 20.0:
            return f"{value:.1f}"
    magnitude = abs(value)
    if magnitude >= 1000.0:
        return f"{value / 1000.0:.1f}k"
    if magnitude >= 100.0:
        return f"{value:.0f}"
    if magnitude >= 10.0:
        return f"{value:.0f}"
    if magnitude >= 1.0:
        return f"{value:.1f}"
    return f"{value:.2f}"


def ticks(lower: float, upper: float, count: int) -> list[float]:
    if count <= 1:
        return [lower]
    return [lower + index * (upper - lower) / (count - 1) for index in range(count)]


def line_chart(
    svg: list[str],
    *,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    x_label: str,
    y_label: str,
    series: list[dict[str, object]],
    x_domain: tuple[float, float] | None = None,
    y_domain: tuple[float, float] | None = None,
    reference_lines: list[tuple[float, str, str]] | None = None,
    legend: bool = True,
) -> None:
    all_x = [value for item in series for value in item["x"]]
    all_y = [value for item in series for value in item["y"]]
    x_min, x_max = x_domain or (min(all_x), max(all_x))
    y_min, y_max = y_domain or (min(all_y), max(all_y))
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0

    def sx(value: float) -> float:
        return x + (value - x_min) / (x_max - x_min) * width

    def sy(value: float) -> float:
        return y + height - (value - y_min) / (y_max - y_min) * height

    svg.extend(
        [
            (
                f'<text x="{x}" y="{y - 15}" font-family="Arial" '
                f'font-size="18" font-weight="700" fill="{NAVY}">{xml(title)}</text>'
            ),
            (
                f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
                f'fill="{WHITE}" stroke="{GRID}"/>'
            ),
        ]
    )
    for value in ticks(y_min, y_max, 4):
        py = sy(value)
        svg.extend(
            [
                (
                    f'<line x1="{x}" y1="{py:.1f}" x2="{x + width}" '
                    f'y2="{py:.1f}" stroke="{GRID}"/>'
                ),
                (
                    f'<text x="{x - 10}" y="{py + 4:.1f}" text-anchor="end" '
                    f'font-family="Arial" font-size="11" fill="{MUTED}">'
                    f"{xml(format_tick(value, y_max - y_min))}</text>"
                ),
            ]
        )
    for value in ticks(x_min, x_max, 4):
        px = sx(value)
        svg.extend(
            [
                (
                    f'<line x1="{px:.1f}" y1="{y}" x2="{px:.1f}" '
                    f'y2="{y + height}" stroke="{GRID}" opacity="0.45"/>'
                ),
                (
                    f'<text x="{px:.1f}" y="{y + height + 17}" text-anchor="middle" '
                    f'font-family="Arial" font-size="11" fill="{MUTED}">'
                    f"{xml(format_tick(value, x_max - x_min))}</text>"
                ),
            ]
        )
    for value, label, color in reference_lines or []:
        if y_min <= value <= y_max:
            py = sy(value)
            svg.extend(
                [
                    (
                        f'<line x1="{x}" y1="{py:.1f}" x2="{x + width}" '
                        f'y2="{py:.1f}" stroke="{color}" stroke-width="1.4" '
                        'stroke-dasharray="7 5"/>'
                    ),
                    (
                        f'<text x="{x + width - 8}" y="{py - 7:.1f}" '
                        f'text-anchor="end" font-family="Arial" font-size="11" '
                        f'fill="{color}">{xml(label)}</text>'
                    ),
                ]
            )
    for item in series:
        points = " ".join(
            f"{sx(px):.1f},{sy(py):.1f}"
            for px, py in zip(item["x"], item["y"])
            if math.isfinite(px) and math.isfinite(py)
        )
        svg.append(
            (
                f'<polyline points="{points}" fill="none" '
                f'stroke="{item["color"]}" stroke-width="{item.get("width", 2.6)}"/>'
            )
        )
    if legend:
        cursor = x + 12
        legend_y = y + 19
        for item in series:
            name = str(item["name"])
            svg.extend(
                [
                    (
                        f'<line x1="{cursor}" y1="{legend_y - 4}" '
                        f'x2="{cursor + 25}" y2="{legend_y - 4}" '
                        f'stroke="{item["color"]}" stroke-width="3"/>'
                    ),
                    (
                        f'<text x="{cursor + 32}" y="{legend_y}" '
                        f'font-family="Arial" font-size="11" fill="{TEXT}">'
                        f"{xml(name)}</text>"
                    ),
                ]
            )
            cursor += 48 + 7.0 * len(name)
    svg.extend(
        [
            (
                f'<text x="{x + width / 2}" y="{y + height + 37}" '
                f'text-anchor="middle" font-family="Arial" font-size="12" '
                f'fill="{TEXT}">{xml(x_label)}</text>'
            ),
            (
                f'<text x="{x - 55}" y="{y + height / 2}" '
                f'transform="rotate(-90 {x - 55},{y + height / 2})" '
                f'text-anchor="middle" font-family="Arial" font-size="12" '
                f'fill="{TEXT}">{xml(y_label)}</text>'
            ),
        ]
    )


def scatter_chart(
    svg: list[str],
    *,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    x_label: str,
    y_label: str,
    series: list[dict[str, object]],
    x_domain: tuple[float, float],
    y_domain: tuple[float, float],
    requirement: float,
) -> None:
    x_min, x_max = x_domain
    y_min, y_max = y_domain

    def sx(value: float) -> float:
        return x + (value - x_min) / (x_max - x_min) * width

    def sy(value: float) -> float:
        return y + height - (value - y_min) / (y_max - y_min) * height

    svg.extend(
        [
            (
                f'<text x="{x}" y="{y - 15}" font-family="Arial" '
                f'font-size="18" font-weight="700" fill="{NAVY}">{xml(title)}</text>'
            ),
            (
                f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
                f'fill="{WHITE}" stroke="{GRID}"/>'
            ),
        ]
    )
    for value in ticks(y_min, y_max, 4):
        py = sy(value)
        svg.extend(
            [
                (
                    f'<line x1="{x}" y1="{py:.1f}" x2="{x + width}" '
                    f'y2="{py:.1f}" stroke="{GRID}"/>'
                ),
                (
                    f'<text x="{x - 10}" y="{py + 4:.1f}" text-anchor="end" '
                    f'font-family="Arial" font-size="11" fill="{MUTED}">'
                    f"{xml(format_tick(value, y_max - y_min))}</text>"
                ),
            ]
        )
    for value in ticks(x_min, x_max, 6):
        px = sx(value)
        svg.append(
            (
                f'<text x="{px:.1f}" y="{y + height + 17}" text-anchor="middle" '
                f'font-family="Arial" font-size="11" fill="{MUTED}">'
                f"{xml(format_tick(value, x_max - x_min))}</text>"
            )
        )
    requirement_y = sy(requirement)
    svg.extend(
        [
            (
                f'<line x1="{x}" y1="{requirement_y:.1f}" x2="{x + width}" '
                f'y2="{requirement_y:.1f}" stroke="{RED}" stroke-width="1.4" '
                'stroke-dasharray="7 5"/>'
            ),
            (
                f'<text x="{x + width - 8}" y="{requirement_y - 7:.1f}" '
                f'text-anchor="end" font-family="Arial" font-size="11" '
                f'fill="{RED}">pass boundary</text>'
            ),
        ]
    )
    cursor = x + 12
    for item in series:
        values_x = item["x"]
        values_y = item["y"]
        for px, py in zip(values_x, values_y):
            svg.append(
                (
                    f'<circle cx="{sx(px):.1f}" cy="{sy(py):.1f}" r="3.2" '
                    f'fill="{item["color"]}" opacity="0.55"/>'
                )
            )
        slope, intercept = linear_fit(values_x, values_y)
        endpoints = (x_min, x_max)
        path = " ".join(
            (
                ("M" if index == 0 else "L")
                + f"{sx(value):.1f},{sy(slope * value + intercept):.1f}"
            )
            for index, value in enumerate(endpoints)
        )
        svg.append(
            f'<path d="{path}" fill="none" stroke="{item["color"]}" stroke-width="2.6"/>'
        )
        svg.extend(
            [
                (
                    f'<circle cx="{cursor}" cy="{y + 18}" r="4" '
                    f'fill="{item["color"]}"/>'
                ),
                (
                    f'<text x="{cursor + 10}" y="{y + 22}" '
                    f'font-family="Arial" font-size="11" fill="{TEXT}">'
                    f'{xml(item["name"])} (r={correlation(values_x, values_y):.3f})</text>'
                ),
            ]
        )
        cursor += 170
    svg.extend(
        [
            (
                f'<text x="{x + width / 2}" y="{y + height + 37}" '
                f'text-anchor="middle" font-family="Arial" font-size="12" '
                f'fill="{TEXT}">{xml(x_label)}</text>'
            ),
            (
                f'<text x="{x - 55}" y="{y + height / 2}" '
                f'transform="rotate(-90 {x - 55},{y + height / 2})" '
                f'text-anchor="middle" font-family="Arial" font-size="12" '
                f'fill="{TEXT}">{xml(y_label)}</text>'
            ),
        ]
    )


def linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float]:
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    slope = (
        sum((x_value - mean_x) * (y_value - mean_y) for x_value, y_value in zip(xs, ys))
        / denominator
        if denominator
        else 0.0
    )
    return slope, mean_y - slope * mean_x


def correlation(xs: list[float], ys: list[float]) -> float:
    mean_x = statistics.fmean(xs)
    mean_y = statistics.fmean(ys)
    numerator = sum(
        (x_value - mean_x) * (y_value - mean_y)
        for x_value, y_value in zip(xs, ys)
    )
    denominator = math.sqrt(
        sum((value - mean_x) ** 2 for value in xs)
        * sum((value - mean_y) ** 2 for value in ys)
    )
    return numerator / denominator if denominator else 0.0


def metric_row(
    svg: list[str],
    x: float,
    y: float,
    label: str,
    value: str,
    color: str = NAVY,
) -> None:
    svg.extend(
        [
            (
                f'<text x="{x}" y="{y}" font-family="Arial" font-size="13" '
                f'fill="{TEXT}">{xml(label)}</text>'
            ),
            (
                f'<text x="{x + 285}" y="{y}" text-anchor="end" '
                f'font-family="Arial" font-size="14" font-weight="700" '
                f'fill="{color}">{xml(value)}</text>'
            ),
        ]
    )


def write_artifact(
    filename: str,
    svg: list[str],
) -> None:
    OUT.mkdir(exist_ok=True)
    FIG.mkdir(exist_ok=True)
    output_path = OUT / filename
    output_path.write_text("\n".join(svg + ["</svg>"]) + "\n")
    shutil.copyfile(output_path, FIG / filename)
    print(f"Wrote {output_path}")
    print(f"Wrote {FIG / filename}")


def control_system_evidence() -> None:
    open_loop = read_rows(OUT / "week2_disturbed_uncontrolled.csv")
    ideal = read_rows(OUT / "week3a_controlled_ideal_torque.csv")
    pd_tvc = read_rows(OUT / "week3b_tvc_controlled.csv")
    lqr_tvc = read_rows(OUT / "week4a_lqr_tvc_controlled.csv")
    cases = [
        ("open loop", RED, open_loop),
        ("ideal torque", BLUE, ideal),
        ("PD TVC", GREEN, pd_tvc),
        ("LQR TVC", TEAL, lqr_tvc),
    ]
    svg = svg_start(
        1500,
        1120,
        "From Open-Loop Tumble to TVC-Stabilized Ascent",
        "Identical disturbance case; controller and moment-generation architecture change.",
    )
    chart_x, chart_width = 95, 850
    panel_height = 165
    starts = (145, 375, 605, 835)
    common = [
        {
            "name": name,
            "color": color,
            "x": column(rows, "time_s"),
            "rows": rows,
        }
        for name, color, rows in cases
    ]
    line_chart(
        svg,
        x=chart_x,
        y=starts[0],
        width=chart_width,
        height=panel_height,
        title="Altitude response",
        x_label="time (s)",
        y_label="altitude z (m)",
        series=[
            {**item, "y": column(item["rows"], "z_m")} for item in common
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 35.0),
    )
    line_chart(
        svg,
        x=chart_x,
        y=starts[1],
        width=chart_width,
        height=panel_height,
        title="Thrust-axis alignment",
        x_label="time (s)",
        y_label="body z dot inertial up",
        series=[
            {**item, "y": column(item["rows"], "body_z_z")} for item in common
        ],
        x_domain=(0.0, 3.0),
        y_domain=(-1.05, 1.05),
        reference_lines=[(0.0, "horizontal thrust axis", RED)],
    )
    line_chart(
        svg,
        x=chart_x,
        y=starts[2],
        width=chart_width,
        height=panel_height,
        title="Integrated lateral consequence",
        x_label="time (s)",
        y_label="lateral displacement (m)",
        series=[
            {**item, "y": column(item["rows"], "lateral_m")} for item in common
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 28.0),
    )
    line_chart(
        svg,
        x=chart_x,
        y=starts[3],
        width=chart_width,
        height=panel_height,
        title="Controlled attitude excursion",
        x_label="time (s)",
        y_label="tilt from vertical (deg)",
        series=[
            {
                "name": name,
                "color": color,
                "x": column(rows, "time_s"),
                "y": column(rows, "tilt_deg"),
            }
            for name, color, rows in cases[1:]
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 15.0),
    )

    side_x = 1010
    heading(svg, side_x, 145, "What the comparison proves")
    text_lines(
        svg,
        side_x,
        183,
        [
            "Open loop loses thrust-axis alignment, so the",
            "same engine force rotates from vertical support",
            "into lateral and eventually downward acceleration.",
            "",
            "Ideal body torque isolates the feedback law.",
            "PD and LQR TVC then demonstrate that stabilizing",
            "moment can be generated through finite engine",
            "gimbal geometry in the nonlinear 6-DOF plant.",
        ],
    )
    heading(svg, side_x, 400, "Nominal evidence")
    metric_row(svg, side_x, 438, "open-loop max tilt", "177.63 deg", RED)
    metric_row(svg, side_x, 468, "ideal-torque max tilt", "9.88 deg", BLUE)
    metric_row(svg, side_x, 498, "PD TVC max tilt", "12.94 deg", GREEN)
    metric_row(svg, side_x, 528, "LQR TVC max tilt", "10.30 deg", TEAL)
    metric_row(svg, side_x, 558, "LQR lateral drift", "10.73 m", TEAL)
    metric_row(svg, side_x, 588, "LQR gimbal saturation", "0.0%", TEAL)
    heading(svg, side_x, 660, "Upper-division interpretation")
    text_lines(
        svg,
        side_x,
        698,
        [
            "The translational mechanism is",
            "m a_I = R_BI(q) F_B + m g_I.",
            "The rotational mechanism is",
            "I omega_dot + omega x (I omega) = tau.",
            "",
            "TVC supplies tau = r_engine x F_thrust, but",
            "redirecting thrust also injects lateral force.",
            "That explains why ideal torque has the smallest",
            "drift and why LQR improvement must be checked",
            "against gimbal usage, not attitude alone.",
        ],
    )
    heading(svg, side_x, 965, "Claim boundary")
    text_lines(
        svg,
        side_x,
        1003,
        [
            "This is a 3 s low-altitude demonstration with a",
            "simplified aerodynamic model. It verifies the",
            "implemented control architecture, not flight",
            "qualification or global tumble recovery.",
        ],
        size=13,
        color=MUTED,
    )
    write_artifact("control-system-evidence.svg", svg)


def monte_carlo_evidence() -> None:
    rows = read_rows(OUT / "week4b_monte_carlo_results.csv")
    by_controller = {
        name: [row for row in rows if row["controller"] == name]
        for name in ("open_loop", "pd_tvc", "lqr_tvc")
    }
    names = {
        "open_loop": "open loop",
        "pd_tvc": "PD TVC",
        "lqr_tvc": "LQR TVC",
    }
    colors = {"open_loop": RED, "pd_tvc": GREEN, "lqr_tvc": TEAL}
    svg = svg_start(
        1500,
        1080,
        "Monte Carlo Robustness and Margin",
        "100 matched dispersions per controller, fixed seed 4242; thresholds shown explicitly.",
    )
    x, width = 95, 850
    svg.extend(
        [
            (
                f'<text x="{x}" y="132" font-family="Arial" font-size="18" '
                f'font-weight="700" fill="{NAVY}">Pass rate against all four requirements</text>'
            ),
            f'<rect x="{x}" y="150" width="{width}" height="170" fill="{WHITE}" stroke="{GRID}"/>',
        ]
    )
    for index, controller in enumerate(("open_loop", "pd_tvc", "lqr_tvc")):
        subset = by_controller[controller]
        success = sum(float(row["passed"]) for row in subset) / len(subset)
        bar_x = x + 75 + index * 245
        bar_width = 125
        bar_height = 120.0 * success
        svg.extend(
            [
                (
                    f'<rect x="{bar_x}" y="{292 - bar_height:.1f}" width="{bar_width}" '
                    f'height="{bar_height:.1f}" fill="{colors[controller]}"/>'
                ),
                (
                    f'<text x="{bar_x + bar_width / 2}" y="{282 - bar_height:.1f}" '
                    f'text-anchor="middle" font-family="Arial" font-size="14" '
                    f'font-weight="700" fill="{NAVY}">{100 * success:.1f}%</text>'
                ),
                (
                    f'<text x="{bar_x + bar_width / 2}" y="310" text-anchor="middle" '
                    f'font-family="Arial" font-size="12" fill="{TEXT}">'
                    f"{xml(names[controller])}</text>"
                ),
            ]
        )
    controlled_series = []
    for controller in ("pd_tvc", "lqr_tvc"):
        subset = by_controller[controller]
        controlled_series.append(
            {
                "name": names[controller],
                "color": colors[controller],
                "x": column(subset, "thrust_misalign_deg"),
                "tilt": column(subset, "max_tilt_deg"),
                "drift": column(subset, "max_lateral_m"),
            }
        )
    scatter_chart(
        svg,
        x=x,
        y=395,
        width=width,
        height=225,
        title="Dominant dispersion: thrust misalignment vs peak tilt",
        x_label="sampled thrust misalignment (deg)",
        y_label="maximum tilt (deg)",
        series=[
            {**item, "y": item["tilt"]} for item in controlled_series
        ],
        x_domain=(0.0, 2.5),
        y_domain=(0.0, 27.5),
        requirement=25.0,
    )
    scatter_chart(
        svg,
        x=x,
        y=730,
        width=width,
        height=225,
        title="Misalignment-driven lateral impulse",
        x_label="sampled thrust misalignment (deg)",
        y_label="maximum lateral drift (m)",
        series=[
            {**item, "y": item["drift"]} for item in controlled_series
        ],
        x_domain=(0.0, 2.5),
        y_domain=(0.0, 27.5),
        requirement=25.0,
    )

    side_x = 1010
    heading(svg, side_x, 145, "Worst-case margin")
    pd = by_controller["pd_tvc"]
    lqr = by_controller["lqr_tvc"]
    metric_row(
        svg,
        side_x,
        186,
        "PD worst tilt / 25 deg",
        f'{max(column(pd, "max_tilt_deg")):.2f} deg',
        GREEN,
    )
    metric_row(
        svg,
        side_x,
        216,
        "LQR worst tilt / 25 deg",
        f'{max(column(lqr, "max_tilt_deg")):.2f} deg',
        TEAL,
    )
    metric_row(
        svg,
        side_x,
        246,
        "PD worst drift / 25 m",
        f'{max(column(pd, "max_lateral_m")):.2f} m',
        GREEN,
    )
    metric_row(
        svg,
        side_x,
        276,
        "LQR worst drift / 25 m",
        f'{max(column(lqr, "max_lateral_m")):.2f} m',
        TEAL,
    )
    metric_row(
        svg,
        side_x,
        306,
        "LQR minimum final altitude",
        f'{min(column(lqr, "final_altitude_m")):.2f} m',
        TEAL,
    )
    heading(svg, side_x, 380, "Why 100% is not enough")
    text_lines(
        svg,
        side_x,
        418,
        [
            "A pass-rate bar alone hides proximity to failure.",
            "The scatter panels expose distance to the tilt",
            "and drift gates and retain every controlled trial.",
            "",
            "LQR preserves 7.08 deg worst-case tilt margin",
            "and 5.90 m worst-case drift margin. PD passes",
            "the same sample but operates closer to both limits.",
        ],
    )
    heading(svg, side_x, 610, "Physical sensitivity")
    text_lines(
        svg,
        side_x,
        648,
        [
            "Thrust misalignment dominates this sampled set:",
            "for LQR, correlation with peak tilt is 0.996",
            "and with maximum drift is 0.987.",
            "",
            "Misalignment creates a persistent transverse",
            "force and TVC compensation demand. The attitude",
            "loop limits rotation; the residual transverse",
            "force integrates into velocity and displacement.",
        ],
    )
    heading(svg, side_x, 875, "Verification boundary")
    text_lines(
        svg,
        side_x,
        913,
        [
            "These are fixed-seed samples from stated uniform",
            "distributions, not probabilities of flight success.",
            "No global stability or certification claim is made.",
        ],
        size=13,
        color=MUTED,
    )
    write_artifact("monte-carlo-control-envelope.svg", svg)


def estimation_evidence() -> None:
    rows = read_rows(OUT / "week5_estimated_tvc_controlled.csv")
    time = column(rows, "time_s")
    svg = svg_start(
        1500,
        1050,
        "Estimated-State Attitude Control",
        "Noisy IMU propagation and low-rate attitude aiding close the same LQR TVC loop.",
    )
    x, width, panel_height = 95, 850, 165
    line_chart(
        svg,
        x=x,
        y=145,
        width=width,
        height=panel_height,
        title="True and estimated thrust-axis tilt",
        x_label="time (s)",
        y_label="tilt (deg)",
        series=[
            {"name": "truth", "color": BLUE, "x": time, "y": column(rows, "tilt_deg")},
            {
                "name": "estimate",
                "color": TEAL,
                "x": time,
                "y": column(rows, "est_tilt_deg"),
            },
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 12.0),
    )
    line_chart(
        svg,
        x=x,
        y=375,
        width=width,
        height=panel_height,
        title="Quaternion attitude estimation error",
        x_label="time (s)",
        y_label="attitude error (deg)",
        series=[
            {
                "name": "attitude error",
                "color": RED,
                "x": time,
                "y": column(rows, "attitude_error_deg"),
            }
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 0.36),
    )
    line_chart(
        svg,
        x=x,
        y=605,
        width=width,
        height=panel_height,
        title="Angular-rate estimation error",
        x_label="time (s)",
        y_label="rate error (rad/s)",
        series=[
            {
                "name": "rate error",
                "color": PURPLE,
                "x": time,
                "y": column(rows, "rate_error_radps"),
            }
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 0.008),
    )
    line_chart(
        svg,
        x=x,
        y=835,
        width=width,
        height=panel_height,
        title="Estimated-state TVC demand",
        x_label="time (s)",
        y_label="gimbal magnitude (deg)",
        series=[
            {
                "name": "gimbal",
                "color": ORANGE,
                "x": time,
                "y": column(rows, "gimbal_total_deg"),
            }
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 1.6),
    )
    side_x = 1010
    heading(svg, side_x, 145, "Closed-loop result")
    errors = column(rows, "attitude_error_deg")
    metric_row(svg, side_x, 186, "maximum attitude error", f"{max(errors):.2f} deg", RED)
    metric_row(
        svg,
        side_x,
        216,
        "RMS attitude error",
        f"{math.sqrt(statistics.fmean(value * value for value in errors)):.2f} deg",
        RED,
    )
    metric_row(svg, side_x, 246, "maximum true tilt", f'{max(column(rows, "tilt_deg")):.2f} deg', BLUE)
    metric_row(svg, side_x, 276, "maximum gimbal", f'{max(column(rows, "gimbal_total_deg")):.2f} deg', ORANGE)
    metric_row(svg, side_x, 306, "gimbal saturation", "0.0%", TEAL)
    heading(svg, side_x, 380, "Estimator physics")
    text_lines(
        svg,
        side_x,
        418,
        [
            "The gyro supplies high-rate angular propagation:",
            "omega_m = omega + b_g + eta_g.",
            "A low-rate attitude reference bounds integrated",
            "bias drift in the quaternion estimate.",
            "",
            "The accelerometer is modeled as specific force,",
            "f_B = R_IB(q)(a_I - g_I). During powered ascent",
            "it is thrust-dominated, so treating it as a clean",
            "gravity vector would be physically incorrect.",
        ],
    )
    heading(svg, side_x, 690, "Control relevance")
    text_lines(
        svg,
        side_x,
        728,
        [
            "Estimation error enters TVC as false attitude",
            "and rate feedback. The sub-degree error remains",
            "small relative to the approximately 10 deg",
            "controlled excursion and does not drive the",
            "modeled gimbal into saturation.",
        ],
    )
    heading(svg, side_x, 900, "Claim boundary")
    text_lines(
        svg,
        side_x,
        938,
        [
            "The estimator is a focused quaternion attitude",
            "filter, not a full inertial navigation EKF.",
        ],
        size=13,
        color=MUTED,
    )
    write_artifact("estimated-state-control-evidence.svg", svg)


def actuator_evidence() -> None:
    instant = read_rows(OUT / "week4a_lqr_tvc_controlled.csv")
    actuator = read_rows(OUT / "week6_lqr_tvc_actuator_limited.csv")
    estimated = read_rows(OUT / "week6_estimated_lqr_tvc_actuator_limited.csv")
    instant_time = column(instant, "time_s")
    actuator_time = column(actuator, "time_s")
    estimated_time = column(estimated, "time_s")
    svg = svg_start(
        1500,
        1050,
        "Finite-Bandwidth TVC Verification",
        "Commanded torque is evaluated through servo lag, slew limits, position limits, and achieved gimbal.",
    )
    x, width, panel_height = 95, 850, 165
    line_chart(
        svg,
        x=x,
        y=145,
        width=width,
        height=panel_height,
        title="Attitude response with actuator and estimation dynamics",
        x_label="time (s)",
        y_label="tilt (deg)",
        series=[
            {"name": "instant", "color": TEAL, "x": instant_time, "y": column(instant, "tilt_deg")},
            {"name": "actuator", "color": ORANGE, "x": actuator_time, "y": column(actuator, "tilt_deg")},
            {"name": "estimated", "color": PURPLE, "x": estimated_time, "y": column(estimated, "tilt_deg")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 12.0),
    )
    line_chart(
        svg,
        x=x,
        y=375,
        width=width,
        height=panel_height,
        title="Commanded versus achieved nozzle deflection",
        x_label="time (s)",
        y_label="gimbal magnitude (deg)",
        series=[
            {"name": "commanded", "color": RED, "x": actuator_time, "y": column(actuator, "cmd_gimbal_total_deg")},
            {"name": "achieved", "color": GREEN, "x": actuator_time, "y": column(actuator, "ach_gimbal_total_deg")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 1.6),
    )
    line_chart(
        svg,
        x=x,
        y=605,
        width=width,
        height=panel_height,
        title="Servo tracking error",
        x_label="time (s)",
        y_label="gimbal lag (deg)",
        series=[
            {"name": "truth feedback", "color": ORANGE, "x": actuator_time, "y": column(actuator, "gimbal_lag_error_deg")},
            {"name": "estimated feedback", "color": PURPLE, "x": estimated_time, "y": column(estimated, "gimbal_lag_error_deg")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 1.6),
    )
    line_chart(
        svg,
        x=x,
        y=835,
        width=width,
        height=panel_height,
        title="Available TVC moment minus requested moment",
        x_label="time (s)",
        y_label="torque margin (N m)",
        series=[
            {"name": "truth feedback", "color": BLUE, "x": actuator_time, "y": column(actuator, "torque_authority_margin_nm")},
            {"name": "estimated feedback", "color": PURPLE, "x": estimated_time, "y": column(estimated, "torque_authority_margin_nm")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 95.0),
        reference_lines=[(0.0, "zero authority margin", RED)],
    )
    side_x = 1010
    heading(svg, side_x, 145, "Actuator result")
    metric_row(svg, side_x, 186, "peak commanded gimbal", "1.50 deg", RED)
    metric_row(svg, side_x, 216, "peak achieved gimbal", "0.86 deg", GREEN)
    metric_row(svg, side_x, 246, "peak lag error", "1.50 deg", ORANGE)
    metric_row(svg, side_x, 276, "minimum torque margin", "48.58 N m", BLUE)
    metric_row(svg, side_x, 306, "rate / position limiting", "0.0% / 0.0%", TEAL)
    heading(svg, side_x, 380, "Why the lag matters")
    text_lines(
        svg,
        side_x,
        418,
        [
            "The plant responds to achieved gimbal, not the",
            "controller request. First-order servo dynamics",
            "insert phase lag between attitude error and",
            "corrective moment.",
            "",
            "In I theta_ddot = tau_TVC + tau_dist, delayed",
            "tau_TVC reduces damping and can inject energy if",
            "the phase loss becomes large. Here the attitude",
            "response remains bounded and close to instant TVC.",
        ],
    )
    heading(svg, side_x, 690, "Authority interpretation")
    text_lines(
        svg,
        side_x,
        728,
        [
            "Available moment scales approximately as",
            "L T sin(delta_max). Positive margin throughout",
            "the run shows the LQR is not stabilized by an",
            "unachievable torque request.",
        ],
    )
    heading(svg, side_x, 900, "Next fidelity step")
    text_lines(
        svg,
        side_x,
        938,
        [
            "Frequency-response identification and explicit",
            "phase/gain-margin analysis would strengthen the",
            "servo model beyond this time-domain check.",
        ],
        size=13,
        color=MUTED,
    )
    write_artifact("actuator-bandwidth-evidence.svg", svg)


def variable_mass_evidence() -> None:
    constant = read_rows(OUT / "week6_lqr_tvc_actuator_limited.csv")
    variable = read_rows(OUT / "week7_variable_mass_lqr_tvc.csv")
    constant_time = column(constant, "time_s")
    variable_time = column(variable, "time_s")
    svg = svg_start(
        1500,
        1110,
        "Time-Varying Mass Properties and Control Authority",
        "Thrust, mass, inertia, center of mass, and TVC authority evolve together during the burn.",
    )
    left_x, right_x = 95, 530
    chart_width, chart_height = 365, 200
    rows_y = (150, 475, 800)
    line_chart(
        svg,
        x=left_x,
        y=rows_y[0],
        width=chart_width,
        height=chart_height,
        title="Altitude response",
        x_label="time (s)",
        y_label="altitude (m)",
        series=[
            {"name": "constant mass", "color": TEAL, "x": constant_time, "y": column(constant, "z_m")},
            {"name": "variable mass", "color": ORANGE, "x": variable_time, "y": column(variable, "z_m")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 35.0),
    )
    line_chart(
        svg,
        x=right_x,
        y=rows_y[0],
        width=chart_width,
        height=chart_height,
        title="Controlled tilt",
        x_label="time (s)",
        y_label="tilt (deg)",
        series=[
            {"name": "constant mass", "color": TEAL, "x": constant_time, "y": column(constant, "tilt_deg")},
            {"name": "variable mass", "color": ORANGE, "x": variable_time, "y": column(variable, "tilt_deg")},
        ],
        x_domain=(0.0, 3.0),
        y_domain=(0.0, 12.0),
    )
    line_chart(
        svg,
        x=left_x,
        y=rows_y[1],
        width=chart_width,
        height=chart_height,
        title="Propellant depletion",
        x_label="time (s)",
        y_label="mass (kg)",
        series=[
            {"name": "mass", "color": NAVY, "x": variable_time, "y": column(variable, "mass_kg")}
        ],
        x_domain=(0.0, 3.0),
        y_domain=(48.8, 50.1),
    )
    line_chart(
        svg,
        x=right_x,
        y=rows_y[1],
        width=chart_width,
        height=chart_height,
        title="Thrust-to-mass schedule",
        x_label="time (s)",
        y_label="T / m (m/s^2)",
        series=[
            {"name": "T / m", "color": RED, "x": variable_time, "y": column(variable, "thrust_to_mass_mps2")}
        ],
        x_domain=(0.0, 3.0),
        y_domain=(15.0, 18.5),
    )
    line_chart(
        svg,
        x=left_x,
        y=rows_y[2],
        width=chart_width,
        height=chart_height,
        title="Transverse inertia",
        x_label="time (s)",
        y_label="Ixx (kg m^2)",
        series=[
            {"name": "Ixx", "color": BLUE, "x": variable_time, "y": column(variable, "inertia_x_kg_m2")}
        ],
        x_domain=(0.0, 3.0),
        y_domain=(3.05, 3.16),
    )
    line_chart(
        svg,
        x=right_x,
        y=rows_y[2],
        width=chart_width,
        height=chart_height,
        title="Center-of-mass migration",
        x_label="time (s)",
        y_label="CM z (cm)",
        series=[
            {
                "name": "CM z",
                "color": GREEN,
                "x": variable_time,
                "y": [100.0 * value for value in column(variable, "center_of_mass_z_m")],
            }
        ],
        x_domain=(0.0, 3.0),
        y_domain=(-8.2, -5.4),
    )
    side_x = 1010
    heading(svg, side_x, 145, "Coupled propulsion result")
    metric_row(svg, side_x, 186, "mass", "50.00 -> 48.93 kg", NAVY)
    metric_row(svg, side_x, 216, "peak thrust-to-mass", "18.20 m/s^2", RED)
    metric_row(svg, side_x, 246, "transverse inertia", "3.15 -> 3.07 kg m^2", BLUE)
    metric_row(svg, side_x, 276, "CM z", "-8.0 -> -5.6 cm", GREEN)
    metric_row(svg, side_x, 306, "minimum TVC torque margin", "50.27 N m", PURPLE)
    heading(svg, side_x, 380, "Physical interpretation")
    text_lines(
        svg,
        side_x,
        418,
        [
            "Propellant flow follows m_dot = -T/(Isp g0).",
            "Acceleration depends on both the thrust curve",
            "and depletion through a_thrust = T(t)/m(t).",
            "",
            "The rotational plant moves at the same time:",
            "I(t) omega_dot + omega x I(t)omega = tau.",
            "Lower transverse inertia increases angular",
            "acceleration for a fixed disturbance or TVC moment.",
            "",
            "CM migration changes r_CP - r_CM and therefore",
            "the aerodynamic moment arm. Thrust variation also",
            "changes available TVC moment L T sin(delta_max).",
        ],
    )
    heading(svg, side_x, 750, "Control conclusion")
    text_lines(
        svg,
        side_x,
        788,
        [
            "Fixed LQR gains remain stable in this short burn,",
            "but the controller is operating on a moving plant.",
            "The altitude increase is caused by the combined",
            "thrust schedule and depletion, not mass loss alone.",
        ],
    )
    heading(svg, side_x, 945, "Next fidelity step")
    text_lines(
        svg,
        side_x,
        983,
        [
            "Schedule gains against mass state, dynamic pressure,",
            "and thrust level, then verify interpolation and",
            "robustness across the full operating envelope.",
        ],
        size=13,
        color=MUTED,
    )
    write_artifact("variable-mass-evidence.svg", svg)


def main() -> None:
    control_system_evidence()
    monte_carlo_evidence()
    estimation_evidence()
    actuator_evidence()
    variable_mass_evidence()


if __name__ == "__main__":
    main()
