#!/usr/bin/env python3
"""Generate an offline HTML report from one AUV simulation CSV file.

This first visualization version intentionally uses only the Python standard
library, so it can run even before pandas/Plotly are installed.
"""

from __future__ import annotations

import argparse
import csv
import html
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


REQUIRED_FIELDS = [
    "time",
    "x",
    "y",
    "psi",
    "u",
    "cross_track_error",
    "heading_error",
    "X_cmd",
    "N_cmd",
]

DEFAULT_LIMITS = {
    "X_max": 80.0,
    "X_min": -80.0,
    "N_max": 20.0,
    "N_min": -20.0,
}


def read_csv(path: Path) -> Tuple[List[Dict[str, float]], List[str]]:
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")

    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        fields = reader.fieldnames or []
        missing = [field for field in REQUIRED_FIELDS if field not in fields]
        if missing:
            raise ValueError(
                f"CSV file {path} is missing required fields: {', '.join(missing)}"
            )

        rows: List[Dict[str, float]] = []
        for line_number, row in enumerate(reader, start=2):
            parsed: Dict[str, float] = {}
            for field in fields:
                value = row.get(field, "")
                if value == "":
                    parsed[field] = math.nan
                    continue
                try:
                    parsed[field] = float(value)
                except ValueError as exc:
                    raise ValueError(
                        f"CSV file {path} has non-numeric value at line "
                        f"{line_number}, field {field}: {value!r}"
                    ) from exc
            rows.append(parsed)

    if not rows:
        raise ValueError(f"CSV file {path} has no data rows")
    return rows, fields


def column(rows: Sequence[Dict[str, float]], field: str) -> List[float]:
    return [row[field] for row in rows if field in row and math.isfinite(row[field])]


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def unwrap_angles(values: Sequence[float]) -> List[float]:
    if not values:
        return []

    unwrapped = [values[0]]
    offset = 0.0
    previous = values[0]
    for value in values[1:]:
        delta = value - previous
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        unwrapped.append(value + offset)
        previous = value
    return unwrapped


def rms(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def max_abs(values: Sequence[float]) -> float:
    finite = [abs(value) for value in values if math.isfinite(value)]
    return max(finite) if finite else math.nan


def mean(values: Sequence[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else math.nan


def time_step(rows: Sequence[Dict[str, float]]) -> float:
    times = column(rows, "time")
    if len(times) < 2:
        return 0.0
    return max(0.0, times[1] - times[0])


def control_limits(rows: Sequence[Dict[str, float]], fields: Sequence[str]) -> Dict[str, float]:
    limits = dict(DEFAULT_LIMITS)
    for key in limits:
        if key in fields:
            values = column(rows, key)
            if values:
                limits[key] = values[0]
    return limits


def saturation_ratio(values: Sequence[float], lower: float, upper: float) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    tolerance = 1e-9
    saturated = sum(1 for value in finite if value <= lower + tolerance or value >= upper - tolerance)
    return saturated / len(finite)


def compute_summary(rows: Sequence[Dict[str, float]], fields: Sequence[str]) -> List[Tuple[str, str, str]]:
    dt = time_step(rows)
    limits = control_limits(rows, fields)

    cross_track = column(rows, "cross_track_error")
    heading_error_rad = [wrap_to_pi(value) for value in column(rows, "heading_error")]
    heading_error_deg = [math.degrees(value) for value in heading_error_rad]
    u = column(rows, "u")
    x_cmd = column(rows, "X_cmd")
    n_cmd = column(rows, "N_cmd")

    energy = sum(
        ((x / limits["X_max"]) ** 2 + (n / limits["N_max"]) ** 2) * dt
        for x, n in zip(x_cmd, n_cmd)
        if math.isfinite(x) and math.isfinite(n)
    )

    rows_out = [
        ("RMS cross-track error", format_value(rms(cross_track), "m"), "横向路径误差均方根"),
        ("Max abs cross-track error", format_value(max_abs(cross_track), "m"), "最大绝对横向路径误差"),
        ("RMS heading error", format_value(rms(heading_error_deg), "deg"), "航向误差均方根"),
        ("Max abs heading error", format_value(max_abs(heading_error_deg), "deg"), "最大绝对航向误差"),
        ("Mean speed", format_value(mean(u), "m/s"), "平均纵向速度"),
        ("Max |X_cmd|", format_value(max_abs(x_cmd), "N"), "最大纵向推力命令"),
        ("Max |N_cmd|", format_value(max_abs(n_cmd), "N*m"), "最大偏航力矩命令"),
        ("Energy proxy", format_value(energy, "s"), "归一化控制输入平方积分"),
        (
            "X saturation ratio",
            format_percent(saturation_ratio(x_cmd, limits["X_min"], limits["X_max"])),
            "纵向推力触碰限幅比例",
        ),
        (
            "N saturation ratio",
            format_percent(saturation_ratio(n_cmd, limits["N_min"], limits["N_max"])),
            "偏航力矩触碰限幅比例",
        ),
    ]
    return rows_out


def format_value(value: float, unit: str = "") -> str:
    if not math.isfinite(value):
        return "N/A"
    if abs(value) >= 1000.0 or (abs(value) < 0.001 and value != 0.0):
        text = f"{value:.3e}"
    else:
        text = f"{value:.4f}"
    return f"{text} {unit}".strip()


def format_percent(value: float) -> str:
    if not math.isfinite(value):
        return "N/A"
    return f"{100.0 * value:.2f}%"


def scale_values(values: Sequence[float], pixel_min: float, pixel_max: float) -> Tuple[List[float], float, float]:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return [0.5 * (pixel_min + pixel_max) for _ in values], 0.0, 1.0
    value_min = min(finite)
    value_max = max(finite)
    if abs(value_max - value_min) < 1e-12:
        value_min -= 1.0
        value_max += 1.0
    scaled = [
        pixel_min + (value - value_min) * (pixel_max - pixel_min) / (value_max - value_min)
        if math.isfinite(value)
        else math.nan
        for value in values
    ]
    return scaled, value_min, value_max


def polyline(points: Iterable[Tuple[float, float]], color: str, width: float = 2.0, dash: str = "") -> str:
    clean = [(x, y) for x, y in points if math.isfinite(x) and math.isfinite(y)]
    if not clean:
        return ""
    pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in clean)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polyline points="{pts}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>'
    )


def nice_ticks(value_min: float, value_max: float, count: int = 5) -> List[float]:
    if not math.isfinite(value_min) or not math.isfinite(value_max):
        return []
    if count < 2:
        return [value_min]
    if abs(value_max - value_min) < 1e-12:
        return [value_min]
    return [
        value_min + (value_max - value_min) * index / (count - 1)
        for index in range(count)
    ]


def format_tick(value: float) -> str:
    if not math.isfinite(value):
        return ""
    if abs(value) >= 1000.0 or (abs(value) < 0.001 and value != 0.0):
        return f"{value:.2e}"
    if abs(value) >= 100.0:
        return f"{value:.0f}"
    if abs(value) >= 10.0:
        return f"{value:.1f}"
    return f"{value:.2f}"


def axis_ticks(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    pad_left: float,
    pad_top: float,
    plot_w: float,
    plot_h: float,
    tick_count: int = 5,
) -> str:
    parts: List[str] = []
    for tick in nice_ticks(x_min, x_max, tick_count):
        px = pad_left + (tick - x_min) * plot_w / (x_max - x_min)
        y_axis = pad_top + plot_h
        parts.append(f'<line x1="{px:.2f}" y1="{y_axis:.2f}" x2="{px:.2f}" y2="{y_axis + 6:.2f}" class="tick"/>')
        parts.append(f'<text x="{px:.2f}" y="{y_axis + 22:.2f}" text-anchor="middle" class="tick-label">{format_tick(tick)}</text>')
        parts.append(f'<line x1="{px:.2f}" y1="{pad_top:.2f}" x2="{px:.2f}" y2="{y_axis:.2f}" class="grid-line"/>')

    for tick in nice_ticks(y_min, y_max, tick_count):
        py = pad_top + plot_h - (tick - y_min) * plot_h / (y_max - y_min)
        parts.append(f'<line x1="{pad_left - 6:.2f}" y1="{py:.2f}" x2="{pad_left:.2f}" y2="{py:.2f}" class="tick"/>')
        parts.append(f'<text x="{pad_left - 10:.2f}" y="{py + 4:.2f}" text-anchor="end" class="tick-label">{format_tick(tick)}</text>')
        parts.append(f'<line x1="{pad_left:.2f}" y1="{py:.2f}" x2="{pad_left + plot_w:.2f}" y2="{py:.2f}" class="grid-line"/>')
    return "\n".join(parts)


def simple_line_chart(
    title: str,
    description: str,
    x_values: Sequence[float],
    series: Sequence[Tuple[str, Sequence[float], str, str]],
    y_label: str,
    y_description: str,
    height: int = 300,
) -> str:
    width = 920
    pad_left, pad_right, pad_top, pad_bottom = 70, 25, 45, 55
    plot_w = width - pad_left - pad_right
    plot_h = height - pad_top - pad_bottom
    xs, x_min, x_max = scale_values(x_values, pad_left, pad_left + plot_w)

    all_y: List[float] = []
    for _, values, _, _ in series:
        all_y.extend(values)
    ys_all, y_min, y_max = scale_values(all_y, pad_top + plot_h, pad_top)

    y_index = 0
    parts = [
        f'<h3>{html.escape(title)}</h3>',
        f'<p class="chart-note">{html.escape(description)}</p>',
        f'<svg viewBox="0 0 {width} {height}" class="chart-svg">',
        f'<rect x="0" y="0" width="{width}" height="{height}" class="chart-bg"/>',
        axis_ticks(x_min, x_max, y_min, y_max, pad_left, pad_top, plot_w, plot_h),
        f'<line x1="{pad_left}" y1="{pad_top + plot_h}" x2="{pad_left + plot_w}" y2="{pad_top + plot_h}" class="axis"/>',
        f'<line x1="{pad_left}" y1="{pad_top}" x2="{pad_left}" y2="{pad_top + plot_h}" class="axis"/>',
        f'<text x="{pad_left + plot_w * 0.5}" y="{height - 16}" text-anchor="middle" class="axis-title">横坐标: time, simulation time [s]</text>',
        f'<text x="16" y="{pad_top + plot_h * 0.5}" transform="rotate(-90 16 {pad_top + plot_h * 0.5})" text-anchor="middle" class="axis-title">纵坐标: {html.escape(y_description)}</text>',
        f'<text x="{pad_left}" y="{height - 34}" class="axis-label">time range: {x_min:.2f} to {x_max:.2f} s</text>',
        f'<text x="{pad_left}" y="{pad_top - 16}" class="axis-label">{html.escape(y_label)} range: {y_min:.3f} to {y_max:.3f}</text>',
    ]

    for label, values, color, dash in series:
        ys = ys_all[y_index : y_index + len(values)]
        y_index += len(values)
        parts.append(polyline(zip(xs, ys), color=color, width=2.0, dash=dash))

    legend_x = pad_left + 10
    legend_y = pad_top + 16
    for label, _, color, dash in series:
        parts.append(f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 28}" y2="{legend_y}" stroke="{color}" stroke-width="3" stroke-dasharray="{dash}"/>')
        parts.append(f'<text x="{legend_x + 36}" y="{legend_y + 4}" class="legend">{html.escape(label)}</text>')
        legend_y += 20

    parts.append("</svg>")
    return "\n".join(parts)


def trajectory_chart(rows: Sequence[Dict[str, float]]) -> str:
    width, height = 920, 560
    pad = 45
    x = column(rows, "x")
    y = column(rows, "y")
    ref_x = column(rows, "x_ref")
    ref_y = column(rows, "y_ref")

    all_x = x + ref_x
    all_y = y + ref_y
    finite_x = [value for value in all_x if math.isfinite(value)]
    finite_y = [value for value in all_y if math.isfinite(value)]
    x_min, x_max = min(finite_x), max(finite_x)
    y_min, y_max = min(finite_y), max(finite_y)
    span = max(x_max - x_min, y_max - y_min, 1.0)
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    x_min, x_max = x_mid - 0.5 * span, x_mid + 0.5 * span
    y_min, y_max = y_mid - 0.5 * span, y_mid + 0.5 * span

    def sx(value: float) -> float:
        return pad + (value - x_min) * (width - 2 * pad) / (x_max - x_min)

    def sy(value: float) -> float:
        return height - pad - (value - y_min) * (height - 2 * pad) / (y_max - y_min)

    actual = [(sx(a), sy(b)) for a, b in zip(x, y)]
    reference = [(sx(a), sy(b)) for a, b in zip(ref_x, ref_y)]

    parts = [
        "<h3>AUV trajectory</h3>",
        '<p class="chart-note">该图展示 AUV 在惯性坐标系中的实际运动轨迹与参考路径，用于直观看路径跟随是否稳定。</p>',
        f'<svg viewBox="0 0 {width} {height}" class="chart-svg">',
        f'<rect x="0" y="0" width="{width}" height="{height}" class="chart-bg"/>',
        axis_ticks(x_min, x_max, y_min, y_max, pad, pad, width - 2 * pad, height - 2 * pad),
        f'<line x1="{pad}" y1="{height - pad}" x2="{width - pad}" y2="{height - pad}" class="axis"/>',
        f'<line x1="{pad}" y1="{pad}" x2="{pad}" y2="{height - pad}" class="axis"/>',
        polyline(reference, "#64748b", 2.0, "6 5"),
        polyline(actual, "#0f766e", 2.4),
    ]

    if actual:
        start = actual[0]
        end = actual[-1]
        parts.append(f'<circle cx="{start[0]:.2f}" cy="{start[1]:.2f}" r="6" fill="#16a34a"/>')
        parts.append(f'<circle cx="{end[0]:.2f}" cy="{end[1]:.2f}" r="6" fill="#dc2626"/>')

    psi = column(rows, "psi")
    if len(actual) == len(psi):
        step = max(1, len(actual) // 20)
        arrow_length = 16.0
        for (px, py), angle in zip(actual[::step], psi[::step]):
            ex = px + arrow_length * math.cos(angle)
            ey = py - arrow_length * math.sin(angle)
            parts.append(f'<line x1="{px:.2f}" y1="{py:.2f}" x2="{ex:.2f}" y2="{ey:.2f}" class="heading-arrow"/>')

    parts.extend(
        [
            f'<text x="{width * 0.5}" y="{height - 14}" text-anchor="middle" class="axis-title">横坐标: x, inertial-frame position [m]</text>',
            f'<text x="16" y="{height * 0.5}" transform="rotate(-90 16 {height * 0.5})" text-anchor="middle" class="axis-title">纵坐标: y, inertial-frame position [m]</text>',
            f'<text x="{pad}" y="{height - 34}" class="axis-label">x range: {x_min:.2f} to {x_max:.2f} m</text>',
            f'<text x="{pad}" y="24" class="axis-label">y range: {y_min:.2f} to {y_max:.2f} m</text>',
            '<g class="legend-block"><line x1="690" y1="28" x2="730" y2="28" stroke="#64748b" stroke-width="3" stroke-dasharray="6 5"/><text x="740" y="32" class="legend">reference</text></g>',
            '<g class="legend-block"><line x1="690" y1="52" x2="730" y2="52" stroke="#0f766e" stroke-width="3"/><text x="740" y="56" class="legend">actual</text></g>',
            "</svg>",
        ]
    )
    return "\n".join(parts)


def html_table(headers: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    head = "".join(f"<th>{html.escape(header)}</th>" for header in headers)
    body = "\n".join(
        "<tr>" + "".join(f"<td>{html.escape(cell)}</td>" for cell in row) + "</tr>"
        for row in rows
    )
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def scenario_table(args: argparse.Namespace, rows: Sequence[Dict[str, float]]) -> str:
    duration = column(rows, "time")[-1] if rows else math.nan
    speed = mean(column(rows, "u"))
    data = [
        ("Scenario", args.scenario),
        ("Controller", "LOS + PID"),
        ("Model", "horizontal 3-DOF underactuated AUV"),
        ("Actuation", "tau_h = [X, 0, N]^T"),
        ("Current", args.current),
        ("Parameter set", args.parameter_set),
        ("Simulation time", format_value(duration, "s")),
        ("Mean speed", format_value(speed, "m/s")),
        ("Source CSV", str(args.input)),
    ]
    return html_table(["Item", "Content"], data)


def build_report(args: argparse.Namespace) -> None:
    rows, fields = read_csv(args.input)
    limits = control_limits(rows, fields)
    times = column(rows, "time")
    heading_error_deg = [math.degrees(wrap_to_pi(value)) for value in column(rows, "heading_error")]
    psi_deg = [math.degrees(value) for value in unwrap_angles(column(rows, "psi"))]
    psi_d_deg = [math.degrees(value) for value in unwrap_angles(column(rows, "psi_d"))] if "psi_d" in fields else []

    summary = compute_summary(rows, fields)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    charts = [
        trajectory_chart(rows),
        simple_line_chart(
            "Cross-track error",
            "该图展示 AUV 相对参考路径的横向偏差，误差越接近 0 表示路径跟随越好。",
            times,
            [("cross_track_error", column(rows, "cross_track_error"), "#2563eb", "")],
            "error [m]",
            "cross-track path-following error [m]",
        ),
        simple_line_chart(
            "Heading error",
            "该图展示实际航向与 LOS 期望航向之间的误差，用于判断航向控制是否收敛和平稳。",
            times,
            [("heading_error", heading_error_deg, "#9333ea", "")],
            "error [deg]",
            "heading tracking error [deg]",
        ),
        simple_line_chart(
            "Heading angle",
            "该图对比实际航向和期望航向，观察 AUV 是否能持续跟随制导指令。",
            times,
            [("psi", psi_deg, "#0891b2", ""), ("psi_d", psi_d_deg, "#f97316", "6 5")] if psi_d_deg else [("psi", psi_deg, "#0891b2", "")],
            "heading [deg]",
            "actual and desired heading angle [deg]",
        ),
        simple_line_chart(
            "Surge speed",
            "该图展示 AUV 体坐标系纵向速度，用于检查速度控制是否稳定。",
            times,
            [("u", column(rows, "u"), "#16a34a", "")],
            "speed [m/s]",
            "surge velocity u in body frame [m/s]",
        ),
        simple_line_chart(
            "Surge force command",
            "该图展示纵向推力命令及其限幅线，用于判断推进输入是否长期饱和。",
            times,
            [
                ("X_cmd", column(rows, "X_cmd"), "#0f766e", ""),
                ("X_max", [limits["X_max"]] * len(times), "#dc2626", "6 5"),
                ("X_min", [limits["X_min"]] * len(times), "#dc2626", "6 5"),
            ],
            "X [N]",
            "surge force command X [N]",
        ),
        simple_line_chart(
            "Yaw moment command",
            "该图展示偏航力矩命令及其限幅线，用于判断转向控制输入是否平滑、是否触碰饱和。",
            times,
            [
                ("N_cmd", column(rows, "N_cmd"), "#7c3aed", ""),
                ("N_max", [limits["N_max"]] * len(times), "#dc2626", "6 5"),
                ("N_min", [limits["N_min"]] * len(times), "#dc2626", "6 5"),
            ],
            "N [N*m]",
            "yaw moment command N [N*m]",
        ),
    ]

    document = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(args.title)}</title>
  <style>
    body {{ margin: 0; font-family: Arial, "Microsoft YaHei", sans-serif; color: #172033; background: #f7f8fb; }}
    header {{ padding: 28px 36px 18px; background: #ffffff; border-bottom: 1px solid #d9dee8; }}
    main {{ max-width: 1120px; margin: 0 auto; padding: 22px 24px 42px; }}
    h1 {{ margin: 0 0 8px; font-size: 28px; }}
    h2 {{ margin-top: 28px; font-size: 20px; }}
    h3 {{ margin: 22px 0 10px; font-size: 17px; }}
    .subtitle {{ margin: 0; color: #5b6472; }}
    section {{ background: #ffffff; border: 1px solid #d9dee8; border-radius: 8px; padding: 18px; margin-bottom: 18px; }}
    table {{ width: 100%; border-collapse: collapse; font-size: 14px; }}
    th, td {{ border-bottom: 1px solid #e5e8ef; padding: 9px 10px; text-align: left; }}
    th {{ background: #f0f3f8; color: #263245; }}
    .chart-svg {{ width: 100%; height: auto; border: 1px solid #e5e8ef; border-radius: 6px; background: #ffffff; }}
    .chart-bg {{ fill: #ffffff; }}
    .axis {{ stroke: #94a3b8; stroke-width: 1; }}
    .axis-title {{ fill: #172033; font-size: 14px; font-weight: 700; }}
    .axis-label {{ fill: #475569; font-size: 13px; }}
    .chart-note {{ margin: -4px 0 10px; color: #5b6472; font-size: 14px; line-height: 1.55; }}
    .tick {{ stroke: #64748b; stroke-width: 1; }}
    .tick-label {{ fill: #475569; font-size: 12px; }}
    .grid-line {{ stroke: #e2e8f0; stroke-width: 1; }}
    .legend {{ fill: #334155; font-size: 13px; }}
    .heading-arrow {{ stroke: #334155; stroke-width: 1.5; marker-end: url(#arrow); }}
    code {{ background: #eef2f7; padding: 2px 5px; border-radius: 4px; }}
  </style>
</head>
<body>
  <header>
    <h1>{html.escape(args.title)}</h1>
    <p class="subtitle">AUV single-vehicle control simulation report</p>
  </header>
  <main>
    <section>
      <h2>Scenario information</h2>
      {scenario_table(args, rows)}
    </section>
    <section>
      <h2>Summary metrics</h2>
      {html_table(["Metric", "Value", "Meaning"], summary)}
    </section>
    <section>
      <svg width="0" height="0"><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#334155"/></marker></defs></svg>
      {"".join(charts)}
    </section>
  </main>
</body>
</html>
"""
    args.output.write_text(document, encoding="utf-8")
    print(f"Generated {args.output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Input simulation CSV")
    parser.add_argument("--output", required=True, type=Path, help="Output HTML report")
    parser.add_argument("--scenario", required=True, help="Scenario name")
    parser.add_argument("--title", required=True, help="Report title")
    parser.add_argument("--current", default="none", help="Current condition")
    parser.add_argument("--parameter-set", default="nominal", help="Parameter set")
    return parser.parse_args()


if __name__ == "__main__":
    build_report(parse_args())
