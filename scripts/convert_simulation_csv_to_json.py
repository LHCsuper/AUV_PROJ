#!/usr/bin/env python3
"""Convert AUV simulation CSV files to Web APP JSON data."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Sequence


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
    "xCommandMax": 80.0,
    "xCommandMin": -80.0,
    "nCommandMax": 20.0,
    "nCommandMin": -20.0,
}

FIELD_MAP = {
    "time": "time",
    "x": "x",
    "y": "y",
    "psi": "psi",
    "u": "u",
    "v": "v",
    "r": "r",
    "x_ref": "xRef",
    "y_ref": "yRef",
    "psi_d": "psiDesired",
    "cross_track_error": "crossTrackError",
    "heading_error": "headingError",
    "X_cmd": "xCommand",
    "N_cmd": "nCommand",
    "segment_index": "segmentIndex",
}


SCENARIO_META = {
    "straight": {
        "name": "直线路径跟随能力演示",
        "description": "展示 AUV 单体控制算法模块对直线路径任务的跟随能力。",
        "conclusion": "该演示表明：AUV 单体控制模块能够在直线路径任务中稳定生成控制输入，并使航行器逐渐贴近参考路径。",
    },
    "circle": {
        "name": "圆形路径跟随能力演示",
        "description": "展示 AUV 单体控制算法模块对连续曲线路径的跟随能力。",
        "conclusion": "该演示表明：模块能够处理连续转向路径，控制输出保持在限幅范围内，轨迹能够围绕目标路径运行。",
    },
    "s_curve": {
        "name": "S 形路径跟随能力演示",
        "description": "展示 AUV 单体控制算法模块对方向连续变化路径的跟随能力。",
        "conclusion": "该演示表明：模块能够处理方向连续变化的机动路径，具备基础路径跟随展示能力。",
    },
}


def read_rows(path: Path) -> List[Dict[str, float]]:
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")

    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        fields = reader.fieldnames or []
        missing = [field for field in REQUIRED_FIELDS if field not in fields]
        if missing:
            raise ValueError(f"{path} is missing required fields: {', '.join(missing)}")

        rows: List[Dict[str, float]] = []
        for row in reader:
            parsed: Dict[str, float] = {}
            for source, target in FIELD_MAP.items():
                if source not in row or row[source] == "":
                    continue
                parsed[target] = float(row[source])
            rows.append(parsed)

    if not rows:
        raise ValueError(f"CSV file has no data rows: {path}")
    return rows


def column(rows: Sequence[Dict[str, float]], field: str) -> List[float]:
    return [row[field] for row in rows if field in row and math.isfinite(row[field])]


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


def saturation_ratio(values: Sequence[float], lower: float, upper: float) -> float:
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    tolerance = 1e-9
    saturated = sum(1 for value in finite if value <= lower + tolerance or value >= upper - tolerance)
    return saturated / len(finite)


def time_step(rows: Sequence[Dict[str, float]]) -> float:
    times = column(rows, "time")
    if len(times) < 2:
        return 0.0
    return max(0.0, times[1] - times[0])


def summary(rows: Sequence[Dict[str, float]]) -> Dict[str, float | str]:
    dt = time_step(rows)
    x_command = column(rows, "xCommand")
    n_command = column(rows, "nCommand")
    energy = sum(
        ((x / DEFAULT_LIMITS["xCommandMax"]) ** 2 + (n / DEFAULT_LIMITS["nCommandMax"]) ** 2) * dt
        for x, n in zip(x_command, n_command)
        if math.isfinite(x) and math.isfinite(n)
    )

    return {
        "rmsCrossTrackError": rms(column(rows, "crossTrackError")),
        "maxAbsCrossTrackError": max_abs(column(rows, "crossTrackError")),
        "rmsHeadingErrorRad": rms(column(rows, "headingError")),
        "maxAbsHeadingErrorRad": max_abs(column(rows, "headingError")),
        "meanSpeed": mean(column(rows, "u")),
        "maxAbsXCommand": max_abs(x_command),
        "maxAbsNCommand": max_abs(n_command),
        "energyProxy": energy,
        "xSaturationRatio": saturation_ratio(
            x_command,
            DEFAULT_LIMITS["xCommandMin"],
            DEFAULT_LIMITS["xCommandMax"],
        ),
        "nSaturationRatio": saturation_ratio(
            n_command,
            DEFAULT_LIMITS["nCommandMin"],
            DEFAULT_LIMITS["nCommandMax"],
        ),
        "status": "normal",
    }


def build_payload(scenario_id: str, rows: List[Dict[str, float]]) -> Dict[str, object]:
    meta = SCENARIO_META[scenario_id]
    return {
        "scenario": {
            "id": scenario_id,
            "name": meta["name"],
            "description": meta["description"],
            "conclusion": meta["conclusion"],
            "controller": "LOS + PID",
            "model": "horizontal 3-DOF underactuated AUV",
            "actuation": "tau_h = [X, 0, N]^T",
            "current": "none",
            "parameterSet": "nominal",
        },
        "limits": DEFAULT_LIMITS,
        "summary": summary(rows),
        "series": rows,
    }


def convert_file(input_path: Path, output_path: Path, scenario_id: str) -> None:
    rows = read_rows(input_path)
    payload = build_payload(scenario_id, rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"Generated {output_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scenario", required=True, choices=sorted(SCENARIO_META))
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    convert_file(args.input, args.output, args.scenario)
