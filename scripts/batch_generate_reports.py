#!/usr/bin/env python3
"""Generate all first-version AUV visualization reports."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

from generate_visualization_index import build_index


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "results" / "data"
VIS_DIR = PROJECT_ROOT / "results" / "visualization"

SCENARIOS = [
    {
        "name": "straight",
        "title": "直线路径跟随报告",
        "source": PROJECT_ROOT / "test_outputs" / "straight_line_path_following.csv",
        "data": DATA_DIR / "straight_line_path_following.csv",
        "report": VIS_DIR / "straight_path_report.html",
    },
    {
        "name": "circle",
        "title": "圆形路径跟随报告",
        "source": PROJECT_ROOT / "test_outputs" / "circle_path_following.csv",
        "data": DATA_DIR / "circle_path_following.csv",
        "report": VIS_DIR / "circle_path_report.html",
    },
    {
        "name": "s-curve",
        "title": "S 形路径跟随报告",
        "source": PROJECT_ROOT / "test_outputs" / "s_curve_path_following.csv",
        "data": DATA_DIR / "s_curve_path_following.csv",
        "report": VIS_DIR / "s_curve_path_report.html",
    },
]


def ensure_inputs() -> None:
    missing = [scenario["source"] for scenario in SCENARIOS if not scenario["source"].exists()]
    if not missing:
        return

    missing_text = "\n".join(f"  - {path}" for path in missing)
    raise FileNotFoundError(
        "Missing simulation CSV files:\n"
        f"{missing_text}\n\n"
        "Run these commands from the project root first:\n"
        r"  .\build\cmake\test_path_following.exe" "\n"
        r"  .\build\cmake\test_curved_path_following.exe"
    )


def copy_data_files() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    for scenario in SCENARIOS:
        shutil.copyfile(scenario["source"], scenario["data"])
        print(f"Copied {scenario['source']} -> {scenario['data']}")


def generate_report(scenario: dict) -> None:
    script = PROJECT_ROOT / "scripts" / "plot_simulation_results.py"
    command = [
        sys.executable,
        str(script),
        "--input",
        str(scenario["data"]),
        "--output",
        str(scenario["report"]),
        "--scenario",
        scenario["name"],
        "--title",
        scenario["title"],
        "--current",
        "none",
        "--parameter-set",
        "nominal",
    ]
    subprocess.run(command, cwd=PROJECT_ROOT, check=True)


def main() -> None:
    ensure_inputs()
    copy_data_files()
    VIS_DIR.mkdir(parents=True, exist_ok=True)

    for scenario in SCENARIOS:
        generate_report(scenario)

    build_index(VIS_DIR)
    print(f"\nOpen this file in a browser:\n  {VIS_DIR / 'index.html'}")


if __name__ == "__main__":
    main()
