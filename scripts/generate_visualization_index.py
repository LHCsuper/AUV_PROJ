#!/usr/bin/env python3
"""Generate the visualization index page."""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_REPORTS = [
    ("直线路径跟随", "straight_path_report.html", "基础 LOS + PID 闭环收敛验证"),
    ("圆形路径跟随", "circle_path_report.html", "连续曲线路径跟随能力验证"),
    ("S 形路径跟随", "s_curve_path_report.html", "连续转向路径跟随能力验证"),
]


def build_index(output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    cards = []
    for title, file_name, description in DEFAULT_REPORTS:
        cards.append(
            f"""
      <a class="card" href="{file_name}">
        <h2>{title}</h2>
        <p>{description}</p>
        <span>打开报告</span>
      </a>"""
        )

    html = f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AUV 仿真可视化报告</title>
  <style>
    body {{ margin: 0; font-family: Arial, "Microsoft YaHei", sans-serif; color: #172033; background: #f7f8fb; }}
    header {{ padding: 34px 42px 22px; background: #ffffff; border-bottom: 1px solid #d9dee8; }}
    main {{ max-width: 1080px; margin: 0 auto; padding: 26px 24px 44px; }}
    h1 {{ margin: 0 0 8px; font-size: 30px; }}
    .subtitle {{ margin: 0; color: #5b6472; }}
    .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 16px; }}
    .card {{ display: block; padding: 20px; background: #ffffff; border: 1px solid #d9dee8; border-radius: 8px; text-decoration: none; color: inherit; }}
    .card:hover {{ border-color: #2563eb; box-shadow: 0 8px 22px rgba(37, 99, 235, 0.12); }}
    .card h2 {{ margin: 0 0 10px; font-size: 20px; }}
    .card p {{ min-height: 48px; margin: 0 0 14px; color: #5b6472; line-height: 1.55; }}
    .card span {{ color: #1d4ed8; font-weight: 600; }}
    .note {{ margin-top: 22px; color: #5b6472; font-size: 14px; }}
  </style>
</head>
<body>
  <header>
    <h1>AUV 仿真可视化报告</h1>
    <p class="subtitle">LOS + PID 路径跟随第一版展示结果</p>
  </header>
  <main>
    <div class="grid">
      {"".join(cards)}
    </div>
    <p class="note">报告由 <code>scripts/batch_generate_reports.py</code> 生成，数据来自 <code>test_outputs/</code> 或 <code>results/data/</code>。</p>
  </main>
</body>
</html>
"""
    output_path = output_dir / "index.html"
    output_path.write_text(html, encoding="utf-8")
    print(f"Generated {output_path}")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/visualization"),
        help="Output visualization directory",
    )
    return parser.parse_args()


if __name__ == "__main__":
    build_index(parse_args().output_dir)
