"""Convert a halted-target g_m2006_trace memory dump into CSV and SVG curves.

Usage: python scripts/plot_m2006_trace.py m2006_trace.bin
The binary layout is checked by static_assert in modules/src/control_task.cpp.
"""

from __future__ import annotations

import argparse
import csv
import struct
from pathlib import Path


SAMPLE = struct.Struct("<I6fhBB")
CAPACITY = 512
FIELDS = (
    "tick",
    "position_target_rad",
    "position_feedback_rad",
    "speed_target_rad_s",
    "speed_feedback_rad_s",
    "position_pid_speed_rad_s",
    "speed_pid_current",
    "current_command",
    "mode",
    "feedback_online",
)


def read_samples(path: Path) -> list[dict[str, float | int]]:
    data = path.read_bytes()
    if len(data) != CAPACITY * SAMPLE.size:
        raise ValueError(f"Expected {CAPACITY * SAMPLE.size} bytes; got {len(data)}")
    rows = [dict(zip(FIELDS, values)) for values in SAMPLE.iter_unpack(data)]
    # The firmware stores a rolling 5.12-second window. Unused entries have tick 0.
    rows = [row for row in rows if row["tick"] != 0]
    rows.sort(key=lambda row: row["tick"])
    return rows


def svg_chart(rows: list[dict[str, float | int]]) -> str:
    width, height = 1000, 820
    left, right = 85, 975
    times = [(row["tick"] - rows[0]["tick"]) / 100.0 for row in rows]
    duration = max(times[-1], 0.01)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font:14px sans-serif;fill:#222}.grid{stroke:#ddd;stroke-width:1}'
        '.axis{stroke:#333;stroke-width:1}</style>',
        '<text x="85" y="30" font-size="20">M2006 PID trace (last 5.12 s)</text>',
    ]

    panels = (
        ("Output position (rad)", 55, 250,
         (("position_target_rad", "#e34a33", "target"),
          ("position_feedback_rad", "#2b6cb0", "feedback"))),
        ("Output speed (rad/s)", 305, 500,
         (("speed_target_rad_s", "#e34a33", "target"),
          ("speed_feedback_rad_s", "#2b6cb0", "feedback"))),
        ("Current command (C610 units)", 555, 750,
         (("current_command", "#3a8f56", "command"),)),
    )

    for title, top, bottom, series in panels:
        values = [float(row[key]) for row in rows for key, _, _ in series]
        low, high = min(values), max(values)
        padding = max((high - low) * 0.1, 0.1)
        low -= padding
        high += padding
        parts.append(f'<text x="{left}" y="{top - 12}">{title}</text>')
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = bottom - fraction * (bottom - top)
            value = low + fraction * (high - low)
            parts.append(f'<line class="grid" x1="{left}" y1="{y:.1f}" '
                         f'x2="{right}" y2="{y:.1f}"/>')
            parts.append(f'<text x="5" y="{y + 5:.1f}">{value:.2f}</text>')
        parts.append(f'<line class="axis" x1="{left}" y1="{bottom}" '
                     f'x2="{right}" y2="{bottom}"/>')
        for index, (key, color, label) in enumerate(series):
            points = " ".join(
                f"{left + t / duration * (right - left):.1f},"
                f"{bottom - (float(row[key]) - low) / (high - low) * (bottom - top):.1f}"
                for t, row in zip(times, rows)
            )
            parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="2" '
                         f'points="{points}"/>')
            x = 650 + index * 150
            parts.append(f'<line x1="{x}" y1="{top - 17}" x2="{x + 20}" '
                         f'y2="{top - 17}" stroke="{color}" stroke-width="3"/>')
            parts.append(f'<text x="{x + 25}" y="{top - 12}">{label}</text>')

    parts.append(f'<text x="{left}" y="790">Time: 0 to {duration:.2f} s; '
                 'feedback_online is also included in the CSV.</text>')
    parts.append('</svg>')
    return "\n".join(parts)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", type=Path, help="GDB binary memory dump")
    args = parser.parse_args()
    rows = read_samples(args.dump)
    if not rows:
        raise SystemExit("No motor control samples found in the dump")
    prefix = args.dump.with_suffix("")
    csv_path = prefix.with_suffix(".csv")
    svg_path = prefix.with_suffix(".svg")
    with csv_path.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    svg_path.write_text(svg_chart(rows), encoding="utf-8")
    print(f"Wrote {len(rows)} samples to {csv_path} and {svg_path}")


if __name__ == "__main__":
    main()
