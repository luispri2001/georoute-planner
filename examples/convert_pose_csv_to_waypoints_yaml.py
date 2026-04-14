#!/usr/bin/env python3
"""Convert a pose CSV (x, y, z, qx, qy, qz, qw) to waypoints in YAML or CSV.

Input CSV columns expected:
- x (longitude)
- y (latitude)
- qx, qy, qz, qw (quaternion)

Output YAML shape:
waypoints:
- latitude: <float>
  longitude: <float>
  yaw: <float>

Output CSV shape:
latitude,longitude,yaw
<float>,<float>,<float>
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw in radians from quaternion components."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _fmt(value: float, precision: int) -> str:
    """Format float preserving precision while trimming trailing zeros."""
    text = f"{value:.{precision}f}".rstrip("0").rstrip(".")
    if text == "-0":
        return "0"
    return text


def read_waypoints_from_pose_csv(input_csv: Path) -> list[dict[str, float]]:
    waypoints = []

    with input_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "qx", "qy", "qz", "qw"}
        if not reader.fieldnames or not required.issubset(
            set(reader.fieldnames)
        ):
            missing = sorted(required - set(reader.fieldnames or []))
            raise ValueError(f"Missing required columns: {', '.join(missing)}")

        for row in reader:
            lon = float(row["x"])
            lat = float(row["y"])
            qx = float(row["qx"])
            qy = float(row["qy"])
            qz = float(row["qz"])
            qw = float(row["qw"])
            yaw = quaternion_to_yaw(qx=qx, qy=qy, qz=qz, qw=qw)

            waypoints.append({
                "latitude": lat,
                "longitude": lon,
                "yaw": yaw,
            })

    return waypoints


def write_waypoints_yaml(
    waypoints: list[dict[str, float]],
    output_yaml: Path,
    precision: int,
) -> None:
    lines = ["waypoints:"]
    for point in waypoints:
        lines.append(f"- latitude: {_fmt(point['latitude'], precision)}")
        lines.append(f"  longitude: {_fmt(point['longitude'], precision)}")
        lines.append(f"  yaw: {_fmt(point['yaw'], precision)}")

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_waypoints_csv(
    waypoints: list[dict[str, float]],
    output_csv: Path,
    precision: int,
) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    with output_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["latitude", "longitude", "yaw"])
        writer.writeheader()
        for point in waypoints:
            writer.writerow({
                "latitude": _fmt(point["latitude"], precision),
                "longitude": _fmt(point["longitude"], precision),
                "yaw": _fmt(point["yaw"], precision),
            })


def convert_pose_csv_to_waypoints(
    input_csv: Path,
    output_path: Path,
    output_format: str,
    precision: int,
) -> None:
    waypoints = read_waypoints_from_pose_csv(input_csv)

    if output_format == "yaml":
        write_waypoints_yaml(waypoints, output_path, precision)
    elif output_format == "csv":
        write_waypoints_csv(waypoints, output_path, precision)
    else:
        raise ValueError(f"Unsupported output format: {output_format}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert pose CSV (x,y,z,qx,qy,qz,qw) to waypoint YAML or CSV "
            "with yaw extracted from quaternion."
        )
    )
    parser.add_argument(
        "input_csv",
        nargs="?",
        default="routes/robot_route_with_poses.csv",
        help="Path to input CSV (default: routes/robot_route_with_poses.csv)",
    )
    parser.add_argument(
        "output_path",
        nargs="?",
        default="routes/robot_route_waypoints.yaml",
        help=(
            "Path to output file (default: "
            "routes/robot_route_waypoints.yaml)"
        ),
    )
    parser.add_argument(
        "--format",
        choices=("yaml", "csv"),
        default="yaml",
        help="Output format to generate (default: yaml)",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=15,
        help=(
            "Decimal precision for latitude/longitude/yaw output "
            "(default: 15)"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_csv = Path(args.input_csv)
    output_path = Path(args.output_path)

    convert_pose_csv_to_waypoints(
        input_csv=input_csv,
        output_path=output_path,
        output_format=args.format,
        precision=args.precision,
    )
    print(f"{args.format.upper()} generated at: {output_path}")


if __name__ == "__main__":
    main()
