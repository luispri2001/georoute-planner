#!/usr/bin/env python3
"""Visual debug map for route pose headings.

Reads a pose CSV (x, y, z, qx, qy, qz, qw) and generates an HTML map with:
- route points
- route polyline
- heading arrows from quaternion using two conventions
- popup diagnostics per point

This helps detect yaw convention mismatches.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import folium


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (radians) from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_deg(angle: float) -> float:
    """Normalize angle in degrees to [0, 360)."""
    return angle % 360.0


def smallest_angle_diff_deg(a: float, b: float) -> float:
    """Smallest absolute difference between two headings in degrees."""
    diff = abs((a - b + 180.0) % 360.0 - 180.0)
    return diff


def bearing_deg_from_latlon(
    lat1: float, lon1: float, lat2: float, lon2: float
) -> float:
    """Forward bearing from point1 to point2 in degrees.

    Heading convention: north=0 degrees, clockwise positive.
    """
    lat1r = math.radians(lat1)
    lon1r = math.radians(lon1)
    lat2r = math.radians(lat2)
    lon2r = math.radians(lon2)

    dlon = lon2r - lon1r
    y = math.sin(dlon) * math.cos(lat2r)
    x = (
        math.cos(lat1r) * math.sin(lat2r)
        - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    )
    return normalize_deg(math.degrees(math.atan2(y, x)))


def endpoint_from_heading_north_cw(
    lat: float, lon: float, heading_deg: float, meters: float
) -> tuple[float, float]:
    """Approximate endpoint given heading in north-clockwise convention."""
    theta = math.radians(heading_deg)
    d_north = meters * math.cos(theta)
    d_east = meters * math.sin(theta)

    dlat = d_north / 111320.0
    dlon = d_east / (111320.0 * max(math.cos(math.radians(lat)), 1e-9))
    return lat + dlat, lon + dlon


def load_pose_rows(input_csv: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with input_csv.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "qx", "qy", "qz", "qw"}
        if not reader.fieldnames or not required.issubset(
            set(reader.fieldnames)
        ):
            missing = sorted(required - set(reader.fieldnames or []))
            raise ValueError(f"Missing required columns: {', '.join(missing)}")

        for row in reader:
            rows.append(
                {
                    "lon": float(row["x"]),
                    "lat": float(row["y"]),
                    "qx": float(row["qx"]),
                    "qy": float(row["qy"]),
                    "qz": float(row["qz"]),
                    "qw": float(row["qw"]),
                }
            )
    return rows


def build_debug_map(
    rows: list[dict[str, float]], output_html: Path, arrow_m: float
) -> None:
    if not rows:
        raise ValueError("Input CSV has no rows.")

    center = [rows[0]["lat"], rows[0]["lon"]]
    fmap = folium.Map(location=center, zoom_start=19, tiles="OpenStreetMap")

    latlon_route = [[row["lat"], row["lon"]] for row in rows]
    folium.PolyLine(
        latlon_route,
        color="black",
        weight=3,
        opacity=0.8,
        tooltip="Route",
    ).add_to(fmap)

    for idx, row in enumerate(rows):
        lat = row["lat"]
        lon = row["lon"]

        yaw_rad = quaternion_to_yaw(row["qx"], row["qy"], row["qz"], row["qw"])
        yaw_deg_raw = normalize_deg(math.degrees(yaw_rad))

        # Interpretation A: yaw already represents north-clockwise heading.
        heading_a = yaw_deg_raw

        # Interpretation B: yaw is east-counterclockwise.
        heading_b = normalize_deg(90.0 - yaw_deg_raw)

        if idx < len(rows) - 1:
            next_row = rows[idx + 1]
            bearing_route = bearing_deg_from_latlon(
                lat, lon, next_row["lat"], next_row["lon"]
            )
        else:
            prev_row = rows[idx - 1]
            bearing_route = bearing_deg_from_latlon(
                prev_row["lat"], prev_row["lon"], lat, lon
            )

        diff_a = smallest_angle_diff_deg(heading_a, bearing_route)
        diff_b = smallest_angle_diff_deg(heading_b, bearing_route)

        end_a = endpoint_from_heading_north_cw(lat, lon, heading_a, arrow_m)
        end_b = endpoint_from_heading_north_cw(lat, lon, heading_b, arrow_m)

        popup_html = (
            f"<b>Point</b>: {idx}<br>"
            f"<b>lat/lon</b>: {lat:.8f}, {lon:.8f}<br>"
            f"<b>yaw rad</b>: {yaw_rad:.8f}<br>"
            f"<b>yaw deg raw</b>: {yaw_deg_raw:.3f}<br>"
            f"<b>route bearing</b>: {bearing_route:.3f} deg<br>"
            f"<b>diff A (north-cw)</b>: {diff_a:.3f} deg<br>"
            f"<b>diff B (east-ccw->north-cw)</b>: {diff_b:.3f} deg"
        )

        folium.CircleMarker(
            location=[lat, lon],
            radius=4,
            color="black",
            fill=True,
            fill_opacity=0.9,
            popup=folium.Popup(popup_html, max_width=320),
        ).add_to(fmap)

        folium.PolyLine(
            [[lat, lon], [end_a[0], end_a[1]]],
            color="red",
            weight=2,
            opacity=0.9,
            tooltip=f"A north-cw idx={idx}",
        ).add_to(fmap)

        folium.PolyLine(
            [[lat, lon], [end_b[0], end_b[1]]],
            color="blue",
            weight=2,
            opacity=0.9,
            tooltip=f"B east-ccw idx={idx}",
        ).add_to(fmap)

    output_html.parent.mkdir(parents=True, exist_ok=True)
    fmap.save(output_html)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize quaternion yaw vs route direction on a map."
    )
    parser.add_argument(
        "input_csv",
        nargs="?",
        default="routes/robot_route_with_poses.csv",
        help="Input pose CSV path.",
    )
    parser.add_argument(
        "output_html",
        nargs="?",
        default="maps/pose_heading_debug.html",
        help="Output debug HTML map path.",
    )
    parser.add_argument(
        "--arrow-m",
        type=float,
        default=6.0,
        help="Arrow length in meters for heading vectors.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_pose_rows(Path(args.input_csv))
    build_debug_map(rows, Path(args.output_html), args.arrow_m)
    print(f"Debug map generated at: {args.output_html}")
    print("Legend: red=A(north-cw), blue=B(east-ccw interpreted)")


if __name__ == "__main__":
    main()
