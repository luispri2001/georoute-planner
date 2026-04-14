"""Pose utilities for route points (bearing, quaternion, CSV export)."""

import math
import os

import pandas as pd


def calculate_bearing(p1, p2):
    """Calculate heading in radians from p1 [lat, lon] to p2 [lat, lon]."""
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])

    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )

    return math.atan2(y, x)


def yaw_to_quaternion(yaw):
    """Convert yaw (radians) into quaternion tuple (qx, qy, qz, qw)."""
    half_yaw = yaw / 2.0
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return (qx, qy, qz, qw)


def _normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _bearing_to_enu_yaw(bearing):
    """Convert north-clockwise bearing to ENU yaw.

    ENU uses east=0 with counterclockwise positive angles.
    """
    return _normalize_angle((math.pi / 2.0) - bearing)


def add_pose_data(route, z_height=0.0, yaw_convention="north_cw"):
    """Attach x/y/z and quaternion orientation to each [lat, lon] route point.

    yaw_convention:
    - "north_cw": bearing-like yaw (north=0, clockwise positive)
    - "enu": ROS-style ENU yaw (east=0, counterclockwise positive)
    """
    if not route:
        return []

    if len(route) == 1:
        lat, lon = route[0]
        return [{
            "x": lon,
            "y": lat,
            "z": z_height,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        }]

    pose_route = []

    for i, (lat, lon) in enumerate(route):
        # Forward heading for all points except the last point.
        # The last point reuses the previous segment heading.
        if i < len(route) - 1:
            bearing = calculate_bearing(route[i], route[i + 1])
        else:
            bearing = calculate_bearing(route[i - 1], route[i])

        if yaw_convention == "north_cw":
            yaw = bearing
        elif yaw_convention == "enu":
            yaw = _bearing_to_enu_yaw(bearing)
        else:
            raise ValueError(
                "Unsupported yaw_convention. Use 'north_cw' or 'enu'."
            )

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        pose_route.append({
            "x": lon,
            "y": lat,
            "z": z_height,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
        })

    return pose_route


def export_pose_route_csv(pose_route, filename):
    """Export pose route dictionaries to CSV (x, y, z, qx, qy, qz, qw)."""
    os.makedirs(os.path.dirname(filename), exist_ok=True)

    dataframe = pd.DataFrame(pose_route)
    dataframe.to_csv(filename, index=False)

    return filename
