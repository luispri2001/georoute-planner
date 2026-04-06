#!/usr/bin/env python3
"""Simple test script to verify pose generation functionality."""

import math
import os
import sys

import pandas as pd

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from robot_route_generation import add_pose_data, export_pose_route_csv


def test_pose_generation():
    """Test the pose generation with a simple route."""
    print("Testing pose information generation...")

    # Create a simple test route (lat, lon points)
    test_route = [
        [42.310665, -6.207228],  # Starting point
        [42.310832, -6.203042],  # Middle point
        [42.312561, -6.204347],  # End point
    ]

    print(f"Original route: {len(test_route)} waypoints")
    for i, point in enumerate(test_route):
        print(f"  Point {i + 1}: lat={point[0]:.6f}, lon={point[1]:.6f}")

    # Generate pose data
    pose_route = add_pose_data(test_route, z_height=0.0)

    print(f"\nGenerated poses: {len(pose_route)} points")

    # Display sample poses
    for i, pose in enumerate(pose_route):
        print(f"Point {i + 1}:")
        print(f"  Position: x={pose['x']:.6f}, y={pose['y']:.6f}, z={pose['z']:.1f}")
        print(f"  Orientation: qx={pose['qx']:.6f}, qy={pose['qy']:.6f}, qz={pose['qz']:.6f}, qw={pose['qw']:.6f}")

        # Convert quaternion back to yaw in degrees for readability.
        yaw_rad = math.atan2(
            2 * (pose['qw'] * pose['qz'] + pose['qx'] * pose['qy']),
            1 - 2 * (pose['qy'] ** 2 + pose['qz'] ** 2),
        )
        yaw_deg = math.degrees(yaw_rad)
        print(f"  Heading: {yaw_deg:.1f}deg")
        print()

    # Test export to CSV
    test_filename = "routes/test_poses.csv"
    os.makedirs("routes", exist_ok=True)

    export_pose_route_csv(pose_route, test_filename)
    print(f"Exported test poses to: {test_filename}")

    # Read back and verify
    dataframe = pd.read_csv(test_filename)
    print(f"\nCSV verification - shape: {dataframe.shape}")
    print("CSV columns:", list(dataframe.columns))
    print("\nFirst few rows:")
    print(dataframe.head())

    return pose_route


if __name__ == "__main__":
    test_pose_generation()
    print("\nPose generation test completed successfully!")