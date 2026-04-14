#!/usr/bin/env python3
"""
Simplified route generation script that generates routes with pose information
without relying on OSMnx (which was causing network timeouts).
"""

import os
import sys

import pandas as pd

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))


def main():
    from robot_route_generation import (
        add_pose_data,
        export_pose_route_csv,
        generate_robot_route,
        get_point_by_id,
        load_waypoints,
    )

    print("=== Georoute Planner with Pose Information ===")

    # 1. Load the waypoints
    poi_geojson = load_waypoints()

    # 2. Define the route
    route_ids = ["water_1", "arbustivo_2", "water_2"]
    robot_waypoints = [get_point_by_id(poi_geojson, rid) for rid in route_ids]
    robot_waypoints = [point for point in robot_waypoints if point]

    print(f"Visit order: {route_ids}")
    print(f"Waypoints: {len(robot_waypoints)}")
    for i, waypoint in enumerate(robot_waypoints):
        print(
            f"  {route_ids[i]}: "
            f"lat={waypoint[0]:.6f}, lon={waypoint[1]:.6f}"
        )

    # 3. Generate direct route (straight-line interpolation)
    interval = 10  # 10 meters between points
    print(f"\nGenerating direct route with {interval}m intervals...")
    direct_route = generate_robot_route(robot_waypoints, interval_m=interval)
    print(
        "Generated route: "
        f"{len(direct_route)} points, approx {len(direct_route) * interval}m"
    )

    # 4. Add pose information
    print("\nAdding pose information...")
    pose_route = add_pose_data(
        direct_route,
        z_height=0.0,
        yaw_convention="enu",
    )
    print(f"Generated poses: {len(pose_route)} points")

    # 5. Export routes
    os.makedirs("routes", exist_ok=True)

    # Export with pose information
    pose_csv = "routes/robot_route_with_poses.csv"
    export_pose_route_csv(pose_route, pose_csv)

    # Export legacy format for compatibility
    legacy_csv = "routes/robot_route_legacy.csv"
    pd.DataFrame(
        direct_route,
        columns=["latitude", "longitude"],
    ).to_csv(legacy_csv, index=False)

    print("\n=== Routes Exported ===")
    print(f"Route with poses: {pose_csv} ({len(pose_route)} points)")
    print(f"Legacy lat/lon route: {legacy_csv} ({len(direct_route)} points)")

    # 6. Show sample pose data
    print("\n=== Sample Pose Data ===")
    for i in [0, len(pose_route) // 2, -1]:  # First, middle, and last points
        pose = pose_route[i]
        print(f"Point {i + 1 if i >= 0 else len(pose_route)}:")
        print(f"  x: {pose['x']:.6f}, y: {pose['y']:.6f}, z: {pose['z']:.1f}")
        print(
            "  "
            f"qx: {pose['qx']:.6f}, qy: {pose['qy']:.6f}, "
            f"qz: {pose['qz']:.6f}, qw: {pose['qw']:.6f}"
        )

    print("\nRoute generation with pose information completed successfully!")


if __name__ == "__main__":
    main()
