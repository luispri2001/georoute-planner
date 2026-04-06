"""Main route generation script and compatibility exports.

This file now orchestrates specialized modules:
- waypoint_data: POI loading and lookup
- route_generation: direct and OSMnx routes
- costmap_routing: costmap+A* route
- pose_generation: heading/quaternion and pose CSV export
"""

import os

import folium
import pandas as pd

from costmap_routing import generate_costmap_route
from pose_generation import (
    add_pose_data,
    calculate_bearing,
    export_pose_route_csv,
    yaw_to_quaternion,
)
from route_generation import (
    generate_osmnx_route,
    generate_osmnx_route_bis,
    generate_robot_route,
    interpolate_points,
)
from waypoint_data import get_point_by_id, load_waypoints


# Keep explicit re-export list for scripts that import from this module.
__all__ = [
    "add_pose_data",
    "calculate_bearing",
    "export_pose_route_csv",
    "generate_costmap_route",
    "generate_osmnx_route",
    "generate_osmnx_route_bis",
    "generate_robot_route",
    "get_point_by_id",
    "interpolate_points",
    "load_waypoints",
    "main",
    "yaw_to_quaternion",
]


def main(filepath="data/points.geojson", route_ids=None):
    """Generate direct, OSM, and costmap routes and export CSV/map outputs."""
    poi_geojson = load_waypoints(filepath=filepath)

    # Default route order used by existing examples.
    if route_ids is None:
        route_ids = ["water_1", "arbustivo_2", "water_2"]

    robot_waypoints = [get_point_by_id(poi_geojson, route_id) for route_id in route_ids]
    robot_waypoints = [point for point in robot_waypoints if point]

    if not robot_waypoints:
        raise ValueError("No valid waypoint IDs were provided in route_ids.")

    print(f"Visit order: {route_ids}")
    print(f"Generating routes for {len(robot_waypoints)} waypoints...")

    interval = 10

    print("\n--- Option A: Direct Route (Straight Lines) ---")
    direct_route = generate_robot_route(robot_waypoints, interval_m=interval)
    print(f"Direct Route: {len(direct_route)} points, approx {round(len(direct_route) * interval, 2)}m")

    print("\n--- Option B: Road-based Route (OSMnx) ---")
    road_route = generate_osmnx_route(robot_waypoints, network_type="walk", interval_m=interval)
    print(f"Road Route: {len(road_route)} points, approx {round(len(road_route) * interval, 2)}m")

    print("\n--- Option C: OSM-first Route with A* Fallback ---")
    cost_route = generate_costmap_route(poi_geojson, robot_waypoints)
    print(f"Cost Route: {len(cost_route)} points")

    # Map output with satellite imagery background.
    satellite_url = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    satellite_attr = (
        "&copy; Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, "
        "Aerogrid, IGN, IGP, UPR-EBP, and the GIS User Community"
    )

    map_obj = folium.Map(
        location=robot_waypoints[0],
        zoom_start=17,
        tiles=satellite_url,
        attr=satellite_attr,
    )

    colors = {"pastizal": "green", "arbustivo": "orange", "matorral": "purple", "water": "blue"}
    for feature in poi_geojson["features"]:
        lon, lat = feature["geometry"]["coordinates"]
        category = feature["properties"]["landcover"]
        feature_id = feature["id"]

        folium.Marker(
            location=[lat, lon],
            popup=f"ID: {feature_id}<br>Landcover: {category}",
            icon=folium.Icon(color=colors.get(category, "gray"), icon="info-sign"),
        ).add_to(map_obj)

    folium.PolyLine(
        direct_route,
        color="red",
        weight=2,
        opacity=0.5,
        dash_array="5, 5",
        tooltip="Direct Route",
    ).add_to(map_obj)

    folium.PolyLine(
        road_route,
        color="blue",
        weight=4,
        opacity=0.8,
        tooltip="Road-following Route",
    ).add_to(map_obj)

    folium.PolyLine(
        cost_route,
        color="green",
        weight=4,
        opacity=0.8,
        tooltip="Costmap Route",
    ).add_to(map_obj)

    for point in road_route:
        folium.CircleMarker(location=point, radius=2, color="blue", fill=True, fill_opacity=0.7).add_to(map_obj)
    for point in direct_route:
        folium.CircleMarker(location=point, radius=1, color="red", fill=True, fill_opacity=0.5).add_to(map_obj)
    for point in cost_route:
        folium.CircleMarker(location=point, radius=3, color="green", fill=True, fill_opacity=0.7).add_to(map_obj)

    map_output = os.path.join("maps", "robot_route_satellite_points.html")
    os.makedirs("maps", exist_ok=True)
    map_obj.save(map_output)
    print(f"\nMap with dual options and points saved to {map_output}")

    print("\n--- Generating Routes with Pose Information ---")
    road_pose_route = add_pose_data(road_route)
    direct_pose_route = add_pose_data(direct_route)
    cost_pose_route = add_pose_data(cost_route)

    os.makedirs("routes", exist_ok=True)

    road_pose_csv = os.path.join("routes", "route_road_osmnx_poses.csv")
    export_pose_route_csv(road_pose_route, road_pose_csv)

    direct_pose_csv = os.path.join("routes", "route_direct_straight_poses.csv")
    export_pose_route_csv(direct_pose_route, direct_pose_csv)

    cost_pose_csv = os.path.join("routes", "route_costmap_astar_poses.csv")
    export_pose_route_csv(cost_pose_route, cost_pose_csv)

    road_csv = os.path.join("routes", "route_road_osmnx.csv")
    pd.DataFrame(road_route, columns=["latitude", "longitude"]).to_csv(road_csv, index=False)

    direct_csv = os.path.join("routes", "route_direct_straight.csv")
    pd.DataFrame(direct_route, columns=["latitude", "longitude"]).to_csv(direct_csv, index=False)

    cost_csv = os.path.join("routes", "route_costmap_astar.csv")
    pd.DataFrame(cost_route, columns=["latitude", "longitude"]).to_csv(cost_csv, index=False)

    print("\n--- Pose Routes Exported ---")
    print(f"Costmap Route with poses: {cost_pose_csv} ({len(cost_pose_route)} points)")
    print(f"Road Route with poses: {road_pose_csv} ({len(road_pose_route)} points)")
    print(f"Direct Route with poses: {direct_pose_csv} ({len(direct_pose_route)} points)")

    print("\n--- Legacy Lat/Lon Routes Exported ---")
    print(f"Costmap Route: {cost_csv} ({len(cost_route)} points)")
    print(f"Road Route: {road_csv} ({len(road_route)} points)")
    print(f"Direct Route: {direct_csv} ({len(direct_route)} points)")

    if road_pose_route:
        sample_pose = road_pose_route[0]
        print("\n--- Sample Pose Data (Road Route) ---")
        print(f"First point: x: {sample_pose['x']:.6f}, y: {sample_pose['y']:.6f}, z: {sample_pose['z']:.1f}")
        print(
            "Orientation: "
            f"qx: {sample_pose['qx']:.6f}, qy: {sample_pose['qy']:.6f}, "
            f"qz: {sample_pose['qz']:.6f}, qw: {sample_pose['qw']:.6f}"
        )


if __name__ == "__main__":
    main()
